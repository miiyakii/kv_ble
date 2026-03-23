/*
 * src/peripheral/main.c — Dongle 2/3 firmware
 *
 * Advertising strategy:
 *   - ID 1 (peer identity), has bonds → undirected adv + FAL (FILTER_CONN + FILTER_SCAN_REQ)
 *     Windows RPA is resolved by the controller via the Resolving List (IRK added during
 *     bonding), so FAL + FILTER_CONN works correctly for bonded Windows peers.
 *   - ID 1, no bonds, pairing_mode=false → idle, red slow blink, wait for SW1
 *   - ID 1, no bonds, pairing_mode=true  → open undirected adv (SW1 long-press)
 *
 * Stale LTK handling (Windows reconnect after firmware reset):
 *   When PIN_OR_KEY_MISSING fires, the controller's Resolving List no longer has the
 *   peer's IRK (lost on reset), so FAL rejected the connection at the link layer.
 *   Fix: reset ID 1 via bt_id_reset() — this changes the local BDA so Windows sees a
 *   brand-new device and initiates fresh pairing without the user needing to delete
 *   the old bond on the PC side. Same approach as nRF Desktop ble_bond.c.
 *
 * Identity map (CONFIG_BT_ID_MAX=3):
 *   ID 0 (BT_ID_DEFAULT) — unused (reserved, nRF Desktop convention)
 *   ID 1 (PEER_ID)       — normal peer / bonding identity
 *   ID 2 (TEMP_ID)       — temporary identity used during bt_id_reset() swap
 *
 * LED states:
 *   red slow blink (500ms) = no bond, waiting for SW1
 *   blue = advertising
 *   purple blink (200ms)   = connected, waiting for security
 *   green = secured / connected
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/bas.h>
#include <bluetooth/services/hids.h>

LOG_MODULE_REGISTER(peripheral, LOG_LEVEL_INF);

/* ── Identity indices ────────────────────────────────────────────────── */
#define PEER_ID  1   /* normal bonding identity */
#define TEMP_ID  2   /* temporary identity for identity reset */

/* ── HID payload types ───────────────────────────────────────────────── */
typedef struct {
	uint8_t mods;
	uint8_t reserved;
	uint8_t keys[6];
} payload_kb_t;

typedef struct {
	uint8_t buttons;
	int8_t  x;
	int8_t  y;
	int8_t  wheel;
	int8_t  pan;
} payload_mouse_t;

/* ── GPIO ────────────────────────────────────────────────────────────── */
static const struct gpio_dt_spec led_active = GPIO_DT_SPEC_GET(DT_ALIAS(led_active), gpios);
static const struct gpio_dt_spec btn        = GPIO_DT_SPEC_GET(DT_ALIAS(sw0),        gpios);

/* ── PWM ─────────────────────────────────────────────────────────────── */
static const struct pwm_dt_spec pwm_red  = PWM_DT_SPEC_GET(DT_ALIAS(pwm_red));
static const struct pwm_dt_spec pwm_blue = PWM_DT_SPEC_GET(DT_ALIAS(pwm_blue));

/* PWM period: 2 ms (500 Hz) — fast enough for flicker-free dimming */
#define PWM_PERIOD_US  2000U
#define PWM_PERIOD     PWM_USEC(PWM_PERIOD_US)

/* ── LED effect engine ───────────────────────────────────────────────── */
/*
 * A single k_timer drives all LED animations at 20 ms intervals.
 * Each effect keeps its own phase counter.
 *
 * Effects:
 *   EFFECT_OFF          — all LEDs off
 *   EFFECT_GREEN_ON     — solid green (GPIO)
 *   EFFECT_RED_BREATHE  — red PWM breathing (sine-like triangle wave)
 *   EFFECT_BLUE_BLINK   — blue PWM slow blink (1 Hz, sharp on/off)
 *   EFFECT_PURPLE_BLINK — red+blue GPIO fast blink 200 ms (security wait)
 */
typedef enum {
	EFFECT_OFF,
	EFFECT_GREEN_ON,
	EFFECT_RED_BREATHE,
	EFFECT_BLUE_BLINK,
	EFFECT_PURPLE_BLINK,
} led_effect_t;

static volatile led_effect_t current_effect = EFFECT_OFF;
static struct k_timer led_timer;
static uint32_t led_tick;   /* increments every 20 ms */

/* Triangle-wave brightness table for breathing effect */
static uint8_t breathe_lut(uint32_t tick)
{
	/* One full breath = 128 ticks × 20 ms = 2.56 s */
	uint32_t phase = tick & 127U;
	/* Triangle: 0→63 up, 64→127 down */
	uint8_t v = (phase < 64U) ? (uint8_t)(phase * 4U)
				   : (uint8_t)((127U - phase) * 4U);
	return v;  /* 0..252 */
}

static void led_timer_cb(struct k_timer *t)
{
	led_effect_t eff = current_effect;

	led_tick++;

	switch (eff) {
	case EFFECT_OFF:
		pwm_set_dt(&pwm_red,  PWM_PERIOD, 0);
		pwm_set_dt(&pwm_blue, PWM_PERIOD, 0);
		gpio_pin_set_dt(&led_active, 0);
		break;

	case EFFECT_GREEN_ON:
		pwm_set_dt(&pwm_red,  PWM_PERIOD, 0);
		pwm_set_dt(&pwm_blue, PWM_PERIOD, 0);
		gpio_pin_set_dt(&led_active, 1);
		break;

	case EFFECT_RED_BREATHE: {
		/* brightness 0..248, map to pulse width */
		uint32_t bright = breathe_lut(led_tick);
		uint32_t pulse  = (PWM_PERIOD * bright) / 255U;

		pwm_set_dt(&pwm_red,  PWM_PERIOD, pulse);
		pwm_set_dt(&pwm_blue, PWM_PERIOD, 0);
		gpio_pin_set_dt(&led_active, 0);
		break;
	}

	case EFFECT_BLUE_BLINK: {
		/* ~4 Hz fast blink: 6 ticks on, 6 ticks off (12-tick cycle at 20ms = 120ms) */
		uint32_t phase = led_tick % 12U;
		uint32_t pulse = (phase < 6U) ? PWM_PERIOD : 0U;

		pwm_set_dt(&pwm_red,  PWM_PERIOD, 0);
		pwm_set_dt(&pwm_blue, PWM_PERIOD, pulse);
		gpio_pin_set_dt(&led_active, 0);
		break;
	}

	case EFFECT_PURPLE_BLINK: {
		/* 200 ms blink: 10-tick cycle at 20ms */
		uint32_t phase  = led_tick % 10U;
		uint32_t on     = (phase < 5U) ? 1U : 0U;
		uint32_t pulse  = on ? PWM_PERIOD : 0U;

		pwm_set_dt(&pwm_red,  PWM_PERIOD, pulse);
		pwm_set_dt(&pwm_blue, PWM_PERIOD, pulse);
		gpio_pin_set_dt(&led_active, 0);
		break;
	}
	}
}

static void led_set_effect(led_effect_t eff)
{
	led_tick = 0;
	current_effect = eff;
}

/* ── Convenience wrappers used by BLE state machine ─────────────────── */
static void leds_off(void)           { led_set_effect(EFFECT_OFF);          }
static void led_set_advertising(void){ led_set_effect(EFFECT_BLUE_BLINK);   }
static void led_set_connected(void)  { led_set_effect(EFFECT_GREEN_ON);     }
static void blink_start(void)        { led_set_effect(EFFECT_PURPLE_BLINK); }
static void blink_stop(void)         { led_set_effect(EFFECT_OFF);          }
static void slow_blink_start(void)   { led_set_effect(EFFECT_RED_BREATHE);  }
static void slow_blink_stop(void)    { /* effect replaced by caller */       }

/* ── HID TX indicator — green flashes 30 ms on each report sent ──────── */
static struct k_timer hid_tx_timer;

static void hid_tx_timer_cb(struct k_timer *t)
{
	/* Only restore green-on if we're still in connected state */
	if (current_effect == EFFECT_GREEN_ON) {
		gpio_pin_set_dt(&led_active, 1);
	}
}

static void hid_tx_flash(void)
{
	if (current_effect != EFFECT_GREEN_ON) {
		return;
	}
	gpio_pin_set_dt(&led_active, 0);
	k_timer_start(&hid_tx_timer, K_MSEC(30), K_NO_WAIT);
}

/* ── BLE state ───────────────────────────────────────────────────────── */
static struct bt_conn *current_conn;
static bool pairing_mode;   /* set by SW1 long-press or stale-LTK, cleared on pairing_complete */

/* ── Advertising data ────────────────────────────────────────────────── */
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
		      BT_UUID_16_ENCODE(BT_UUID_HIDS_VAL),
		      BT_UUID_16_ENCODE(BT_UUID_BAS_VAL)),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE,
		CONFIG_BT_DEVICE_NAME,
		sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/* ── Filter Accept List ──────────────────────────────────────────────── */
static void accept_list_add(const struct bt_bond_info *info, void *user_data)
{
	int *cnt = user_data;
	int err  = bt_le_filter_accept_list_add(&info->addr);

	if (err) {
		LOG_WRN("accept_list_add failed: %d", err);
	} else {
		(*cnt)++;
	}
}

/* ── Advertising work ────────────────────────────────────────────────── */
static struct k_work adv_work;

static void adv_work_handler(struct k_work *work)
{
	struct bt_le_adv_param adv_param = {
		.id           = PEER_ID,
		.options      = BT_LE_ADV_OPT_CONN,
		.interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
		.interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
	};
	int err;

	/* Rebuild filter accept list from bonds on PEER_ID */
	bt_le_filter_accept_list_clear();

	if (!pairing_mode) {
		int bond_cnt = 0;

		bt_foreach_bond(PEER_ID, accept_list_add, &bond_cnt);

		if (bond_cnt == 0) {
			/* No bonds and not in pairing mode — stay idle */
			LOG_INF("No bonds — long-press SW1 to pair");
			slow_blink_start();
			return;
		}

		/* Bonded peers are in the Resolving List (IRK added during bonding),
		 * so the controller can resolve Windows RPA → identity address and
		 * match the FAL entry. Both FILTER_CONN and FILTER_SCAN_REQ are safe. */
		adv_param.options |= BT_LE_ADV_OPT_FILTER_SCAN_REQ;
		adv_param.options |= BT_LE_ADV_OPT_FILTER_CONN;
		LOG_INF("Advertising (bonded-only, %d peer(s))", bond_cnt);
	} else {
		LOG_INF("Advertising open (pairing mode)");
	}

	slow_blink_stop();
	err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err && err != -EALREADY) {
		LOG_ERR("bt_le_adv_start failed: %d", err);
		return;
	}
	led_set_advertising();
}

static void advertising_start(void)
{
	bt_le_adv_stop();
	k_work_submit(&adv_work);
}

/* ── Identity reset work — called on stale LTK ───────────────────────── */
static struct k_work identity_reset_work;

static void identity_reset_handler(struct k_work *work)
{
	/* Reset PEER_ID: generates a new BDA so Windows sees a brand-new device
	 * and initiates fresh pairing without requiring the user to delete the
	 * old bond on the PC. The old stale bond on Windows becomes orphaned. */
	int err = bt_id_reset(PEER_ID, NULL, NULL);

	if (err < 0) {
		LOG_ERR("bt_id_reset failed: %d", err);
	} else {
		LOG_INF("Identity reset — new address assigned to ID %d", PEER_ID);
	}

	pairing_mode = true;
	advertising_start();
}

/* ── SW1 long press: clear bonds, enter pairing mode ────────────────── */
static void do_unpair(void)
{
	LOG_INF("Long press — resetting identity, entering pairing mode");

	if (current_conn) {
		/* identity_reset_work will be submitted from disconnected() */
		pairing_mode = true;
		bt_conn_disconnect(current_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	} else {
		k_work_submit(&identity_reset_work);
	}
}

/* ═══════════════════════════════════════════════════════════════════════
 * HID Report Descriptor — keyboard (ID 1) + mouse (ID 2)
 * ═══════════════════════════════════════════════════════════════════════ */
static const uint8_t hid_report_desc[] = {
	/* Keyboard — Report ID 1 */
	0x05, 0x01, 0x09, 0x06, 0xA1, 0x01, 0x85, 0x01,
	0x05, 0x07, 0x19, 0xE0, 0x29, 0xE7,
	0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x08, 0x81, 0x02,
	0x95, 0x01, 0x75, 0x08, 0x81, 0x01,
	0x95, 0x06, 0x75, 0x08, 0x15, 0x00, 0x25, 0x65,
	0x05, 0x07, 0x19, 0x00, 0x29, 0x65, 0x81, 0x00,
	0x05, 0x08, 0x19, 0x01, 0x29, 0x05,
	0x95, 0x05, 0x75, 0x01, 0x91, 0x02,
	0x95, 0x01, 0x75, 0x03, 0x91, 0x01,
	0xC0,
	/* Mouse — Report ID 2 */
	0x05, 0x01, 0x09, 0x02, 0xA1, 0x01, 0x85, 0x02,
	0x09, 0x01, 0xA1, 0x00,
	0x05, 0x09, 0x19, 0x01, 0x29, 0x05,
	0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x05, 0x81, 0x02,
	0x75, 0x03, 0x95, 0x01, 0x81, 0x01,
	0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x38,
	0x15, 0x81, 0x25, 0x7F, 0x75, 0x08, 0x95, 0x03, 0x81, 0x06,
	0xC0, 0xC0,
};

#define REP_IDX_KB     0
#define REP_IDX_MOUSE  1

BT_HIDS_DEF(hids_obj, sizeof(payload_kb_t), sizeof(payload_mouse_t), 1);

static void hids_led_handler(struct bt_hids_rep *rep,
			     struct bt_conn *conn, bool is_write)
{
	if (is_write && rep->data && rep->size >= 1) {
		LOG_DBG("LED state: 0x%02x", rep->data[0]);
	}
}

static int hids_init(void)
{
	struct bt_hids_init_param p = {0};

	p.inp_rep_group_init.reports[REP_IDX_KB].size    = sizeof(payload_kb_t);
	p.inp_rep_group_init.reports[REP_IDX_KB].id      = 1;
	p.inp_rep_group_init.reports[REP_IDX_MOUSE].size = sizeof(payload_mouse_t);
	p.inp_rep_group_init.reports[REP_IDX_MOUSE].id   = 2;
	p.inp_rep_group_init.cnt = 2;

	p.outp_rep_group_init.reports[0].size    = 1;
	p.outp_rep_group_init.reports[0].id      = 1;
	p.outp_rep_group_init.reports[0].handler = hids_led_handler;
	p.outp_rep_group_init.cnt = 1;

	p.rep_map.data = hid_report_desc;
	p.rep_map.size = sizeof(hid_report_desc);

	p.info.bcd_hid        = 0x0111;
	p.info.b_country_code = 0x00;
	p.info.flags = BT_HIDS_REMOTE_WAKE | BT_HIDS_NORMALLY_CONNECTABLE;
	p.is_kb    = true;
	p.is_mouse = true;
	p.boot_kb_outp_rep_handler = hids_led_handler;

	return bt_hids_init(&hids_obj, &p);
}

/* ═══════════════════════════════════════════════════════════════════════
 * BLE connection callbacks
 * ═══════════════════════════════════════════════════════════════════════ */
static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_ERR("Connection failed: 0x%02x", err);
		advertising_start();
		return;
	}
	current_conn = bt_conn_ref(conn);
	bt_hids_connected(&hids_obj, conn);
	slow_blink_stop();
	blink_start();   /* purple = waiting for security */
	LOG_INF("Connected");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("Disconnected (0x%02x)", reason);
	bt_hids_disconnected(&hids_obj, conn);
	bt_conn_unref(current_conn);
	current_conn = NULL;
	blink_stop();

	/* If pairing_mode was set (stale LTK or SW1 long-press), reset identity
	 * so Windows sees a fresh device and re-pairs automatically. */
	if (pairing_mode) {
		k_work_submit(&identity_reset_work);
	} else {
		advertising_start();
	}
}

static void security_changed(struct bt_conn *conn, bt_security_t level,
			      enum bt_security_err err)
{
	if (err == BT_SECURITY_ERR_PIN_OR_KEY_MISSING) {
		/* Stale LTK: controller lost IRK/LTK (e.g. firmware reset wiped
		 * settings). Reset the identity so Windows sees a brand-new device
		 * address and initiates fresh pairing automatically — no manual
		 * bond deletion needed on the PC side.
		 * Do NOT use FORCE_PAIR — causes infinite loop on Windows. */
		LOG_WRN("Stale LTK — resetting identity for automatic re-pair");
		pairing_mode = true;
		blink_stop();
		led_set_advertising();
		bt_conn_disconnect(conn, BT_HCI_ERR_AUTH_FAIL);
		/* identity_reset_work submitted from disconnected() */
		return;
	}
	if (err) {
		LOG_WRN("Security failed (%d) — disconnecting", err);
		blink_stop();
		led_set_advertising();
		bt_conn_disconnect(conn, BT_HCI_ERR_AUTH_FAIL);
		return;
	}
	if (level >= BT_SECURITY_L2) {
		LOG_INF("Secured (level %d)", level);
		blink_stop();
		led_set_connected();   /* green */
	}
}

BT_CONN_CB_DEFINE(conn_cb) = {
	.connected        = connected,
	.disconnected     = disconnected,
	.security_changed = security_changed,
};

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	LOG_INF("Paired (bonded=%d)", bonded);
	pairing_mode = false;
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	LOG_WRN("Pairing failed (%d)", reason);
	blink_stop();
	led_set_advertising();
	bt_conn_disconnect(conn, BT_HCI_ERR_AUTH_FAIL);
}

static struct bt_conn_auth_info_cb auth_info_cb = {
	.pairing_complete = pairing_complete,
	.pairing_failed   = pairing_failed,
};

/* ═══════════════════════════════════════════════════════════════════════
 * main
 * ═══════════════════════════════════════════════════════════════════════ */
int main(void)
{
	int err;

	/* LEDs */
	gpio_pin_configure_dt(&led_active, GPIO_OUTPUT_INACTIVE);

	/* Button */
	gpio_pin_configure_dt(&btn, GPIO_INPUT);

	/* LED effect timer — fires every 20 ms */
	k_timer_init(&led_timer, led_timer_cb, NULL);
	k_timer_start(&led_timer, K_MSEC(20), K_MSEC(20));

	/* HID TX flash timer — single-shot, restores green after 30 ms */
	k_timer_init(&hid_tx_timer, hid_tx_timer_cb, NULL);

	/* Work items */
	k_work_init(&adv_work,            adv_work_handler);
	k_work_init(&identity_reset_work, identity_reset_handler);

	/* BLE init — red breathing during initialization */
	led_set_effect(EFFECT_RED_BREATHE);
	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("bt_enable: %d", err);
		return err;
	}

	settings_load();

	/* Ensure PEER_ID (ID 1) and TEMP_ID (ID 2) exist.
	 * bt_id_get() returns count via pointer; bt_id_create() returns the new id. */
	size_t id_count = 0;

	bt_id_get(NULL, &id_count);
	while ((int)id_count <= TEMP_ID) {
		err = bt_id_create(NULL, NULL);
		if (err < 0) {
			LOG_ERR("bt_id_create failed: %d", err);
			break;
		}
		LOG_INF("Created BT identity ID %d", err);
		id_count++;
	}

	/* Clean up any stale bonds on TEMP_ID from a previous interrupted reset */
	bt_unpair(TEMP_ID, BT_ADDR_LE_ANY);

	bt_conn_auth_cb_register(NULL);   /* Just Works */
	bt_conn_auth_info_cb_register(&auth_info_cb);

	err = hids_init();
	if (err) {
		LOG_ERR("hids_init: %d", err);
		return err;
	}

	advertising_start();

	/* ── Main loop ── */
	int64_t last_hid        = 0;
	int64_t btn_press_start = 0;
	bool    kb_pressed      = false;
	bool    btn_held        = false;

	while (1) {
		int64_t now = k_uptime_get();

		/* SW1 long-press detection (3s) */
		bool btn_down = (gpio_pin_get_dt(&btn) == 1);

		if (btn_down) {
			if (!btn_held) {
				btn_held        = true;
				btn_press_start = now;
			} else if ((now - btn_press_start) >= 3000) {
				btn_held = false;
				do_unpair();
			}
		} else {
			btn_held = false;
		}

		/* Fake HID every 500ms when connected */
		if (current_conn && (now - last_hid >= 500)) {
			last_hid = now;

			payload_kb_t kb = {0};

			if (!kb_pressed) {
				kb.keys[0] = 0x04;   /* 'a' */
			}
			kb_pressed = !kb_pressed;
			bt_hids_inp_rep_send(&hids_obj, current_conn,
					     REP_IDX_KB,
					     (uint8_t *)&kb, sizeof(kb), NULL);

			payload_mouse_t mouse = {0};

			mouse.x = 5;
			bt_hids_inp_rep_send(&hids_obj, current_conn,
					     REP_IDX_MOUSE,
					     (uint8_t *)&mouse, sizeof(mouse),
					     NULL);

			hid_tx_flash();
		}

		k_sleep(K_MSEC(10));
	}

	return 0;
}
