/*
 * src/peripheral/main.c — KVM Dongle firmware (keyboard-only)
 *
 * Advertising strategy:
 *   - ID 1 (peer identity), has bonds → undirected adv + FAL (FILTER_CONN + FILTER_SCAN_REQ)
 *     Windows RPA is resolved by the controller via the Resolving List (IRK added during
 *     bonding), so FAL + FILTER_CONN works correctly for bonded Windows peers.
 *   - ID 1, no bonds, pairing_mode=false → idle, red slow breathe, wait for SW1
 *   - ID 1, no bonds, pairing_mode=true  → open undirected adv (SW1 long-press)
 *
 * Stale LTK handling (Windows reconnect after firmware reset):
 *   When PIN_OR_KEY_MISSING fires, reset ID 1 via bt_id_reset() so Windows sees a
 *   brand-new device and initiates fresh pairing without manual bond deletion.
 *
 * Identity map (CONFIG_BT_ID_MAX=3):
 *   ID 0 (BT_ID_DEFAULT) — unused (reserved, nRF Desktop convention)
 *   ID 1 (PEER_ID)       — normal peer / bonding identity
 *   ID 2 (TEMP_ID)       — temporary identity used during bt_id_reset() swap
 *
 * LED states:
 *   red breathing        = initializing / no bond, waiting for SW1
 *   blue fast blink      = advertising
 *   purple blink (200ms) = connected, waiting for security
 *   green solid          = secured / connected
 *   green 30ms flash     = HID report sent
 *
 * Serial frame format: [0xAA][TYPE:1B][LEN:1B][PAYLOAD:LEN B][CRC8-CCITT:1B]
 *   TYPE 0x01 → keyboard report  (8 B: mods, reserved, 6 keycodes)
 *   TYPE 0x02 → mouse report     (5 B: buttons, x, y, wheel, pan)
 *   TYPE 0x03 → all-keys-release (8 B payload ignored, sends zero report)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/crc.h>
#include <zephyr/usb/usbd.h>
#include <sample_usbd.h>

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

/* 7-byte raw passthrough — Logitech POP Mouse format:
 *   [0] buttons (5 keys + 3-bit padding)
 *   [1] reserved
 *   [2][3] X  12-bit signed little-endian (packed across bytes)
 *   [3][4] Y  12-bit signed little-endian (packed across bytes)
 *   [5] wheel signed
 *   [6] pan   signed
 */
typedef struct {
	uint8_t raw[7];
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
 * A k_timer fires every 20 ms and submits led_work to the system workqueue.
 * All PWM/GPIO calls happen in workqueue context (not ISR), which is safe
 * for drivers that may sleep or use mutexes internally.
 *
 * Effects:
 *   EFFECT_OFF          — all LEDs off
 *   EFFECT_GREEN_ON     — solid green (GPIO)
 *   EFFECT_RED_BREATHE  — red PWM breathing (triangle wave, ~2.5 s period)
 *   EFFECT_BLUE_BLINK   — blue PWM fast blink (~4 Hz)
 *   EFFECT_PURPLE_BLINK — red+blue PWM fast blink 200 ms (security wait)
 */
typedef enum {
	EFFECT_OFF,
	EFFECT_GREEN_ON,
	EFFECT_RED_BREATHE,
	EFFECT_BLUE_BLINK,
	EFFECT_PURPLE_BLINK,
} led_effect_t;

static atomic_t current_effect = ATOMIC_INIT(EFFECT_OFF);
static struct k_timer led_timer;
static struct k_work  led_work;
static uint32_t led_tick;   /* incremented in workqueue, no concurrent writers */

/* Triangle-wave brightness for breathing effect */
static uint8_t breathe_lut(uint32_t tick)
{
	/* One full breath = 128 ticks × 20 ms = 2.56 s */
	uint32_t phase = tick & 127U;
	/* Triangle: 0→63 up, 64→127 down */
	return (phase < 64U) ? (uint8_t)(phase * 4U)
			     : (uint8_t)((127U - phase) * 4U);
}

static void led_work_handler(struct k_work *work)
{
	led_effect_t eff = (led_effect_t)atomic_get(&current_effect);

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
		uint32_t bright = breathe_lut(led_tick);
		uint32_t pulse  = (PWM_PERIOD * bright) / 255U;

		pwm_set_dt(&pwm_red,  PWM_PERIOD, pulse);
		pwm_set_dt(&pwm_blue, PWM_PERIOD, 0);
		gpio_pin_set_dt(&led_active, 0);
		break;
	}

	case EFFECT_BLUE_BLINK: {
		/* ~4 Hz: 6 ticks on, 6 ticks off (12-tick cycle at 20ms = 240ms) */
		uint32_t pulse = ((led_tick % 12U) < 6U) ? PWM_PERIOD : 0U;

		pwm_set_dt(&pwm_red,  PWM_PERIOD, 0);
		pwm_set_dt(&pwm_blue, PWM_PERIOD, pulse);
		gpio_pin_set_dt(&led_active, 0);
		break;
	}

	case EFFECT_PURPLE_BLINK: {
		/* 200 ms blink: 10-tick cycle at 20ms */
		uint32_t pulse = ((led_tick % 10U) < 5U) ? PWM_PERIOD : 0U;

		pwm_set_dt(&pwm_red,  PWM_PERIOD, pulse);
		pwm_set_dt(&pwm_blue, PWM_PERIOD, pulse);
		gpio_pin_set_dt(&led_active, 0);
		break;
	}
	}
}

/* Timer expiry: ISR context — only submits work, no driver calls */
static void led_timer_cb(struct k_timer *t)
{
	k_work_submit(&led_work);
}

static void led_set_effect(led_effect_t eff)
{
	led_tick = 0;
	atomic_set(&current_effect, (atomic_val_t)eff);
}

/* ── Convenience wrappers used by BLE state machine ─────────────────── */
static void led_set_advertising(void) { led_set_effect(EFFECT_BLUE_BLINK);   }
static void led_set_connected(void)   { led_set_effect(EFFECT_GREEN_ON);     }
static void led_set_securing(void)    { led_set_effect(EFFECT_PURPLE_BLINK); }
static void led_set_idle(void)        { led_set_effect(EFFECT_RED_BREATHE);  }

/* ── HID TX indicator — green flashes 30 ms on each report sent ──────── */
static struct k_work_delayable hid_tx_work;

static void hid_tx_work_handler(struct k_work *work)
{
	/* Restore green if still in connected state */
	if ((led_effect_t)atomic_get(&current_effect) == EFFECT_GREEN_ON) {
		gpio_pin_set_dt(&led_active, 1);
	}
}

/* Call this after sending a HID report to flash green LED briefly */
void hid_tx_flash(void)
{
	if ((led_effect_t)atomic_get(&current_effect) != EFFECT_GREEN_ON) {
		return;
	}
	gpio_pin_set_dt(&led_active, 0);
	k_work_reschedule(&hid_tx_work, K_MSEC(30));
}

/* ── Button interrupt ────────────────────────────────────────────────── */
static struct gpio_callback btn_cb_data;
static struct k_work_delayable btn_work;

static void btn_work_handler(struct k_work *work);  /* forward declaration */

static void btn_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	bool pressed = (gpio_pin_get_dt(&btn) == 1);

	if (pressed) {
		k_work_reschedule(&btn_work, K_MSEC(3000));
	} else {
		k_work_cancel_delayable(&btn_work);
	}
}

/* ── BLE state ───────────────────────────────────────────────────────── */
static struct bt_conn *current_conn;
static bool pairing_mode;

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

	bt_le_filter_accept_list_clear();

	if (!pairing_mode) {
		int bond_cnt = 0;

		bt_foreach_bond(PEER_ID, accept_list_add, &bond_cnt);

		if (bond_cnt == 0) {
			LOG_INF("No bonds — long-press SW1 to pair");
			led_set_idle();
			return;
		}

		/* Bonded peers in Resolving List → FAL works with Windows RPA */
		adv_param.options |= BT_LE_ADV_OPT_FILTER_SCAN_REQ;
		adv_param.options |= BT_LE_ADV_OPT_FILTER_CONN;
		LOG_INF("Advertising (bonded-only, %d peer(s))", bond_cnt);
	} else {
		LOG_INF("Advertising open (pairing mode)");
	}

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

/* ── Identity reset work — called on stale LTK or SW1 long-press ─────── */
static struct k_work identity_reset_work;

static void identity_reset_handler(struct k_work *work)
{
	int err = bt_id_reset(PEER_ID, NULL, NULL);

	if (err < 0) {
		LOG_ERR("bt_id_reset failed: %d", err);
	} else {
		LOG_INF("Identity reset — new address assigned to ID %d", PEER_ID);
	}

	pairing_mode = true;
	advertising_start();
}

/* ── SW1 long-press handler (runs in workqueue after 3 s) ────────────── */
static void btn_work_handler(struct k_work *work)
{
	/* Verify button is still held at 3 s mark */
	if (gpio_pin_get_dt(&btn) != 1) {
		return;
	}

	LOG_INF("Long press — resetting identity, entering pairing mode");

	if (current_conn) {
		/* identity_reset_work submitted from disconnected() */
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
	/* Modifier keys */
	0x05, 0x07, 0x19, 0xE0, 0x29, 0xE7,
	0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x08, 0x81, 0x02,
	/* Reserved byte */
	0x95, 0x01, 0x75, 0x08, 0x81, 0x01,
	/* Keycodes (6-key rollover) */
	0x95, 0x06, 0x75, 0x08, 0x15, 0x00, 0x25, 0x65,
	0x05, 0x07, 0x19, 0x00, 0x29, 0x65, 0x81, 0x00,
	/* LED output report (NumLock, CapsLock, ScrollLock, Compose, Kana) */
	0x05, 0x08, 0x19, 0x01, 0x29, 0x05,
	0x95, 0x05, 0x75, 0x01, 0x91, 0x02,
	0x95, 0x01, 0x75, 0x03, 0x91, 0x01,
	0xC0,
	/* Mouse — Report ID 2 (Logitech POP Mouse, 7-byte, 12-bit XY) */
	0x05, 0x01, 0x09, 0x02, 0xA1, 0x01, 0x85, 0x02,
	0x09, 0x01, 0xA1, 0x00,
	/* Buttons 1-5 */
	0x05, 0x09, 0x19, 0x01, 0x29, 0x05,
	0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x05, 0x81, 0x02,
	/* Padding 3 bits */
	0x75, 0x03, 0x95, 0x01, 0x81, 0x01,
	/* Reserved 8 bits */
	0x75, 0x08, 0x95, 0x01, 0x81, 0x01,
	/* X 12-bit signed (-2048..2047) */
	0x05, 0x01, 0x09, 0x30,
	0x16, 0x00, 0xF8, 0x26, 0xFF, 0x07,
	0x75, 0x0C, 0x95, 0x01, 0x81, 0x06,
	/* Y 12-bit signed (-2048..2047) */
	0x09, 0x31,
	0x16, 0x00, 0xF8, 0x26, 0xFF, 0x07,
	0x75, 0x0C, 0x95, 0x01, 0x81, 0x06,
	/* Wheel 8-bit signed */
	0x09, 0x38,
	0x15, 0x81, 0x25, 0x7F, 0x75, 0x08, 0x95, 0x01, 0x81, 0x06,
	/* Pan 8-bit signed */
	0x05, 0x0C, 0x0A, 0x38, 0x02,
	0x15, 0x81, 0x25, 0x7F, 0x75, 0x08, 0x95, 0x01, 0x81, 0x06,
	0xC0, 0xC0,
};

#define REP_IDX_KB     0
#define REP_IDX_MOUSE  1

BT_HIDS_DEF(hids_obj, sizeof(payload_kb_t), 7, 1);

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
 * USB CDC-ACM + km_proto frame receiver
 *
 * Frame: [0xAA][TYPE:1B][LEN:1B][PAYLOAD:LEN B][CRC8-CCITT:1B]
 *   TYPE 0x01 → keyboard report  (8 B)
 *   TYPE 0x02 → mouse report     (5 B: buttons, x, y, wheel, pan)
 *   TYPE 0x03 → all-keys-release (payload ignored, sends zero report)
 *
 * UART ISR puts raw bytes into a ring buffer and submits uart_rx_work.
 * The workqueue handler runs the parser state machine and calls
 * frame_dispatch() on valid frames — all driver calls stay out of ISR.
 * ═══════════════════════════════════════════════════════════════════════ */
#define KM_FRAME_MAGIC   0xAAU
#define KM_TYPE_KB       0x01U
#define KM_TYPE_MOUSE    0x02U
#define KM_TYPE_RELEASE  0x03U
#define KM_PAYLOAD_MAX   16U
#define KM_CRC_INIT      0xFFU

static const struct device *km_uart = DEVICE_DT_GET(DT_ALIAS(km_uart));
static struct usbd_context  *usbd_ctx;

RING_BUF_DECLARE(uart_rx_rb, 256);
static struct k_work uart_rx_work;

/* ── frame_dispatch: send HID report over BLE ───────────────────────── */
static void frame_dispatch(uint8_t type, const uint8_t *p, uint8_t len)
{
	static const uint8_t zero_report[sizeof(payload_kb_t)] = {0};

	if (!current_conn) {
		return;
	}

	if (type == KM_TYPE_RELEASE) {
		bt_hids_inp_rep_send(&hids_obj, current_conn,
				     REP_IDX_KB, zero_report,
				     sizeof(zero_report), NULL);
		return;
	}

	if (type == KM_TYPE_KB && len == sizeof(payload_kb_t)) {
		bt_hids_inp_rep_send(&hids_obj, current_conn,
				     REP_IDX_KB, p, len, NULL);
		hid_tx_flash();
	} else if (type == KM_TYPE_MOUSE && len == sizeof(payload_mouse_t)) {
		bt_hids_inp_rep_send(&hids_obj, current_conn,
				     REP_IDX_MOUSE, p, len, NULL);
		hid_tx_flash();
	} else {
		LOG_WRN("frame_dispatch: unknown type=0x%02x len=%u", type, len);
	}
}

/* ── Frame parser state machine ─────────────────────────────────────── */
typedef enum {
	ST_MAGIC,
	ST_TYPE,
	ST_LEN,
	ST_PAYLOAD,
	ST_CRC,
} frame_state_t;

static void uart_rx_work_handler(struct k_work *work)
{
	static frame_state_t state   = ST_MAGIC;
	static uint8_t       f_type;
	static uint8_t       f_len;
	static uint8_t       f_buf[KM_PAYLOAD_MAX];
	static uint8_t       f_idx;

	uint8_t byte;

	while (ring_buf_get(&uart_rx_rb, &byte, 1) == 1) {
		switch (state) {
		case ST_MAGIC:
			if (byte == KM_FRAME_MAGIC) {
				state = ST_TYPE;
			}
			break;

		case ST_TYPE:
			f_type = byte;
			state  = ST_LEN;
			break;

		case ST_LEN:
			if (byte == 0 || byte > KM_PAYLOAD_MAX) {
				LOG_WRN("Bad frame len: %u — resync", byte);
				state = ST_MAGIC;
				break;
			}
			f_len  = byte;
			f_idx  = 0;
			state  = ST_PAYLOAD;
			break;

		case ST_PAYLOAD:
			f_buf[f_idx++] = byte;
			if (f_idx == f_len) {
				state = ST_CRC;
			}
			break;

		case ST_CRC: {
			uint8_t crc = crc8_ccitt(KM_CRC_INIT, f_buf, f_len);

			if (crc == byte) {
				frame_dispatch(f_type, f_buf, f_len);
			} else {
				LOG_WRN("CRC mismatch: got 0x%02x expect 0x%02x",
					byte, crc);
			}
			state = ST_MAGIC;
			break;
		}
		}
	}
}

/* ── UART ISR: copy bytes to ring buffer, submit work ───────────────── */
static void uart_isr(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (!uart_irq_rx_ready(dev)) {
			break;
		}

		uint8_t buf[64];
		int     n = uart_fifo_read(dev, buf, sizeof(buf));

		if (n > 0) {
			uint32_t written = ring_buf_put(&uart_rx_rb, buf, n);

			if (written < (uint32_t)n) {
				LOG_WRN("uart_rx_rb full, dropped %d B", n - (int)written);
			}
			k_work_submit(&uart_rx_work);
		}
	}
}

/* ── USB USBD msg callback ───────────────────────────────────────────── */
static void usbd_msg_cb(struct usbd_context *const ctx, const struct usbd_msg *msg)
{
	LOG_DBG("USBD: %s", usbd_msg_type_string(msg->type));

	if (usbd_can_detect_vbus(ctx)) {
		if (msg->type == USBD_MSG_VBUS_READY) {
			if (usbd_enable(ctx)) {
				LOG_ERR("usbd_enable failed");
			}
		} else if (msg->type == USBD_MSG_VBUS_REMOVED) {
			usbd_disable(ctx);
		}
	}
}

/* ── USB + UART init (call before bt_enable) ─────────────────────────── */
static int usb_uart_init(void)
{
	int err;

	if (!device_is_ready(km_uart)) {
		LOG_ERR("CDC ACM UART not ready");
		return -ENODEV;
	}

	usbd_ctx = sample_usbd_init_device(usbd_msg_cb);
	if (!usbd_ctx) {
		LOG_ERR("sample_usbd_init_device failed");
		return -ENODEV;
	}

	if (!usbd_can_detect_vbus(usbd_ctx)) {
		err = usbd_enable(usbd_ctx);
		if (err) {
			LOG_ERR("usbd_enable failed: %d", err);
			return err;
		}
	}

	k_work_init(&uart_rx_work, uart_rx_work_handler);
	uart_irq_callback_user_data_set(km_uart, uart_isr, NULL);
	uart_irq_rx_enable(km_uart);

	LOG_INF("USB CDC-ACM ready");
	return 0;
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
	led_set_securing();   /* purple = waiting for security */
	LOG_INF("Connected");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("Disconnected (0x%02x)", reason);
	bt_hids_disconnected(&hids_obj, conn);
	bt_conn_unref(current_conn);
	current_conn = NULL;

	/* Cancel any pending HID TX flash */
	k_work_cancel_delayable(&hid_tx_work);

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
		/* Stale LTK: reset identity so Windows sees a new device address
		 * and re-pairs automatically — no manual bond deletion needed. */
		LOG_WRN("Stale LTK — resetting identity for automatic re-pair");
		pairing_mode = true;
		bt_conn_disconnect(conn, BT_HCI_ERR_AUTH_FAIL);
		/* identity_reset_work submitted from disconnected() */
		return;
	}
	if (err) {
		LOG_WRN("Security failed (%d) — disconnecting", err);
		bt_conn_disconnect(conn, BT_HCI_ERR_AUTH_FAIL);
		return;
	}
	if (level >= BT_SECURITY_L2) {
		LOG_INF("Secured (level %d)", level);
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

	/* Button — interrupt driven */
	gpio_pin_configure_dt(&btn, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&btn, GPIO_INT_EDGE_BOTH);
	gpio_init_callback(&btn_cb_data, btn_isr, BIT(btn.pin));
	gpio_add_callback(btn.port, &btn_cb_data);

	/* LED effect: timer submits work, driver calls happen in workqueue */
	k_work_init(&led_work, led_work_handler);
	k_timer_init(&led_timer, led_timer_cb, NULL);
	k_timer_start(&led_timer, K_MSEC(20), K_MSEC(20));

	/* HID TX flash — delayable work */
	k_work_init_delayable(&hid_tx_work, hid_tx_work_handler);

	/* Button long-press — delayable work */
	k_work_init_delayable(&btn_work, btn_work_handler);

	/* Other work items */
	k_work_init(&adv_work,            adv_work_handler);
	k_work_init(&identity_reset_work, identity_reset_handler);

	/* USB CDC-ACM + UART — init before BLE */
	err = usb_uart_init();
	if (err) {
		LOG_ERR("usb_uart_init: %d", err);
		return err;
	}

	/* BLE init — red breathing during initialization */
	led_set_idle();
	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("bt_enable: %d", err);
		return err;
	}

	settings_load();

	/* Ensure PEER_ID (ID 1) and TEMP_ID (ID 2) exist */
	size_t id_count = 0;

	bt_id_get(NULL, &id_count);
	for (size_t i = id_count; i <= TEMP_ID; i++) {
		err = bt_id_create(NULL, NULL);
		if (err < 0) {
			LOG_ERR("bt_id_create failed: %d", err);
			break;
		}
		LOG_INF("Created BT identity ID %d", err);
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

	/* Main thread has nothing to do — all logic is event-driven */
	return 0;
}
