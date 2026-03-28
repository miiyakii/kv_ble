/*
 * src/central/main.c — Dongle 1 firmware (KM-Central)
 *
 * Role: BLE Central — connects to mouse (HOGP client), forwards
 *       mouse HID reports as km_proto frames over USB CDC-ACM to Linux km_relay.
 *       Also receives TYPE 0x10 (switch notification) from Linux for LED feedback.
 *
 * Flow:
 *   scan (UUID=HIDS) → connect → security L2 → GATT discover → hogp_assign
 *   → hogp_ready → subscribe all INPUT reports
 *   → hogp_notify_cb → uart_send_frame([0xAA][TYPE][LEN][PAYLOAD][CRC])
 *
 * LED states:
 *   red breathing   = scanning / no connection
 *   blue fast blink = connecting / discovering
 *   green solid     = secured & HOGP ready (forwarding active)
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
#include <bluetooth/gatt_dm.h>
#include <bluetooth/scan.h>
#include <bluetooth/services/hogp.h>

LOG_MODULE_REGISTER(central, LOG_LEVEL_INF);

/* ── km_proto frame constants ────────────────────────────────────────── */
#define KM_FRAME_MAGIC   0xAAU
#define KM_TYPE_MOUSE    0x02U
#define KM_TYPE_SWITCH   0x10U   /* Linux→Central: switch notification */
#define KM_PAYLOAD_MAX   16U
#define KM_CRC_INIT      0xFFU

/* ── GPIO ────────────────────────────────────────────────────────────── */
static const struct gpio_dt_spec led_active = GPIO_DT_SPEC_GET(DT_ALIAS(led_active), gpios);

/* ── PWM ─────────────────────────────────────────────────────────────── */
static const struct pwm_dt_spec pwm_red  = PWM_DT_SPEC_GET(DT_ALIAS(pwm_red));
static const struct pwm_dt_spec pwm_blue = PWM_DT_SPEC_GET(DT_ALIAS(pwm_blue));

#define PWM_PERIOD_US  2000U
#define PWM_PERIOD     PWM_USEC(PWM_PERIOD_US)

/* ── LED effect engine ───────────────────────────────────────────────── */
typedef enum {
	EFFECT_OFF,
	EFFECT_RED_BREATHE,    /* scanning */
	EFFECT_BLUE_BLINK,     /* connecting / discovering */
	EFFECT_GREEN_ON,       /* forwarding active */
} led_effect_t;

static atomic_t current_effect = ATOMIC_INIT(EFFECT_RED_BREATHE);
static struct k_work led_work;
static struct k_timer led_timer;

static void led_work_handler(struct k_work *work)
{
	static uint32_t tick;
	led_effect_t effect = (led_effect_t)atomic_get(&current_effect);

	tick++;

	/* All off first */
	gpio_pin_set_dt(&led_active, 0);
	pwm_set_dt(&pwm_red,  PWM_PERIOD, 0);
	pwm_set_dt(&pwm_blue, PWM_PERIOD, 0);

	switch (effect) {
	case EFFECT_RED_BREATHE: {
		/* Triangle wave: period ~2.5 s at 20 ms tick = 125 ticks */
		uint32_t phase = tick % 125U;
		uint32_t pulse = (phase < 63U) ? phase * (PWM_PERIOD / 62U)
					       : (124U - phase) * (PWM_PERIOD / 62U);
		pwm_set_dt(&pwm_red, PWM_PERIOD, pulse);
		break;
	}
	case EFFECT_BLUE_BLINK:
		/* 4 Hz blink: 12 ticks on, 12 off at 20 ms */
		if ((tick % 24U) < 12U) {
			pwm_set_dt(&pwm_blue, PWM_PERIOD, PWM_PERIOD);
		}
		break;
	case EFFECT_GREEN_ON:
		gpio_pin_set_dt(&led_active, 1);
		break;
	case EFFECT_OFF:
	default:
		break;
	}
}

static void led_timer_cb(struct k_timer *t)
{
	k_work_submit(&led_work);
}

static void led_set_scanning(void)
{
	atomic_set(&current_effect, EFFECT_RED_BREATHE);
}

static void led_set_connecting(void)
{
	atomic_set(&current_effect, EFFECT_BLUE_BLINK);
}

static void led_set_ready(void)
{
	atomic_set(&current_effect, EFFECT_GREEN_ON);
}

/* ── USB CDC-ACM (km_uart) ───────────────────────────────────────────── */
static const struct device *km_uart = DEVICE_DT_GET(DT_ALIAS(km_uart));
static struct usbd_context  *usbd_ctx;

/* TX: serialize access with a mutex + ring buffer */
static K_MUTEX_DEFINE(uart_tx_mutex);
RING_BUF_DECLARE(uart_tx_rb, 256);
static struct k_work uart_tx_work;

/* RX: ISR → ring buffer → workqueue (frame parser) */
RING_BUF_DECLARE(uart_rx_rb, 64);
static struct k_work uart_rx_work;

/* ── uart_send_frame: build km_proto frame and enqueue for TX ────────── */
static void uart_send_frame(uint8_t type, const uint8_t *payload, uint8_t len)
{
	uint32_t dtr = 0;

	/* Only send if Linux has the port open */
	uart_line_ctrl_get(km_uart, UART_LINE_CTRL_DTR, &dtr);
	if (!dtr) {
		return;
	}

	uint8_t crc = crc8_ccitt(KM_CRC_INIT, payload, len);
	uint8_t hdr[3] = { KM_FRAME_MAGIC, type, len };

	k_mutex_lock(&uart_tx_mutex, K_FOREVER);
	ring_buf_put(&uart_tx_rb, hdr, 3);
	ring_buf_put(&uart_tx_rb, payload, len);
	ring_buf_put(&uart_tx_rb, &crc, 1);
	k_mutex_unlock(&uart_tx_mutex);

	k_work_submit(&uart_tx_work);
}

/* TX work: drain ring buffer via UART FIFO */
static void uart_tx_work_handler(struct k_work *work)
{
	uint8_t buf[64];
	uint32_t n;

	k_mutex_lock(&uart_tx_mutex, K_FOREVER);
	n = ring_buf_get(&uart_tx_rb, buf, sizeof(buf));
	k_mutex_unlock(&uart_tx_mutex);

	if (n > 0) {
		uart_fifo_fill(km_uart, buf, (int)n);
	}
}

/* ── HOGP notify → uart_send_frame ──────────────────────────────────── */
static uint8_t hogp_notify_cb(struct bt_hogp *hogp_obj,
			      struct bt_hogp_rep_info *rep,
			      uint8_t err,
			      const uint8_t *data)
{
	if (!data || err) {
		return BT_GATT_ITER_STOP;
	}

	uint8_t id   = bt_hogp_rep_id(rep);
	uint8_t size = bt_hogp_rep_size(rep);

	if (id == 2) {
		uart_send_frame(KM_TYPE_MOUSE, data, size);
	} else {
		LOG_DBG("hogp: unhandled report id=%u size=%u", id, size);
	}

	return BT_GATT_ITER_CONTINUE;
}

/* ── HOGP client ─────────────────────────────────────────────────────── */
static struct bt_hogp hogp;
static struct k_work  hogp_ready_work;

static void hogp_on_ready(struct k_work *work)
{
	int err;
	struct bt_hogp_rep_info *rep = NULL;

	LOG_INF("HOGP ready — subscribing input reports");
	led_set_ready();

	while (NULL != (rep = bt_hogp_rep_next(&hogp, rep))) {
		if (bt_hogp_rep_type(rep) == BT_HIDS_REPORT_TYPE_INPUT) {
			LOG_INF("Subscribe report id=%u", bt_hogp_rep_id(rep));
			err = bt_hogp_rep_subscribe(&hogp, rep, hogp_notify_cb);
			if (err) {
				LOG_ERR("Subscribe failed (%d)", err);
			}
		}
	}
}

static void hogp_ready_cb(struct bt_hogp *hogp_obj)
{
	k_work_submit(&hogp_ready_work);
}

static void hogp_prep_fail_cb(struct bt_hogp *hogp_obj, int err)
{
	LOG_ERR("HOGP preparation failed (%d)", err);
}

static void hogp_pm_update_cb(struct bt_hogp *hogp_obj)
{
	LOG_INF("HOGP protocol mode: %s",
		bt_hogp_pm_get(hogp_obj) == BT_HIDS_PM_BOOT ? "BOOT" : "REPORT");
}

static const struct bt_hogp_init_params hogp_init_params = {
	.ready_cb      = hogp_ready_cb,
	.prep_error_cb = hogp_prep_fail_cb,
	.pm_update_cb  = hogp_pm_update_cb,
};

/* ── GATT discovery ──────────────────────────────────────────────────── */
static void discovery_completed_cb(struct bt_gatt_dm *dm, void *context)
{
	int err;

	LOG_INF("GATT discovery completed");

	err = bt_hogp_handles_assign(dm, &hogp);
	if (err) {
		LOG_ERR("bt_hogp_handles_assign failed (%d)", err);
	}

	err = bt_gatt_dm_data_release(dm);
	if (err) {
		LOG_ERR("bt_gatt_dm_data_release failed (%d)", err);
	}
}

static void discovery_not_found_cb(struct bt_conn *conn, void *context)
{
	LOG_WRN("HIDS service not found on peer");
}

static void discovery_error_cb(struct bt_conn *conn, int err, void *context)
{
	LOG_ERR("GATT discovery error (%d)", err);
}

static const struct bt_gatt_dm_cb discovery_cb = {
	.completed    = discovery_completed_cb,
	.service_not_found = discovery_not_found_cb,
	.error_found  = discovery_error_cb,
};

static void gatt_discover(struct bt_conn *conn)
{
	int err = bt_gatt_dm_start(conn, BT_UUID_HIDS, &discovery_cb, NULL);

	if (err) {
		LOG_ERR("bt_gatt_dm_start failed (%d)", err);
	}
}

/* ── BLE connection callbacks ────────────────────────────────────────── */
static struct bt_conn *default_conn;

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (conn_err) {
		LOG_ERR("Connection to %s failed (0x%02x)", addr, conn_err);
		if (conn == default_conn) {
			bt_conn_unref(default_conn);
			default_conn = NULL;
			bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
			led_set_scanning();
		}
		return;
	}

	LOG_INF("Connected: %s", addr);
	led_set_connecting();

	/* Elevate to encrypted link — discovery happens in security_changed */
	int err = bt_conn_set_security(conn, BT_SECURITY_L2);

	if (err) {
		LOG_WRN("Security request failed (%d), discovering anyway", err);
		gatt_discover(conn);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Disconnected: %s (reason 0x%02x)", addr, reason);

	if (bt_hogp_assign_check(&hogp)) {
		bt_hogp_release(&hogp);
	}

	if (default_conn == conn) {
		bt_conn_unref(default_conn);
		default_conn = NULL;
	}

	led_set_scanning();
	bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
}

static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		LOG_WRN("Security failed %s (level %u err %d)", addr, level, err);
	} else {
		LOG_INF("Secured %s level %u", addr, level);
	}

	/* Discover HIDS regardless — even on error attempt discovery */
	gatt_discover(conn);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected        = connected,
	.disconnected     = disconnected,
	.security_changed = security_changed,
};

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Paired %s bonded=%d", addr, bonded);
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_WRN("Pairing failed %s reason=%d", addr, reason);
}

static struct bt_conn_auth_info_cb auth_info_cb = {
	.pairing_complete = pairing_complete,
	.pairing_failed   = pairing_failed,
};

/* ── BLE scan callbacks ───────────────────────────────────────────────── */
static void scan_filter_match(struct bt_scan_device_info *device_info,
			      struct bt_scan_filter_match *filter_match,
			      bool connectable)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));
	LOG_INF("HIDS device found: %s connectable=%d", addr, connectable);
}

static void scan_connecting_error(struct bt_scan_device_info *device_info)
{
	LOG_WRN("Scan connecting error");
	led_set_scanning();
}

static void scan_connecting(struct bt_scan_device_info *device_info,
			    struct bt_conn *conn)
{
	default_conn = bt_conn_ref(conn);
}

/* Handle directed advertising (device reconnecting after bond) */
static void scan_filter_no_match(struct bt_scan_device_info *device_info,
				 bool connectable)
{
	int err;
	struct bt_conn *conn;
	char addr[BT_ADDR_LE_STR_LEN];

	if (device_info->recv_info->adv_type == BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
		bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));
		LOG_INF("Direct adv from bonded peer: %s", addr);
		bt_scan_stop();

		err = bt_conn_le_create(device_info->recv_info->addr,
					BT_CONN_LE_CREATE_CONN,
					device_info->conn_param, &conn);
		if (!err) {
			default_conn = bt_conn_ref(conn);
			bt_conn_unref(conn);
		}
	}
}

BT_SCAN_CB_INIT(scan_cb, scan_filter_match, scan_filter_no_match,
		scan_connecting_error, scan_connecting);

static void scan_init(void)
{
	int err;

	struct bt_scan_init_param scan_init_param = {
		.connect_if_match = 1,
		.scan_param       = NULL,
		.conn_param       = BT_LE_CONN_PARAM_DEFAULT,
	};

	bt_scan_init(&scan_init_param);
	bt_scan_cb_register(&scan_cb);

	err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, BT_UUID_HIDS);
	if (err) {
		LOG_ERR("bt_scan_filter_add failed (%d)", err);
		return;
	}

	err = bt_scan_filter_enable(BT_SCAN_UUID_FILTER, false);
	if (err) {
		LOG_ERR("bt_scan_filter_enable failed (%d)", err);
	}
}

/* ── RX: receive TYPE 0x10 (switch notification) from Linux ─────────── */
typedef enum {
	ST_MAGIC,
	ST_TYPE,
	ST_LEN,
	ST_PAYLOAD,
	ST_CRC,
} frame_state_t;

static void uart_rx_work_handler(struct k_work *work)
{
	static frame_state_t state = ST_MAGIC;
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
				LOG_WRN("rx: bad len=%u", byte);
				state = ST_MAGIC;
				break;
			}
			f_len = byte;
			f_idx = 0;
			state = ST_PAYLOAD;
			break;

		case ST_PAYLOAD:
			f_buf[f_idx++] = byte;
			if (f_idx >= f_len) {
				state = ST_CRC;
			}
			break;

		case ST_CRC: {
			uint8_t expected = crc8_ccitt(KM_CRC_INIT, f_buf, f_len);

			if (byte == expected) {
				if (f_type == KM_TYPE_SWITCH && f_len >= 1) {
					LOG_INF("Switch cmd: target=%u", f_buf[0]);
					/* Central just logs; km_relay handles routing */
					/* Future: LED change to indicate active PC */
				}
			} else {
				LOG_WRN("rx: CRC mismatch 0x%02x != 0x%02x",
					byte, expected);
			}
			state = ST_MAGIC;
			break;
		}

		default:
			state = ST_MAGIC;
			break;
		}
	}
}

/* ── UART ISR: buffer bytes, submit work ────────────────────────────── */
static void uart_isr(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {
			uint8_t buf[16];
			int n = uart_fifo_read(dev, buf, sizeof(buf));

			if (n > 0) {
				ring_buf_put(&uart_rx_rb, buf, (uint32_t)n);
				k_work_submit(&uart_rx_work);
			}
		}

		if (uart_irq_tx_ready(dev)) {
			uint8_t buf[64];
			uint32_t n;

			k_mutex_lock(&uart_tx_mutex, K_NO_WAIT);
			n = ring_buf_get(&uart_tx_rb, buf, sizeof(buf));
			k_mutex_unlock(&uart_tx_mutex);

			if (n > 0) {
				uart_fifo_fill(dev, buf, (int)n);
			} else {
				uart_irq_tx_disable(dev);
			}
		}
	}
}

/* TX work enables TX IRQ to drain the ring buffer via ISR */
static void uart_tx_work_handler_impl(struct k_work *work)
{
	uart_irq_tx_enable(km_uart);
}

/* ── USBD message callback ───────────────────────────────────────────── */
static void usbd_msg_cb(struct usbd_context *ctx, const struct usbd_msg *msg)
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

/* ── USB + UART init ─────────────────────────────────────────────────── */
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
	k_work_init(&uart_tx_work, uart_tx_work_handler_impl);
	uart_irq_callback_user_data_set(km_uart, uart_isr, NULL);
	uart_irq_rx_enable(km_uart);

	LOG_INF("USB CDC-ACM ready");
	return 0;
}

/* ═══════════════════════════════════════════════════════════════════════
 * main
 * ═══════════════════════════════════════════════════════════════════════ */
int main(void)
{
	int err;

	/* LED GPIO */
	gpio_pin_configure_dt(&led_active, GPIO_OUTPUT_INACTIVE);

	/* LED effect engine */
	k_work_init(&led_work, led_work_handler);
	k_timer_init(&led_timer, led_timer_cb, NULL);
	k_timer_start(&led_timer, K_MSEC(20), K_MSEC(20));

	/* HOGP ready work */
	k_work_init(&hogp_ready_work, hogp_on_ready);

	/* USB CDC-ACM (before bt_enable) */
	err = usb_uart_init();
	if (err) {
		LOG_ERR("usb_uart_init failed (%d)", err);
		return err;
	}

	/* BLE */
	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("bt_enable failed (%d)", err);
		return err;
	}
	LOG_INF("Bluetooth initialized");

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_conn_auth_info_cb_register(&auth_info_cb);
	if (err) {
		LOG_ERR("bt_conn_auth_info_cb_register failed (%d)", err);
		return err;
	}

	bt_hogp_init(&hogp, &hogp_init_params);

	scan_init();

	err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
	if (err) {
		LOG_ERR("bt_scan_start failed (%d)", err);
		return err;
	}

	LOG_INF("KM-Central scanning for HID devices...");
	return 0;
}
