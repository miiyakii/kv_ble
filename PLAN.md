# KV BLE KM 切换器 — 完整实现计划

## Context

构建一个纯蓝牙 KM 切换器，由三个 nRF52840 Dongle 组成：
- **Dongle 1（Central）**：BLE central，连接蓝牙键鼠（HOGP client），将 HID 报告通过 USB CDC-ACM 发给 Linux
- **Dongle 2/3（Peripheral）**：BLE peripheral，接收来自 Linux 的 HID 帧，通过 BLE 发给对应 PC（Windows）
- **Linux km_relay**：在 /dev/ttyACM* 之间路由数据，实现热键切换目标 PC

Linux 只做数据中转和切换逻辑，不做 BLE，不做 USB HID gadget。

---

## 串口帧协议（km_proto）

帧格式：`[0xAA][TYPE:1B][LEN:1B][PAYLOAD:LEN][CRC8-CCITT:1B]`

| TYPE | 方向 | Payload | 说明 |
|------|------|---------|------|
| 0x01 | Central→Linux | payload_kb_t (8B) | 键盘报告 |
| 0x02 | Central→Linux | payload_mouse_t (5B) | 鼠标报告 |
| 0x10 | Linux→Central | 1B target (0/1) | 切换通知（仅供 Central LED 反映用） |
| 0x11 | Linux→Peripheral | payload_kb_t 或 mouse_t | 转发给 Peripheral |

实际上 Linux 直接把 0x01/0x02 帧原样转给对应 Peripheral，Peripheral 解析 TYPE 即可，无需额外 0x11 类型。

CRC：`crc8_ccitt(0xFF, payload, len)`，使用 `<zephyr/sys/crc.h>`，Python 端手动实现。

---

## 文件变更列表

| 文件 | 操作 |
|------|------|
| `src/peripheral/main.c` | **修改** — 增加 USB CDC init + UART IRQ + 帧解析 + frame_dispatch |
| `src/central/main.c` | **新建** — BLE central + HOGP client + USB CDC + 帧收发 |
| `config/prj_peripheral.conf` | **修改** — 增加 USB_DEVICE_STACK_NEXT + UART_INTERRUPT_DRIVEN |
| `config/prj_central.conf` | **新建** — Central 完整 Kconfig |
| `config/central.overlay` | **新建** — 复用 peripheral.overlay 结构（同硬件） |
| `CMakeLists.txt` | **修改** — 增加 central 分支 + common.cmake |
| `tools/km_relay/km_relay.py` | **新建** — Linux 热键路由程序 |
| `tools/km_relay/requirements.txt` | **新建** — pyserial |

---

## Step 1：修改 CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(kv_ble)

if(KM_APP STREQUAL "peripheral")
  target_sources(app PRIVATE src/peripheral/main.c)
  include(${ZEPHYR_BASE}/samples/subsys/usb/common/common.cmake)
endif()

if(KM_APP STREQUAL "central")
  target_sources(app PRIVATE src/central/main.c)
  include(${ZEPHYR_BASE}/samples/subsys/usb/common/common.cmake)
endif()
```

`common.cmake` 将 `sample_usbd_init.c` 加入编译，并使 `sample_usbd.h` 可 include。

---

## Step 2：修改 peripheral/main.c

在现有 ~560 行代码基础上新增以下内容（不改动已有逻辑）：

### 2.1 新增头文件
```c
#include <zephyr/usb/usbd.h>
#include <sample_usbd.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/crc.h>
```

### 2.2 新增全局变量
```c
static const struct device *km_uart = DEVICE_DT_GET(DT_ALIAS(km_uart));

RING_BUF_DECLARE(uart_rx_rb, 128);
static struct k_work uart_rx_work;
```

### 2.3 UART ISR + work handler（帧解析状态机）

在 ISR 中只做 `ring_buf_put` + `k_work_submit`，在 workqueue 中做解析，避免驱动调用在 ISR 上下文。

状态机：ST_MAGIC → ST_TYPE → ST_LEN → ST_PAYLOAD → ST_CRC

### 2.4 frame_dispatch（发送 HID 报告）
```c
static void frame_dispatch(uint8_t type, const uint8_t *p, uint8_t len)
{
    if (!current_conn) return;
    if (type == 0x01 && len == sizeof(payload_kb_t)) {
        bt_hids_inp_rep_send(&hids_obj, current_conn,
                             REP_IDX_KB, p, len, NULL);
        hid_tx_flash();
    } else if (type == 0x02 && len == sizeof(payload_mouse_t)) {
        bt_hids_inp_rep_send(&hids_obj, current_conn,
                             REP_IDX_MOUSE, p, len, NULL);
        hid_tx_flash();
    }
}
```

### 2.5 main() 中在 bt_enable() 之前初始化 USB + UART
```c
struct usbd_context *usbd = sample_usbd_init_device(NULL);
if (!usbd) { LOG_ERR("USB init failed"); return -1; }
// usbd_enable 在 sample_usbd_init_device 内已调用

k_work_init(&uart_rx_work, uart_rx_work_handler);
uart_irq_callback_user_data_set(km_uart, uart_isr, NULL);
uart_irq_rx_enable(km_uart);
```

---

## Step 3：修改 prj_peripheral.conf

在现有基础上追加：
```ini
# USB CDC-ACM (USBD new stack)
CONFIG_USB_DEVICE_STACK_NEXT=y
CONFIG_UART_LINE_CTRL=y
CONFIG_UART_INTERRUPT_DRIVEN=y
CONFIG_SAMPLE_USBD_MANUFACTURER="KM-Switch"
CONFIG_SAMPLE_USBD_PRODUCT="KM-Peripheral"
CONFIG_SAMPLE_USBD_PID=0x0003
CONFIG_HWINFO=y
```

注意：保持 `CONFIG_BOARD_SERIAL_BACKEND_CDC_ACM=n`，USB 由 `sample_usbd` 统一管理。

---

## Step 4：新建 src/central/main.c

### 整体结构
```
main()
 ├── GPIO LED 初始化（复用 peripheral 相同 overlay）
 ├── USB USBD 初始化（sample_usbd_init_device）
 ├── UART IRQ 初始化（接收 Linux 的 SWITCH_CMD 0x10）
 ├── bt_enable() + settings_load()
 ├── bt_hogp_init(&hogp, &hogp_init_params)
 ├── scan_init() + bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE)
 └── 返回（事件驱动）
```

### 关键 API 引用
- 扫描：`bt_scan_init()`, `bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, BT_UUID_HIDS)`, `bt_scan_cb_register()`
- 连接后安全：`bt_conn_set_security(conn, BT_SECURITY_L2)`
- GATT 发现：`bt_gatt_dm_start(conn, BT_UUID_HIDS, &dm_cb, NULL)`
- HOGP 分配：`bt_hogp_handles_assign(&hogp, dm)`
- 报告订阅：`bt_hogp_rep_subscribe(&hogp, rep, hogp_notify_cb)` （在 hogp_ready_cb 中遍历所有 INPUT report）
- 断连重扫：`bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE)`

### 通知回调 → 发串口帧
```c
static uint8_t hogp_notify_cb(struct bt_hogp *hogp,
                               struct bt_hogp_rep_info *rep,
                               uint8_t err, const uint8_t *data)
{
    uint8_t id   = bt_hogp_rep_id(rep);
    uint8_t size = bt_hogp_rep_size(rep);
    uint8_t type = (id == 1) ? 0x01 : (id == 2) ? 0x02 : 0;
    if (type && data && !err) uart_send_frame(type, data, size);
    return BT_GATT_ITER_CONTINUE;
}
```

### uart_send_frame（加帧头 + CRC）
```c
static void uart_send_frame(uint8_t type, const uint8_t *payload, uint8_t len)
{
    // 先检查 DTR：uart_line_ctrl_get(km_uart, UART_LINE_CTRL_DTR, &dtr)
    uint8_t crc = crc8_ccitt(0xFF, payload, len);
    uart_irq_tx_enable(km_uart);
    // 通过 TX FIFO 或 ring_buf 发送 [0xAA, type, len, ...payload, crc]
}
```

---

## Step 5：新建 config/prj_central.conf

```ini
CONFIG_NCS_SAMPLES_DEFAULTS=y
CONFIG_BOARD_SERIAL_BACKEND_CDC_ACM=n
CONFIG_GPIO=y
CONFIG_PWM=y
CONFIG_LOG=y
CONFIG_LOG_DEFAULT_LEVEL=3
CONFIG_LOG_BACKEND_RTT=y
CONFIG_USE_SEGGER_RTT=y
CONFIG_UART_CONSOLE=n

# BLE Central + HOGP
CONFIG_BT=y
CONFIG_BT_CENTRAL=y
CONFIG_BT_SMP=y
CONFIG_BT_GATT_CLIENT=y
CONFIG_BT_GATT_DM=y
CONFIG_BT_PRIVACY=y
CONFIG_BT_HOGP=y
CONFIG_BT_HOGP_REPORTS_MAX=4
CONFIG_BT_SCAN=y
CONFIG_BT_SCAN_FILTER_ENABLE=y
CONFIG_BT_SCAN_UUID_CNT=1
CONFIG_BT_DEVICE_NAME="KM-Switch-Central"
CONFIG_BT_MAX_CONN=1
CONFIG_BT_MAX_PAIRED=1
CONFIG_BT_SETTINGS=y
CONFIG_SETTINGS=y
CONFIG_FLASH=y
CONFIG_FLASH_PAGE_LAYOUT=y
CONFIG_FLASH_MAP=y
CONFIG_NVS=y

# USB CDC-ACM
CONFIG_USB_DEVICE_STACK_NEXT=y
CONFIG_UART_LINE_CTRL=y
CONFIG_UART_INTERRUPT_DRIVEN=y
CONFIG_SAMPLE_USBD_MANUFACTURER="KM-Switch"
CONFIG_SAMPLE_USBD_PRODUCT="KM-Central"
CONFIG_SAMPLE_USBD_PID=0x0002
CONFIG_HWINFO=y

CONFIG_MAIN_STACK_SIZE=4096
CONFIG_SYSTEM_WORKQUEUE_STACK_SIZE=2048
CONFIG_HEAP_MEM_POOL_SIZE=8192
```

---

## Step 6：新建 config/central.overlay

内容与 `peripheral.overlay` 相同（同为 nRF52840 Dongle 硬件，LED / 按键 / CDC ACM 节点一致）。

---

## Step 7：新建 tools/km_relay/km_relay.py

### 功能
- 读 `/dev/ttyACM0`（Central），解析 0x01/0x02 帧
- 热键检测：键盘报告中 `mods==0x30 && ScrollLock(0x47) in keys` → 切换目标
- 切换时发 0x10 帧给 Central（供其 LED 反映），并发空键盘报告清除按键状态
- 转发 HID 帧给 `/dev/ttyACM1` 或 `/dev/ttyACM2`（对应 Peripheral Dongle）
- 用 `select()` 实现非阻塞多路复用

### 帧解析器（Python 状态机，与 C 端对称）
与 C 端相同的 CRC8-CCITT（init=0xFF）。

### 配置（命令行参数）
```
--central /dev/ttyACM0
--pc1     /dev/ttyACM1
--pc2     /dev/ttyACM2
--hotkey  "rctrl+ralt+scrolllock"  (默认)
```

---

## 构建命令

```bash
# Peripheral (Dongle 2 & 3)
west build -b nrf52840dongle/nrf52840 -- \
  -DKM_APP=peripheral \
  -DCONF_FILE=config/prj_peripheral.conf \
  -DDTC_OVERLAY_FILE=config/peripheral.overlay

# Central (Dongle 1)
west build -b nrf52840dongle/nrf52840 -d build_central -- \
  -DKM_APP=central \
  -DCONF_FILE=config/prj_central.conf \
  -DDTC_OVERLAY_FILE=config/central.overlay
```

---

## 验证方案

1. **Peripheral 单测**：接 Linux，用 `minicom /dev/ttyACM1`，手动发帧 `AA 01 08 00 00 04 00 00 00 00 00 <crc>`（字母 A），观察 PC 收到按键
2. **Central 单测**：接蓝牙键盘，用 `cat /dev/ttyACM0 | xxd` 观察帧输出
3. **全链路测试**：运行 `python km_relay.py`，键盘输入流向 PC1；按热键（RCtrl+RAlt+ScrLk）后输入流向 PC2
4. **重连测试**：断电 Peripheral Dongle 重插，确认 Windows 自动重连（FAL + 绑定恢复）
5. **Stale LTK 测试**：peripheral 长按 SW1 reset 身份，Windows 应自动重新配对

---

## 关键技术注意事项

1. **USB 时序**：`sample_usbd_init_device()` 在 `bt_enable()` 之前调用；发送前检查 DTR 避免在 Linux 未打开串口时刷 log
2. **线程安全**：`hogp_notify_cb` 在 BT RX 线程，`uart_fifo_fill` 不能直接在其中调用——需通过 `k_work` 提交到 system workqueue，或用 `k_mutex` 保护
3. **HOGP Report ID 映射**：用 `bt_hogp_rep_id(rep)` 区分 keyboard(1) 和 mouse(2)；若目标键鼠 report ID 不是标准 1/2，需读 report map 建立映射
4. **串口设备号稳定**：用 udev 规则按 USB VID/PID/iSerial 绑定 `/dev/km-central`、`/dev/km-pc1`、`/dev/km-pc2`，避免 ttyACMx 编号漂移（`CONFIG_HWINFO=y` 使每个 Dongle 有唯一 iSerial）
5. **扫描目标过滤**：初期用 UUID 过滤；若环境有多个 HID 设备，后续增加设备名过滤或绑定后 FAL 限制
