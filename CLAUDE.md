# kv_ble Project Context

## NCS Environment

- **NCS version**: v3.2.4
- **NCS root**: `~/ncs/v3.2.4/`
- **nrf SDK**: `~/ncs/v3.2.4/nrf/`
- **Zephyr**: `~/ncs/v3.2.4/zephyr/`
- **nrfxlib**: `~/ncs/v3.2.4/nrfxlib/`

## Documentation Paths (local)

| Topic | Path |
|-------|------|
| nRF SDK docs source | `~/ncs/v3.2.4/nrf/doc/nrf/` |
| Zephyr docs source | `~/ncs/v3.2.4/zephyr/doc/` |
| Kconfig reference | `~/ncs/v3.2.4/nrf/doc/kconfig/` |
| nrfxlib docs | `~/ncs/v3.2.4/nrf/doc/nrfxlib/` |
| BT HID service | `~/ncs/v3.2.4/nrf/include/bluetooth/services/hids.h` |
| BT HID impl | `~/ncs/v3.2.4/nrf/subsys/bluetooth/services/hids.c` |
| nRF Desktop BLE bond | `~/ncs/v3.2.4/nrf/applications/nrf_desktop/src/modules/ble_bond.c` |

## Project Structure

```
kv_ble/
├── CMakeLists.txt        # Build target: KM_APP=peripheral
├── Kconfig               # Custom Kconfig symbols
├── prj.conf              # Base config
├── config/
│   └── peripheral.overlay  # nRF52840 Dongle DTS overlay (PWM LEDs, USB CDC)
├── src/
│   └── peripheral/
│       └── main.c        # BLE HID peripheral firmware (Dongle 2/3)
└── build_peripheral/     # CMake build dir (west build target)
```

## Build Commands

```bash
# Build peripheral firmware
west build -b nrf52840dongle/nrf52840 -- -DKM_APP=peripheral

# Incremental build (faster)
cd build_peripheral && cmake --build .
```

## Flashing (nRF52840 Dongle via USB DFU)

`west flash` 在此项目不可用（west manifest 解析有 Python 兼容性问题）。
使用以下手动流程，已验证可用。

### 前提

- nrfutil 8.x（含 nrf5sdk-tools 插件）位于：
  `~/ncs/toolchains/2ac5840438/nrfutil/home/bin/nrfutil`
- 插件目录：`~/ncs/toolchains/2ac5840438/nrfutil/home/lib/`

### 步骤

**1. 进入 DFU 模式**

按住 Dongle 侧面按钮后插入 USB，红色 LED 慢闪即为 DFU 模式。

确认设备：
```bash
lsusb | grep Nordic
# 应看到: Nordic Semiconductor ASA Open DFU Bootloader (VID 1915:521f)
```

确认串口：
```bash
ls /dev/ttyACM*
# DFU Bootloader 通常是 ttyACM1（ttyACM0 是 Central 设备）
```

**2. 生成 DFU zip 包**

```bash
NRFUTIL=~/ncs/toolchains/2ac5840438/nrfutil/home/bin/nrfutil

$NRFUTIL nrf5sdk-tools pkg generate \
  --hw-version 52 \
  --sd-req 0x00 \
  --application build_peripheral/zephyr/zephyr.hex \
  --application-version 1 \
  /tmp/peripheral_dfu.zip
```

`--sd-req 0x00` 表示不依赖 SoftDevice（Zephyr BLE 栈无需 SoftDevice）。

**3. 烧录**

```bash
NRFUTIL_HOME=~/ncs/toolchains/2ac5840438/nrfutil/home \
sudo -E ~/ncs/toolchains/2ac5840438/nrfutil/home/bin/nrfutil nrf5sdk-tools dfu usb-serial \
  -pkg /tmp/peripheral_dfu.zip \
  -p /dev/ttyACM1
```

成功输出：`Device programmed.`

烧录完成后 Dongle 自动重启运行新固件，从 `lsusb` 消失属正常（固件无 USB 描述符）。

### 注意事项

- 必须用 `sudo -E`（保留 `NRFUTIL_HOME` 环境变量），否则 sudo 找不到 nrf5sdk-tools 插件
- 串口号用错会报 `No trigger interface found`，换另一个 ttyACM 端口即可
- `LIBUSB_ERROR_ACCESS` 说明缺少 sudo 权限

## Current Firmware: BLE HID Peripheral (nRF52840 Dongle)

**Target board**: `nrf52840dongle/nrf52840`

### Identity Map (`CONFIG_BT_ID_MAX=3`)
| ID | Name | Purpose |
|----|------|---------|
| 0 | `BT_ID_DEFAULT` | Reserved (nRF Desktop convention) |
| 1 | `PEER_ID` | Normal peer / bonding identity |
| 2 | `TEMP_ID` | Temporary during `bt_id_reset()` swap |

### LED States (PWM + GPIO)
| State | LED | Pattern |
|-------|-----|---------|
| No bond, waiting SW1 | Red | Slow blink 500ms |
| Advertising | Blue | PWM breathing |
| Connected, waiting security | Purple | Blink 200ms |
| Secured/connected | Green | Solid |

### Advertising Strategy
- Has bonds → undirected adv + FAL (`FILTER_CONN` + `FILTER_SCAN_REQ`)
- No bonds + `pairing_mode=false` → idle, wait SW1
- No bonds + `pairing_mode=true` → open undirected adv (SW1 long-press)

### Stale LTK Handling
When `PIN_OR_KEY_MISSING` fires → call `bt_id_reset()` to change local BDA, forcing Windows to treat device as new (avoids user needing to delete old bond). Pattern from nRF Desktop `ble_bond.c`.

## Key APIs

```c
// BT identity management
bt_id_create()      // ~/ncs/v3.2.4/nrf/include/bluetooth/bluetooth.h
bt_id_reset()       // resets identity, changes BDA
bt_id_delete()

// HIDS service (NCS-specific)
bt_hids_init()      // ~/ncs/v3.2.4/nrf/include/bluetooth/services/hids.h
bt_hids_inp_rep_send()

// Settings (bond persistence)
settings_load()     // ~/ncs/v3.2.4/zephyr/include/zephyr/settings/settings.h
```

## Important Kconfig Options

```
CONFIG_BT_ID_MAX=3
CONFIG_BT_PRIVACY=y          # RPA support
CONFIG_BT_SMP=y              # Security Manager
CONFIG_BT_SETTINGS=y         # Bond persistence via settings
CONFIG_BT_HIDS=y             # NCS HID service
CONFIG_BT_HID_DEVICES_MAX=1
CONFIG_PWM=y
CONFIG_USB_DEVICE_STACK=y
```

## DTS Notes (nRF52840 Dongle)

Board node labels from `nrf52840dongle_nrf52840_common.dtsi`:
- `led0_green`: P0.06 ACTIVE_LOW (GPIO only)
- `led1_red`: P0.08 ACTIVE_LOW (PWM0 ch0)
- `led1_green`: P1.09 ACTIVE_LOW (PWM0 ch1)
- `led1_blue`: P0.12 ACTIVE_LOW (PWM0 ch2)

PWM polarity overridden to `POLARITY_INVERTED` in overlay (pulse=0 → off, pulse=period → full brightness).

## How to Look Up SDK Documentation

When you need API details, check these local sources in order:
1. Header files in `~/ncs/v3.2.4/nrf/include/` or `~/ncs/v3.2.4/zephyr/include/`
2. RST docs in `~/ncs/v3.2.4/nrf/doc/nrf/`
3. Sample code in `~/ncs/v3.2.4/nrf/samples/`
4. nRF Desktop app as reference: `~/ncs/v3.2.4/nrf/applications/nrf_desktop/`
