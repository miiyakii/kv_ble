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

# Flash (after nrfutil install)
west flash

# Or build in build_peripheral dir
cd build_peripheral && cmake --build .
```

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
