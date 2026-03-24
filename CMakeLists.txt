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
