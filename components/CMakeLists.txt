set(COMPONENT_SRCS
  camera/camera.c
  camera/sccb.c
  camera/sensor.c
  camera/xclk.c
  sensors/ov2640.c
  sensors/ov3660.c
  sensors/ov5640.c
  sensors/ov7725.c
  sensors/ov7670.c
  sensors/nt99141.c
  conversions/yuv.c
  conversions/to_jpg.cpp
  conversions/to_bmp.c
  conversions/jpge.cpp
  conversions/esp_jpg_decode.c
  processing/processing.c
  processing/SD_CARD.c
  processing/demosaicing.c
  bluetooth/gatt_svr.c
  bluetooth/aometer_ble_server.c
  bluetooth/aometer_ble_client.c
  bluetooth/peer.c
  bluetooth/misc.c
  )

set(COMPONENT_ADD_INCLUDEDIRS
  camera/include
  conversions/include
  processing
  bluetooth
  )

set(COMPONENT_PRIV_INCLUDEDIRS
  camera/private_include
  sensors/private_include
  conversions/private_include
  )

set(COMPONENT_REQUIRES)
set(COMPONENT_PRIV_REQUIRES freertos nvs_flash spi_flash bt fatfs vfs sdmmc bt)
  
register_component()
