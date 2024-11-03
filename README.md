![αΩ-meter](/material/ao-meter.JPG?raw=true)

# αΩ-meter

The αΩ-meter is a camera based light dosimeter, that allows for α-opic and spatially resolved measurements of the personal light history. The firmware is written and tested with version 4.4.3 of the [ESP-IDF](https://github.com/espressif/esp-idf) environment and uses the [ESP-IDF FreeRTOS](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/freertos.html) (Real-Time Operating System).

## Dependencies:

The αΩ-meter firmware uses the following repositories, which are already included and adapted in the alphaOmega-meter repository:

1. [espressif/esp32-camera](https://github.com/espressif/esp32-camera) under Apache 2.0 license

## Components:

For the assembly and firmware flashing of the αΩ-meter the following components are required:

* Espressif ESP32 microcontroller: ESP32-CAM developer Board
* OmniVision OV2640 camera sensor with adequate lens (fisheye recommended)
* TTL-USB adapter
* Housing (3D printable modell included in the repository)
* 3 V or 5 V lithium battery (3D modell for 2x AAA Lithium-Ion battery)
* An appropriate battery clip
* Red and black wire
* Micro-SD card (FAT32 formatted)



 

