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

## Assemble the αΩ-meter:

1. Solder the wires to the battery clip.
2. Glue the battery clip into the housing.
3. Solder the other ends of the wires carefully to 3V and ground of the ESP32-CAM board.
4. Place the camera sensor in the designated hole of the dosimeter housing.
5. Insert the camera sensor cable into the ESP32-CAM camera port.
6. Format the SD-card with FAT32 file system.
7. Create a "calibration" folder on the SD-Card, add the calibration and setting files and put the SD-card it into the SD-Card slot of the ESP32-CAM board.
8. Insert the batteries.
9. Close the housing.

## Flashing the firmware:

1. Setup a toolchain of your choice, see [espressif docs](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html).
2. Get the firmware via [git or download directly](https://github.com/Frudawski/alphaOmega-meter)
3. Connect the TTL-USB adapter to the ESP32-CAM as below
4. Flash the firmware

 ![Wire diagram](/material/flash-wireing.png?raw=true)

## αΩ-meter settings:

To change the operation mode and settings of the αΩ-meter create a binary "settings.dat" file with eleven uint32 values:

| Nr | Name | Values | Description |
|----|------|------|------------|
|  1 | MODE | 0 or 1  | 0 = USB-CAM mode: waiting for commands over UART <br>1 = dosimeter mode: measure in fixed time intervalls<br> |
|  2 | UART | 0 or 1 | 0 = don't send data back over UART <br>1 = send data back over UART |
|  3 | FREQ | uint32 | Measurement interval in seconds when in Dosimeter mode |
|  4 | BAUD | uint32 | ESP32 baudrate: values between 115200 to 1152000 seem to work |
|  5 | LOG  | 0 or 1 | 0 = disable log information over UART <br>1 = enable log information over UART (does not work with USB-CAM mode) | 
|  6 | SAVE | 0-5    | Specifies which images are saved during operation:<br>0 = save nothing<br>1 = save RGB image (without vignetting correction)<br>2 = save RGB image (with vignetting correction)<br>3 = save HDR image<br>4 = save α-opic image<br>5 = integral image mask values<br> |
|  7 | CONF | 0 or 1 | 0 = don't confirm succesfull image saving<br>1 = confirm succesfull image saving by sending 4 bytes back over UART|
|  8 | LED  | 0 or 1 | 0 = disable flash LED on ESP32-CAM board<br>1 = enable flash LED on ESP32-CAM boar. NOTE: SD-card cannot be used when LED flash is enabled. |
|  9 | RESO | 1-4 | Defines image resolution output size:<br>1 = 160 x 120 pixels: QQVGA<br>2 = 320 x 240 pixels: QVGA<br>3 = 640 x 480 pixels: VGA<br>4 = 1024 x 768 pixels XGA|
| 10 | DEMO | 1 or 2 | 1 = bilinear interpolation<br>2 = downsampling (for resolutions 1 and 2 only) |
| 11 | BLE | 0 or 1 | Bluetooth:<br>1 = enable (currently no functionality other than pairing)<br>0 = disabled|

## αΩ-meter commands:

**In USB-CAM mode the αΩ-meter can be controlled over UART with the following commands:**

| Command | Parameter | Description |
|---|------|---|
| IDN? | - | Returns the αΩ-meter firmware version and serial number if specified in the source code. |
| RGB | 1-256 | Request a 8bit raw RGB image without vigentting correction with an "integration time value" of 1-256. The actual integration time depends on several factors, e.g. CPU speed and camera clock speed. Values over a certain limit might not have any further influence. |
| nRGB | n 1-256 | Request an averaged 8bit raw RGB image composed out of n single exposure images without vigentting correction with an "integration time value" of 1-256. |
| FLOAT | 1-256 | Request a vignetting corrected float image with "integration time value" 1-256. |
| nFLOAT | n 1-256 | Request an averaged vignetting corrected float image composed out of  n single exposure images with "integration time value" 1-256. |
| HDR | - |Request a HDR image which is created out of 9 single images with different integration time |
| AOPIC | - | Request an α-opic image with six channels: sc, mc, lc, rh, mel, V(λ) |
| MASK | n | Request an α-opic image with a binary image mask applied, up to six individual image masks are supported. |
| REGIONS | - | Returns spatial-integral region values for all available binary image masks. |
| LIST MASKS | - | Lists the avaliable binary image masks on the SDcard. |
| LIST DATA | - |Lists all files in the data folder. |
| LIST CAL | - | Lists all files in the calibration folder. |


## Calibration:
The calibration process persists of several independend steps. For some calibration steps adequate measurement equipment is required. It also might be easier to calibrate the αΩ-meter before assembling. For each calibration step Matlab/Octave scripts are available. 

### Lens angle determination:


### Vignetting calibration

Vignetting describes the describes the phenomenon of reduced brightness especially at the lens'es peripherie as well as chromatic shifts. Both lens and sensor contribute to vignetting effects and should be calibrated together. 

For the relative vignetting correction an area light source with diffusor can be used:
1. Configure the αΩ-meter in USB-CAM mode and send data over UART mode with LOG mode off, than connect it to the TTL-USB adapter.
2. Position the αΩ-meter in close distance perpendicular to the light source, the luminaire should fill the complete image area. 
3. Determine a good integration time value by testing different values and check that no image area is over- or underexposed. 
4. Take several RGB images. Rotate the luninaire or dosimeter and change positions between images, that way inhomogeneous luminaire areas are averaged out.
5. The images are averaged and normalized with the image center, the inverse value gives the vignetting calibration.
6. Save the result as "vignetting.cal" and add it into the SD-dard folder calibration.

### RGB channel sensitivities

For the creation of the α-opic weighting function the camera sensor's RGB sensitivites must be known. To determine the RGB channel sensitivities adequate measurement equipment is neccessary., a tunable light source with narrow wavelength bands, e.g. a monochromator, and a spectroradiometer. If such measurement equipment is not availabe, use the provided data which was derived and averaged from several sensor measurements. 

1. Configure the αΩ-meter in USB-CAM mode and send data over UART mode with LOG mode off, then connect it to the TTL-USB adapter.
2. Set up the measurement equipment.
3. For the αΩ-meter measurements the vignetting corrected RGB images are used. 
4. Evaluate the image region of the αΩ-meter that corresponds to the spectroradiometer measurement field. Some devices have a target laser indicator.
5. Evaluate an integration time value for the αΩ-meter and make sure neither devices is overexposed during the measurements. The measurments are performed with a fixed integration time.
6. Measure with both devices the narrowband radiation in the visible region: 380  nm - 780 nm.
7. Derive the RGB sensitivities by averaging the correct αΩ-meter region and dividing the value with the unweighted integral of the spectroradiometer measurement.

### Relative calibration:

The RGB channels are linear combined to fit the α-opic and V(λ) target functions. 

1. Use the provided Matlab/Octave script to fit the α-opic and V(λ) weighting curves with the RGB channel sensitivities.

### Absolute calibration:





 

