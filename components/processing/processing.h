// DOSIMETER IMAGE PROCESSING

#ifndef __DOSIMETER_IMAGE_PROCESSING_H__
#define __DOSIMETER_IMAGE_PROCESSING_H__
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>
#include <stddef.h>
#include <esp_event.h>
#include <esp_log.h>
#include "sensor.h"
#include "esp_camera.h"

// DEFINITIONS
static const char *TAG = "ao-meter";

// define RGB channel indices
#define R (0)
#define G (1)
#define B (2)

static uint8_t lo = 16;				   // HDR image lower threshold
static uint8_t hi = 240;			   // HDR image upper threshold

// parameters
#define ON  (1)
#define OFF (0)
#define MHz (1000000)		   		   // MHz factor
#define CAM_FREQ (20*MHz) 		  	   // sensor chip frequency (10 MHz or 20 MHz), affects exposure time | 6 - 20
#define numpics (9)	      	  		   // number of pictures for HDR image
static size_t dtime[numpics] = {1, 2, 4, 8, 16, 32, 64, 128, 256};
#define SAMPLE_DOWN (ON)				// Enable camera chip down sampling
uint32_t RESO1;
uint32_t RESO2;
framesize_t IMAGESIZE;
framesize_t TARGETSIZE;

typedef enum{
	BILINEAR,
	DOWNSAMPLING,
}demosaicing_mode;

demosaicing_mode DEMOSAICING;

// target definition for final image
typedef enum
{
	UART,
	SD,
}target;

// GPIO PIN for LED FLASH, also used by SD CARD
#define LED_FLASH_1 (4)
#define LED_FLASH_2 (2)

// Image format
typedef enum
{
	PIXFORMAT_RGB = 0,
	PIXFORMAT_FLOAT,
	PIXFORMAT_HDR,
	PIXFORMAT_AOPIC,
	PIXFORMAT_nRGB,
	PIXFORMAT_nFLOAT,
	PIXFORMAT_MASK,
	PIXFORMAT_REGIONS,
} imageformat;

// define image buffer uint8_t
typedef struct {
	uint8_t * buf;
	size_t len;
	size_t width;
	size_t height;
	imageformat format;
	struct timeval timestamp;
} imagebuffer;



// FUNCTIONS

float uint8_2_float(uint8_t u8);

float uint16_2_float(uint16_t u16);

esp_err_t init_imbuffer(imagebuffer *buffer, uint32_t size);

camera_config_t camera_settings(imageformat format, framesize_t resolution);

esp_err_t init_camera(camera_config_t camera_config);

esp_err_t deinit_camera();

#endif /* __DOSIMETER_IMAGE_PROCESSING_H__ */


