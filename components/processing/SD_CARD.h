// SD CARD HEADER

// THE ORIGINAL SD-CARD EXAMPLE SOFTWARE FROM ESPRESSIF ESP-IDF V4.1.3 WAS CHANGED BY FREDERIC RUDAWSKI

#ifndef __DOSIMETER_SD_H__
#define __DOSIMETER_SD_H__
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <sys/param.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include <driver/gpio.h>
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include <dirent.h>

// DEFINITIONS

typedef enum{	// save image(s) on SD card
	SDNONE = 0, // save nothing
	SDRGB,	    // save RGB images   (no vignetting correction)
	SDFLOAT,    // save FLOAT images (with vignetting correction)
	SDHDR,	    // save HDR images
	SDAOPIC,    // save AOPIC images
	SDLOG,	    // save values in log - not yet implemented
	SDALL,      // save all images and values
	SDGRAY,	    // save high resolution grayscale image in dosimeter 2 mode
}save_sd_t;


// FUNCTIONS

esp_err_t init_sdcard(uint8_t mode);

esp_err_t deinit_sdcard();


#endif /* __DOSIMETER_SD_H__ */
