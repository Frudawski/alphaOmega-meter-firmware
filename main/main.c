/*
alpha-Omega-dosimeter
Author: Frederic Rudawski
mail: dosimeter@frudawski.de
Date: 05.07.2020 - last edited: 16.04.2024
 */

#include <aometer_ble_server.h>
#include <aometer_ble_client.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include <time.h>
#include <sys/time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "freertos/event_groups.h"
#include <driver/gpio.h>
#include <esp_vfs_fat.h>
#include <esp_sleep.h>
#include "esp_camera.h"
#include <driver/uart.h>
#include "esp_heap_caps.h"
#include <esp32/himem.h>
#include "sdkconfig.h"
#include "driver/adc.h"
#include "processing.h"
#include "demosaicing.h"
#include "SD_CARD.h"


// device information
uint32_t SERIAL_NUMBER = 1;					// To differentiate between devices
static float VERSION = 1.0;					// firmware version

// default settings - will be overwritten by settings file on SD card
uint32_t BAUD_RATE = 115200; 				// BAUD rate
uint32_t DOSIMETER_MODE = 0;				// 1 dosimeter mode, 0 = USB-CAM, 2 = USB-CAM high resolution gray scales
uint32_t MEAS_FREQ = 360;					// Measurement frequency in s
uint32_t UART_SEND = 0;						// send pictures over UART in USB-CAM mode
uint32_t SEND_SAVED = 0;					// send 4 bytes to confirm that image is saved on SD-Card
uint32_t BLE = 0;							// Bluetooth Low Enegergy
uint32_t MASKS = 6;							// number of masks
float VALUES[36];							// Mask values
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG	// ESP_LOG_NONE // ESP_LOG_DEBUG

// HDR series picture settings
static uint32_t image_delay = 3000;    // delay between images in ms (1000 without SD save, 1500 with RGB SD Save in HDR mode)
static uint16_t sensor_delay = 1000;   // delay after sensor setting change in ms
save_sd_t SD_SAVE = SDHDR;		       // default save mode

// SETTINGS VALUES
#define values (12)
// UART settings
#define EX_UART_NUM UART_NUM_0
#define PATTERN_CHR_NUM    (3)   /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/
#define BUF_SIZE	    (1024)
#define RX_BUF_SIZE (BUF_SIZE)
#define TX_BUF_SIZE (BUF_SIZE)

// queues and semaphore initialization
static QueueHandle_t uart0_queue;
static QueueHandle_t hdr_queue;
static QueueHandle_t camera_queue;
static QueueHandle_t image_data_queue;
static QueueHandle_t target_data_queue;
static QueueHandle_t format_data_queue;
static QueueHandle_t mask_data_queue;
static QueueHandle_t HDR_image_data_queue;
static QueueHandle_t HDR_data_queue;
static SemaphoreHandle_t xSDMutex;



// CAMERA EVENT
static void camera_event_task(void *pvParameters)
{
	// set up CAM sensor connection
	sensor_t *s = esp_camera_sensor_get();

	// sensor settings
	s->set_brightness(s, 0);     // -2 to 2
	s->set_contrast(s, 0);       // -2 to 2
	s->set_saturation(s, 0);     // -2 to 2
	s->set_special_effect(s, 0); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
	s->set_whitebal(s, 0);       // whitebalancing 0 = disable , 1 = enable
	s->set_sharpness(s, 0);      //
	s->set_denoise(s, 0);        //
	s->set_quality(s, 0);        // 0 to 60? low provides better JPEG quality
	s->set_awb_gain(s, 0);       // 0 = disable , 1 = enable
	s->set_wb_mode(s, 0);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
	s->set_exposure_ctrl(s, 0);  // 0 = disable , 1 = enable
	s->set_aec2(s, 0);           // 0 = disable , 1 = enable
	s->set_ae_level(s, 0);       // automatic exposure?: -2 to 2
	s->set_gain_ctrl(s, 0);      // 0 = disable , 1 = enable
	s->set_agc_gain(s, 0);       // 0 to 30
	s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
	s->set_bpc(s, 0);            // black pixel compensation? 0 = disable , 1 = enable
	s->set_wpc(s, 0);            // white pixel compensation? 0 = disable , 1 = enable
	s->set_raw_gma(s, 0);        // raw gamma auto?: 0 = disable , 1 = enable
	s->set_lenc(s, 1);           // lens correction: 0 = disable , 1 = enable
	s->set_hmirror(s, 0);        // horizontal mirror: 0 = disable , 1 = enable
	s->set_vflip(s, 0);          // vertical flip: 0 = disable , 1 = enable
	s->set_dcw(s, SAMPLE_DOWN);  // down size: 0 = disable , 1 = enable
	s->set_colorbar(s, 0);       // 0 = disable , 1 = enable

	// initialize variables
	camera_fb_t *pic = (camera_fb_t *)malloc(sizeof(camera_fb_t));
	//	uint32_t counter;
	uint32_t exptime;
	uint32_t send_mode;
	uint32_t pic_number;
	uint32_t mask_number;
	uint32_t max_pic_number;
	//	uint32_t folder = 0;
	BaseType_t camera_event_status;
	uint32_t event;
	static uint32_t send_0 = 0;
	static uint32_t send_1 = 1;
	static uint32_t send_format_RGB = 2;
	static target UART = 0;
	static target SD = 1;
	static imageformat RGB = PIXFORMAT_RGB;
	static imageformat FLOAT = PIXFORMAT_FLOAT;
	static imageformat HDR = PIXFORMAT_HDR;
	static imageformat AOPIC = PIXFORMAT_AOPIC;
	static imageformat nRGB = PIXFORMAT_nRGB;
	static imageformat nFLOAT = PIXFORMAT_nFLOAT;
	static imageformat MASK = PIXFORMAT_MASK;
	static imageformat REGIONS = PIXFORMAT_REGIONS;


	// TASK LOOP
	while(1){

		// check for event
		camera_event_status = xQueueReceive( camera_queue, &event, pdMS_TO_TICKS(100));

		if( camera_event_status == pdPASS)
		{

			ESP_LOGI(TAG, "Camera-event: image request received");

			// receive exposure time and image type
			xQueueReceive(camera_queue, &exptime,   portMAX_DELAY); // get exposure time
			ESP_LOGI(TAG, "Camera-event: exposure time received: %d", exptime);
			xQueueReceive(camera_queue, &send_mode, portMAX_DELAY); // get send mode (HDR = 1, RGB = 2, AOPIC = 3, FLOAT = 4)
			ESP_LOGI(TAG, "Camera-event: image format received: %d", send_mode);

			// Check format
			if(send_mode == HDR || send_mode == AOPIC || send_mode == nRGB || send_mode == nFLOAT || send_mode == REGIONS){
				xQueueReceive(camera_queue, &pic_number, portMAX_DELAY);
				xQueueReceive(camera_queue, &max_pic_number, portMAX_DELAY);
				ESP_LOGI(TAG, "Camera-event: image %zu of %zu", pic_number, max_pic_number);
			}
			if(send_mode == MASK){
				xQueueReceive(camera_queue, &mask_number, portMAX_DELAY);
				xQueueReceive(camera_queue, &pic_number, portMAX_DELAY);
				xQueueReceive(camera_queue, &max_pic_number, portMAX_DELAY);
				ESP_LOGI(TAG, "Camera-event: mask %zu ", mask_number);
				ESP_LOGI(TAG, "Camera-event: image %zu of %zu", pic_number, max_pic_number);
			}


			// set exposure time
			if( (send_mode == nRGB && pic_number>1) || (send_mode == nFLOAT && pic_number>1) ){
				// do nothing
			}else{
				ESP_LOGI(TAG, "Camera-event: set exposure time: %zu", exptime);
				s->set_aec_value(s, exptime);    // 0 to 1200
				// wait or sensor adjustment
				vTaskDelay(sensor_delay / portTICK_RATE_MS);
			}

			// get image
			ESP_LOGI(TAG, "Camera-event: get frame buffer...");
			pic = esp_camera_fb_get();

			// send image
			ESP_LOGI(TAG, "Camera-event: send-mode: %zu", send_mode);
			switch(send_mode){
			case PIXFORMAT_RGB: // RAW RGB (NO vignetting correction)
				// send image buffer to UART
				pic->format = PIXFORMAT_RGB;
				ESP_LOGI(TAG, "Camera-event: Send RGB to demosaicing.");
				xQueueSendToBack(image_data_queue, (imagebuffer*)&pic, pdMS_TO_TICKS(1000));
				xQueueSendToBack(target_data_queue, (target*)&UART, pdMS_TO_TICKS(1000));
				xQueueSendToBack(format_data_queue, (imageformat*)&RGB, pdMS_TO_TICKS(1000));
				break;
			case PIXFORMAT_FLOAT: // FLOAT + vignetting correction
				pic->format = PIXFORMAT_FLOAT;
				ESP_LOGI(TAG, "Camera-event: Send FLOAT to demosaicing.");
				xQueueSendToBack(image_data_queue, (imagebuffer*)&pic, pdMS_TO_TICKS(1000));
				xQueueSendToBack(target_data_queue, (target*)&UART, pdMS_TO_TICKS(1000));
				xQueueSendToBack(format_data_queue, (imageformat*)&FLOAT, pdMS_TO_TICKS(1000));
				break;
			case PIXFORMAT_HDR: // HDR + vignetting correction
				// set image format
				pic->format = PIXFORMAT_HDR;
				// send image buffer to HDR task
				ESP_LOGI(TAG, "Camera-event: Send to FLOAT -> HDR.");
				ESP_LOGI(TAG, "Camera-event: FLOAT -> HDR - pic number: %zu", pic_number);
				xQueueSendToBack(HDR_image_data_queue, (imagebuffer*)&pic, pdMS_TO_TICKS(1000));
				xQueueSendToBack(HDR_data_queue, (uint32_t*)&pic_number, pdMS_TO_TICKS(1000));
				xQueueSendToBack(HDR_data_queue, (uint32_t*)&max_pic_number, pdMS_TO_TICKS(1000));
				xQueueSendToBack(format_data_queue, (imageformat *)&HDR, pdMS_TO_TICKS(1000));
				vTaskDelay(sensor_delay / portTICK_RATE_MS);
				break;
			case PIXFORMAT_AOPIC: // AOPIC + vignetting correction
				// set image format
				pic->format = PIXFORMAT_AOPIC;
				// send to HDR task for further processing
				ESP_LOGI(TAG, "Camera-event: Send FLOAT -> HDR -> AOPIC.");
				ESP_LOGI(TAG, "Camera-event: FLOAT -> HDR -> AOPIC - pic number: %d.", pic_number);
				xQueueSendToBack(HDR_image_data_queue, (imagebuffer*)&pic, pdMS_TO_TICKS(1000));
				xQueueSendToBack(HDR_data_queue, (uint32_t*)&pic_number, pdMS_TO_TICKS(1000));
				xQueueSendToBack(HDR_data_queue, (uint32_t*)&max_pic_number, pdMS_TO_TICKS(1000));
				xQueueSendToBack(format_data_queue, (imageformat *)&AOPIC, pdMS_TO_TICKS(1000));
				vTaskDelay(sensor_delay / portTICK_RATE_MS);
				break;
			case PIXFORMAT_nRGB: // average RGB images without vignetting correction
				// set image format
				pic->format = PIXFORMAT_nRGB;
				// send image buffer to HDR task
				//ESP_LOGI(TAG, "Camera-event: Send nRGB -> HDR.");
				ESP_LOGI(TAG, "Camera-event: nRGB -> HDR - pic number: %zu", pic_number);
				xQueueSendToBack(HDR_image_data_queue, (imagebuffer*)&pic, pdMS_TO_TICKS(1000));
				xQueueSendToBack(HDR_data_queue, (uint32_t*)&pic_number, pdMS_TO_TICKS(1000));
				xQueueSendToBack(HDR_data_queue, (uint32_t*)&max_pic_number, pdMS_TO_TICKS(1000));
				xQueueSendToBack(format_data_queue, (imageformat *)&nRGB, pdMS_TO_TICKS(1000));
				vTaskDelay(sensor_delay / portTICK_RATE_MS);
				break;
			case PIXFORMAT_nFLOAT: // average FLOAT images with vignetting correction
				// set image format
				pic->format = PIXFORMAT_nFLOAT;
				ESP_LOGI(TAG, "Camera-event: nFLOAT -> HDR - pic number: %zu", pic_number);
				xQueueSendToBack(HDR_image_data_queue, (imagebuffer*)&pic, pdMS_TO_TICKS(1000));
				xQueueSendToBack(HDR_data_queue, (uint32_t*)&pic_number, pdMS_TO_TICKS(1000));
				xQueueSendToBack(HDR_data_queue, (uint32_t*)&max_pic_number, pdMS_TO_TICKS(1000));
				xQueueSendToBack(format_data_queue, (imageformat *)&nFLOAT, pdMS_TO_TICKS(1000));
				vTaskDelay(sensor_delay / portTICK_RATE_MS);
				break;
			case PIXFORMAT_MASK: // AOPIC + vignetting correction + MASK
				// set image format
				pic->format = PIXFORMAT_MASK;
				// send to HDR task for further processing
				ESP_LOGI(TAG, "Camera-event: Send FLOAT -> HDR -> AOPIC+MASK.");
				ESP_LOGI(TAG, "Camera-event: FLOAT -> HDR -> AOPIC+MASK - pic number: %d.", pic_number);
				xQueueSendToBack(HDR_image_data_queue, (imagebuffer*)&pic, pdMS_TO_TICKS(1000));
				xQueueSendToBack(HDR_data_queue, (uint32_t*)&mask_number, pdMS_TO_TICKS(1000));
				xQueueSendToBack(HDR_data_queue, (uint32_t*)&pic_number, pdMS_TO_TICKS(1000));
				xQueueSendToBack(HDR_data_queue, (uint32_t*)&max_pic_number, pdMS_TO_TICKS(1000));
				xQueueSendToBack(format_data_queue, (imageformat *)&MASK, pdMS_TO_TICKS(1000));
				vTaskDelay(sensor_delay / portTICK_RATE_MS);
				break;
			case PIXFORMAT_REGIONS: // AOPIC + vignetting correction + MASK
				// set image format
				pic->format = PIXFORMAT_MASK;
				// send to HDR task for further processing
				ESP_LOGI(TAG, "Camera-event: Send FLOAT -> HDR -> AOPIC-REGIONS.");
				ESP_LOGI(TAG, "Camera-event: FLOAT -> HDR -> AOPIC-REGIONS - pic number: %d.", pic_number);
				xQueueSendToBack(HDR_image_data_queue, (imagebuffer*)&pic, pdMS_TO_TICKS(1000));
				xQueueSendToBack(HDR_data_queue, (uint32_t*)&mask_number, pdMS_TO_TICKS(1000));
				xQueueSendToBack(HDR_data_queue, (uint32_t*)&pic_number, pdMS_TO_TICKS(1000));
				xQueueSendToBack(HDR_data_queue, (uint32_t*)&max_pic_number, pdMS_TO_TICKS(1000));
				xQueueSendToBack(format_data_queue, (imageformat *)&REGIONS, pdMS_TO_TICKS(1000));
				vTaskDelay(sensor_delay / portTICK_RATE_MS);
				break;
			}
		}
	}
}



// https://github.com/espressif/esp-idf/blob/master/examples/peripherals/uart/uart_events/main/uart_events_example_main.c
static void uart_event_task(void *pvParameters)
{

	uart_event_t event;
	size_t buffered_size;
	uint8_t* dtmp = (uint8_t*) malloc(RX_BUF_SIZE);
	static uint32_t send_mode_RGB = 0;
	static uint32_t send_mode_FLOAT = 1;
	static uint32_t send_mode_HDR = 2;
	static uint32_t send_mode_AOPIC = 3;
	static uint32_t send_mode_nRGB = 4;
	static uint32_t send_mode_nFLOAT = 5;
	static uint32_t send_mode_MASK = 6;
	static uint32_t send_mode_REGIONS = 7;

	static uint32_t send_request = 1;
	float temperature = 0;
	size_t logcounter = 1;
	//	uint32_t counter;
	uint8_t len = 32;
	char line[len];
	//	BaseType_t temperature_event_status;
	FILE *log = NULL;


	// infinitive loop
	while(1) {

		//Waiting for UART event.
		if(xQueueReceive(uart0_queue, (void *)&event, (portTickType)portMAX_DELAY)){
			bzero(dtmp, RX_BUF_SIZE);
			ESP_LOGI(TAG, "uart[%d] event:", EX_UART_NUM);
			switch(event.type){
			//Event of UART receiving data
			/*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
			case UART_DATA:
				uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);

				ESP_LOGI(TAG, "UART-event: UART input: %s", dtmp);

				// read UART command

				// IDN? request - device info and version
				if((strncmp((char*)dtmp,"IDN?",4)) == 0){
					ESP_LOGI(TAG, "UART-event: Command 'IDN?' received.");
					printf("aometer - version: %f, serial: %d", VERSION, SERIAL_NUMBER);
				}

				if(DOSIMETER_MODE!=2){

					// HDR request - single HDR image with vignetting correction
					if((strncmp((char*)dtmp,"HDR",3)) == 0){
						ESP_LOGI(TAG, "UART-event: Command 'HDR' received.");

						long maxpicnumber = numpics;

						// HDR picture loop
						for (uint32_t n=1; n<=numpics; n++){
							uint32_t ind = n-1;
							uint32_t t = dtime[ind];

							ESP_LOGI(TAG, "UART-event: request HDR image %d", n);
							xQueueSendToBack(camera_queue, &send_request, pdMS_TO_TICKS(1000));
							//xQueueSendToBack(camera_queue, &folder, pdMS_TO_TICKS(100));
							ESP_LOGI(TAG, "UART-event - send exposure time: %u", t);
							xQueueSendToBack(camera_queue, &t, portMAX_DELAY);		// exposure time
							ESP_LOGI(TAG, "UART-event - send image format %d", (uint32_t)PIXFORMAT_HDR);
							xQueueSendToBack(camera_queue, &send_mode_HDR, portMAX_DELAY);
							ESP_LOGI(TAG, "UART-event - send pic number %d", n);
							xQueueSendToBack(camera_queue, &n, portMAX_DELAY);
							ESP_LOGI(TAG, "UART-event: send number of images %ld", maxpicnumber);
							xQueueSendToBack(camera_queue, (uint32_t*)&maxpicnumber, pdMS_TO_TICKS(1000));
							vTaskDelay(image_delay/1/portTICK_RATE_MS);
							if(SD_SAVE == SDALL || SD_SAVE == SDRGB){
								//vTaskDelay(image_delay/2/portTICK_RATE_MS);
							}
						}
					}


					// RAW RGB request - single image with variable exposure time (0-1200)
					if((strncmp((char*)dtmp,"RGB",3)) == 0){
						ESP_LOGI(TAG, "UART-event: Command 'RGB' received.");
						uint8_t l = strlen((char *)dtmp);
						//ESP_LOGI(TAG, "Command length: %d", l);

						// get substring with exposure time
						char * ptr = NULL;
						char str[l-2];
						memset(str, NULL, (l-2)*sizeof(char));
						memcpy(str, &dtmp[3], l-3);

						// get exposure time from string
						long exptime = strtoll(str, &ptr, 10);
						ESP_LOGI(TAG, "UART-event: exposure time: %ld", exptime);

						// request image
						ESP_LOGI(TAG, "UART-event - send image request");
						xQueueSendToBack(camera_queue, &send_request, portMAX_DELAY); // request image
						ESP_LOGI(TAG, "UART-event - send exposure time: %ld", exptime);
						xQueueSendToBack(camera_queue, &exptime, portMAX_DELAY);		// exposure time
						ESP_LOGI(TAG, "UART-event - send image format %d", (uint32_t)PIXFORMAT_RGB);
						xQueueSendToBack(camera_queue, &send_mode_RGB, portMAX_DELAY);
					}


					// FLOAT request - single float image with variable exposure time (0-1200)
					if((strncmp((char*)dtmp,"FLOAT",5)) == 0){
						ESP_LOGI(TAG, "UART-event: Command 'FLOAT' received.");
						uint8_t l = strlen((char *)dtmp);

						// get substring with exposure time
						char* ptr = NULL;
						char str[l-2];
						memset(str, NULL, sizeof(str));
						memcpy(str, &dtmp[5], l-3);

						// get exposure time from string
						long exptime = strtoll(str, &ptr, 10);
						ESP_LOGI(TAG, "UART-event: exposure time: %ld", exptime);

						// request image
						ESP_LOGI(TAG, "UART-event - send image request");
						xQueueSendToBack(camera_queue, &send_request, portMAX_DELAY); // request image
						ESP_LOGI(TAG, "UART-event - send exposure time: %ld", exptime);
						xQueueSendToBack(camera_queue, &exptime, portMAX_DELAY);		// exposure time
						ESP_LOGI(TAG, "UART-event - send image format %d", (uint32_t)PIXFORMAT_FLOAT);
						xQueueSendToBack(camera_queue, (uint32_t*)&send_mode_FLOAT, portMAX_DELAY);  // send mode RGB = 4
					}


					// RAW nRGB request - averaged image with variable exposure time (0-1200)
					if((strncmp((char*)dtmp,"nRGB",4)) == 0){
						ESP_LOGI(TAG, "UART-event: Command 'nRGB' received.");
						uint8_t l = strlen((char *)dtmp);
						//ESP_LOGI(TAG, "Command length: %d", l);

						// get substring with exposure time
						char * ptr = NULL;
						char str[l-2];
						memset(str, NULL, sizeof(str));
						memcpy(str, &dtmp[4], l-3);

						// get number of images from string
						char * token = strtok(str, " ");
						long number = strtoll(token, &ptr, 10);
						// get exposure time from string
						token = strtok(NULL, " ");
						long exptime = strtoll(token, &ptr, 10);

						ESP_LOGI(TAG, "UART-event: number of images: %ld", number);
						ESP_LOGI(TAG, "UART-event: exposure time: %ld", exptime);


						// picture request loop
						for (long n=1; n<=number; n++){

							ESP_LOGI(TAG, "UART-event: request nRGB image %ld", n);
							xQueueSendToBack(camera_queue, &send_request, pdMS_TO_TICKS(1000));
							ESP_LOGI(TAG, "UART-event - send exposure time: %ld", exptime);
							xQueueSendToBack(camera_queue, &exptime, portMAX_DELAY);		// exposure time
							ESP_LOGI(TAG, "UART-event - send image format %d", (uint32_t)PIXFORMAT_nRGB);
							xQueueSendToBack(camera_queue, &send_mode_nRGB, portMAX_DELAY);
							ESP_LOGI(TAG, "UART-event - send pic number %ld", n);
							xQueueSendToBack(camera_queue, &n, portMAX_DELAY);
							ESP_LOGI(TAG, "UART-event: send number of images %ld", number);
							xQueueSendToBack(camera_queue, &number, pdMS_TO_TICKS(1000));
							vTaskDelay(image_delay/3/portTICK_RATE_MS);
						}
					}


					// RAW nFLOAT request - averaged image with variable exposure time (0-1200)
					if((strncmp((char*)dtmp,"nFLOAT",6)) == 0){
						ESP_LOGI(TAG, "UART-event: Command 'nFLOAT' received.");
						uint8_t l = strlen((char *)dtmp);
						//ESP_LOGI(TAG, "Command length: %d", l);

						// get substring with exposure time
						char * ptr = NULL;
						char str[l-2];
						memset(str, NULL, sizeof(str));
						memcpy(str, &dtmp[6], l-3);

						// get number of images from string
						char * token = strtok(str, " ");
						long number = strtoll(token, &ptr, 10);
						// get exposure time from string
						token = strtok(NULL, " ");
						long exptime = strtoll(token, &ptr, 10);

						ESP_LOGI(TAG, "UART-event: number of images: %ld", number);
						ESP_LOGI(TAG, "UART-event: exposure time: %ld", exptime);


						// picture request loop
						for (long n=1; n<=number; n++){

							ESP_LOGI(TAG, "UART-event: request nFLOAT image %ld", n);
							xQueueSendToBack(camera_queue, &send_request, pdMS_TO_TICKS(1000));
							ESP_LOGI(TAG, "UART-event - send exposure time: %ld", exptime);
							xQueueSendToBack(camera_queue, &exptime, portMAX_DELAY);		// exposure time
							ESP_LOGI(TAG, "UART-event - send image format %d", (uint32_t)PIXFORMAT_nRGB);
							xQueueSendToBack(camera_queue, &send_mode_nFLOAT, portMAX_DELAY);
							ESP_LOGI(TAG, "UART-event - send pic number %ld", n);
							xQueueSendToBack(camera_queue, &n, portMAX_DELAY);
							ESP_LOGI(TAG, "UART-event: send number of images %ld", number);
							xQueueSendToBack(camera_queue, &number, pdMS_TO_TICKS(1000));
							vTaskDelay(image_delay/3/portTICK_RATE_MS);
						}
					}


					// AOPIC request - 6 channel (sc,mc,lc,rh,mel,VL) image with vignetting correction
					if((strncmp((char *)dtmp,"AOPIC",5)) == 0){
						ESP_LOGI(TAG, "UART-event: Command 'AOPIC' received.");
						ESP_LOGI(TAG, "UART-event: send_mode: %zu", send_mode_AOPIC);

						long maxpicnumber = numpics;

						// HDR picture loop
						for (uint32_t n=1; n<=numpics; n++){
							uint32_t ind = n-1;
							uint32_t t = dtime[ind];
							ESP_LOGI(TAG, "HDR-event: request image %d", n);
							xQueueSendToBack(camera_queue, &send_request, pdMS_TO_TICKS(1000));
							ESP_LOGI(TAG, "UART-event - send exposure time: %u", t);
							xQueueSendToBack(camera_queue, &t, portMAX_DELAY);		// exposure time
							ESP_LOGI(TAG, "UART-event - send image format %d", (uint32_t)PIXFORMAT_AOPIC);
							xQueueSendToBack(camera_queue, &send_mode_AOPIC, portMAX_DELAY);
							ESP_LOGI(TAG, "UART-event - send pic number %d", n);
							xQueueSendToBack(camera_queue, &n, portMAX_DELAY);
							ESP_LOGI(TAG, "UART-event: send number of images to HDR task %ld", maxpicnumber);
							xQueueSendToBack(camera_queue, (uint32_t*)&maxpicnumber, pdMS_TO_TICKS(1000));
							vTaskDelay(image_delay/1/portTICK_RATE_MS);
							if(SD_SAVE == SDALL || SD_SAVE == SDRGB){
								//vTaskDelay(image_delay/2/portTICK_RATE_MS);
							}
						}
					}


					// LIST MASKS
					if((strncmp((char *)dtmp,"LIST MASKS",10)) == 0){
						ESP_LOGI(TAG, "UART-event: Command 'LIST MASKS' received.");

						// open mask files
						// https://github.com/espressif/esp-idf/blob/master/examples/protocols/http_server/file_serving/main/file_server.c#L81-L144
						char entrypath[128];
						char entrysize[16];
						const char *entrytype;
						struct dirent *entry;
						struct stat entry_stat;

						DIR *dir = opendir("/sdcard/calibration/masks");
						const size_t dirpath_len = strlen("/sdcard/calibration/masks");

						/* Retrieve the base path of file storage to construct the full path */
						strlcpy(entrypath, "/sdcard/calibration/masks", sizeof(entrypath));

						if (!dir) {
							ESP_LOGE(TAG, "Failed to stat dir : /sdcard/calibration/masks");
							//return ESP_FAIL;
						}else{
							/* Iterate over all files / folders and fetch their names and sizes */
							uint8_t n = 1;
							while ((entry = readdir(dir)) != NULL) {
								entrytype = (entry->d_type == DT_DIR ? "directory" : "file");
								if(DT_DIR == 2){ // file found
									printf("Mask %d: %s\n", n, entry->d_name);
									n++;
								}
							}
							closedir(dir);
						}
					}


					// LIST DATA
					if((strncmp((char *)dtmp,"LIST DATA",9)) == 0){
						ESP_LOGI(TAG, "UART-event: Command 'LIST DATA' received.");

						// open mask files
						// https://github.com/espressif/esp-idf/blob/master/examples/protocols/http_server/file_serving/main/file_server.c#L81-L144
						char entrypath[128];
						char entrysize[16];
						const char *entrytype;
						struct dirent *entry;
						struct stat entry_stat;

						DIR *dir = opendir("/sdcard/data");
						const size_t dirpath_len = strlen("/sdcard/data");

						/* Retrieve the base path of file storage to construct the full path */
						strlcpy(entrypath, "/sdcard/data", sizeof(entrypath));

						if (!dir) {
							ESP_LOGE(TAG, "Failed to stat dir : /sdcard/data");
							//return ESP_FAIL;
						}else{
							/* Iterate over all files / folders and fetch their names and sizes */
							uint8_t n = 1;
							while ((entry = readdir(dir)) != NULL) {
								entrytype = (entry->d_type == DT_DIR ? "directory" : "file");
								if(DT_DIR == 2){ // file found
									printf("%s\n", entry->d_name);
									n++;
								}
							}
							closedir(dir);
						}
					}

					// LIST CAL DATA
					if((strncmp((char *)dtmp,"LIST CAL",8)) == 0){
						ESP_LOGI(TAG, "UART-event: Command 'LIST CAL' received.");

						// open mask files
						// https://github.com/espressif/esp-idf/blob/master/examples/protocols/http_server/file_serving/main/file_server.c#L81-L144
						char entrypath[128];
						char entrysize[16];
						const char *entrytype;
						struct dirent *entry;
						struct stat entry_stat;

						DIR *dir = opendir("/sdcard/calibration");
						const size_t dirpath_len = strlen("/sdcard/calibration");

						/* Retrieve the base path of file storage to construct the full path */
						strlcpy(entrypath, "/sdcard/data", sizeof(entrypath));

						if (!dir) {
							ESP_LOGE(TAG, "Failed to stat dir : /sdcard/calibration");
							//return ESP_FAIL;
						}else{
							/* Iterate over all files / folders and fetch their names and sizes */
							uint8_t n = 1;
							while ((entry = readdir(dir)) != NULL) {
								entrytype = (entry->d_type == DT_DIR ? "directory" : "file");
								//ESP_LOGI(TAG, "type: %s", entrytype);
								//ESP_LOGI(TAG, "filename: %s", entry->d_name);
								if(DT_DIR == 2){ // file found
									printf("%s\n", entry->d_name);
									n++;
								}
							}
							closedir(dir);
						}
					}



					// AOPIC MASK request - 6 channel (sc,mc,lc,rh,mel,VL) image with vignetting correction and mask
					if((strncmp((char *)dtmp,"MASK",4)) == 0){
						ESP_LOGI(TAG, "UART-event: Command 'MASK' received.");

						// get substring with exposure time
						uint8_t l = strlen((char *)dtmp);
						char* ptr = NULL;
						char str[l-2];
						memset(str, NULL, sizeof(str));
						memcpy(str, &dtmp[4], l-3);

						// get exposure time from string
						uint32_t mask = strtoll(str, &ptr, 10);
						ESP_LOGI(TAG, "UART-event: mask: %d", mask);

						long maxpicnumber = numpics;
						ESP_LOGI(TAG, "UART-event: send_mode: %zu", send_mode_MASK);

						// HDR picture loop
						for (uint32_t n=1; n<=numpics; n++){
							uint32_t ind = n-1;
							uint32_t t = dtime[ind];
							ESP_LOGI(TAG, "HDR-event: request image %d", n);
							xQueueSendToBack(camera_queue, &send_request, pdMS_TO_TICKS(1000));
							//xQueueSendToBack(camera_queue, &folder, pdMS_TO_TICKS(100));
							ESP_LOGI(TAG, "UART-event - send exposure time: %u", t);
							xQueueSendToBack(camera_queue, &t, portMAX_DELAY);		// exposure time
							ESP_LOGI(TAG, "UART-event - send image format %d", (uint32_t)PIXFORMAT_MASK);
							xQueueSendToBack(camera_queue, &send_mode_MASK, portMAX_DELAY);
							ESP_LOGI(TAG, "UART-event - send MASK number: %d", (uint32_t)PIXFORMAT_MASK);
							xQueueSendToBack(camera_queue, &mask, portMAX_DELAY);
							ESP_LOGI(TAG, "UART-event - send pic number %d", n);
							xQueueSendToBack(camera_queue, &n, portMAX_DELAY);
							ESP_LOGI(TAG, "UART-event: send number of images to HDR task %ld", maxpicnumber);
							xQueueSendToBack(camera_queue, (uint32_t*)&maxpicnumber, pdMS_TO_TICKS(1000));
							vTaskDelay(image_delay/1/portTICK_RATE_MS);
							if(SD_SAVE == SDALL || SD_SAVE == SDRGB){
								//vTaskDelay(image_delay/2/portTICK_RATE_MS);
							}
						}
					}


					// AOPIC MASK REGION VALUE
					if((strncmp((char *)dtmp,"REGIONS",7)) == 0){
						ESP_LOGI(TAG, "UART-event: Command 'REGIONS' received.");
						ESP_LOGI(TAG, "UART-event: send_mode: %zu", send_mode_REGIONS);

						long maxpicnumber = numpics;

						// HDR picture loop
						for (uint32_t n=1; n<=numpics; n++){
							uint32_t ind = n-1;
							uint32_t t = dtime[ind];
							ESP_LOGI(TAG, "HDR-event: request image %d", n);
							xQueueSendToBack(camera_queue, &send_request, pdMS_TO_TICKS(1000));
							//xQueueSendToBack(camera_queue, &folder, pdMS_TO_TICKS(100));
							ESP_LOGI(TAG, "UART-event - send exposure time: %u", t);
							xQueueSendToBack(camera_queue, &t, portMAX_DELAY);		// exposure time
							ESP_LOGI(TAG, "UART-event - send image format %d", (uint32_t)PIXFORMAT_REGIONS);
							xQueueSendToBack(camera_queue, &send_mode_REGIONS, portMAX_DELAY);
							ESP_LOGI(TAG, "UART-event - send pic number %d", n);
							xQueueSendToBack(camera_queue, &n, portMAX_DELAY);
							ESP_LOGI(TAG, "UART-event: send number of images to HDR task %ld", maxpicnumber);
							xQueueSendToBack(camera_queue, (uint32_t*)&maxpicnumber, pdMS_TO_TICKS(1000));
							vTaskDelay(image_delay/1/portTICK_RATE_MS);
							if(SD_SAVE == SDALL || SD_SAVE == SDRGB){
								//vTaskDelay(image_delay/2/portTICK_RATE_MS);
							}
						}
					}
				}


				//uart_write_bytes(EX_UART_NUM, (const char*) dtmp, event.size);
				break;
				//Event of HW FIFO overflow detected
			case UART_FIFO_OVF:
				ESP_LOGW(TAG, "hw fifo overflow");
				// If fifo overflow happened, you should consider adding flow control for your application.
				// The ISR has already reset the rx FIFO,
				// As an example, we directly flush the rx buffer here in order to read more data.
				uart_flush_input(EX_UART_NUM);
				xQueueReset(uart0_queue);
				break;
				//Event of UART ring buffer full
			case UART_BUFFER_FULL:
				ESP_LOGW(TAG, "ring buffer full");
				// If buffer full happened, you should consider encreasing your buffer size
				// As an example, we directly flush the rx buffer here in order to read more data.
				uart_flush_input(EX_UART_NUM);
				xQueueReset(uart0_queue);
				break;
				//Event of UART RX break detected
			case UART_BREAK:
				ESP_LOGW(TAG, "uart rx break");
				break;
				//Event of UART parity check error
			case UART_PARITY_ERR:
				ESP_LOGW(TAG, "uart parity error");
				break;
				//Event of UART frame error
			case UART_FRAME_ERR:
				ESP_LOGW(TAG, "uart frame error");
				break;
				//UART_PATTERN_DET
			case UART_PATTERN_DET:
				uart_get_buffered_data_len(EX_UART_NUM, &buffered_size);
				int pos = uart_pattern_pop_pos(EX_UART_NUM);
				ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
				if (pos == -1) {
					// There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
					// record the position. We should set a larger queue size.
					// As an example, we directly flush the rx buffer here.
					uart_flush_input(EX_UART_NUM);
				} else {
					uart_read_bytes(EX_UART_NUM, &dtmp, pos, 100 / portTICK_PERIOD_MS);
					uint8_t pat[PATTERN_CHR_NUM + 1];
					memset(pat, 0, sizeof(pat));
					uart_read_bytes(EX_UART_NUM, pat, PATTERN_CHR_NUM, 100 / portTICK_PERIOD_MS);
					ESP_LOGI(TAG, "read data: %s", dtmp);
					ESP_LOGI(TAG, "read pat : %s", pat);
				}
				break;
				//Others
			default:
				ESP_LOGI(TAG, "UART event type: %d", event.type);
				break;
			}
		}
	}
	free(dtmp);
	dtmp = NULL;
	vTaskDelete(NULL);
}





// send image data
void processing_task()
{
	// task variables
	BaseType_t send_image_event_status;
	imagebuffer *pic;
	imagebuffer *expt;
	size_t len;
	static target target;
	static imageformat format;
	static uint32_t mask;
	static float maskfactor;
	FILE *vigfile = NULL;
	uint8_t filenum = 1;
	FILE *mask1 = NULL;
	FILE *mask2 = NULL;
	FILE *mask3 = NULL;
	FILE *mask4 = NULL;
	FILE *mask5 = NULL;
	FILE *mask6 = NULL;
	FILE *omega_file = NULL;

	FILE *relfile = NULL;
	FILE *absfile = NULL;
	static FILE *imagefile = NULL;
	bool vignetting = 1;
	static float relcal[18] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
	static float abscal[6] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0,};

	// read relative calibration data
	relfile = fopen("/sdcard/calibration/rel_calibration.cal", "r");
	if (relfile != NULL)
	{
		// read relative calibration data
		ESP_LOGI(TAG, "send-image-event - read relative calibration file.");
		size_t err = fread(relcal, sizeof(float), 18, relfile);
	}else{
		ESP_LOGE(TAG, "Send-image-event: Failed to open relative calibration file.");
	}
	fclose(relfile);
	ESP_LOGI(TAG, "rel SC:  %+1.3f %+1.3f %+1.3f", relcal[0],  relcal[1],  relcal[2]);
	ESP_LOGI(TAG, "rel MC:  %+1.3f %+1.3f %+1.3f", relcal[3],  relcal[4],  relcal[5]);
	ESP_LOGI(TAG, "rel LC:  %+1.3f %+1.3f %+1.3f", relcal[6],  relcal[7],  relcal[8]);
	ESP_LOGI(TAG, "rel RH:  %+1.3f %+1.3f %+1.3f", relcal[9],  relcal[10], relcal[11]);
	ESP_LOGI(TAG, "rel MEL: %+1.3f %+1.3f %+1.3f", relcal[12], relcal[13], relcal[14]);
	ESP_LOGI(TAG, "rel VL:  %+1.3f %+1.3f %+1.3f", relcal[15], relcal[16], relcal[17]);

	// read absolute calibration data
	absfile = fopen("/sdcard/calibration/abs_calibration.cal", "r");
	if (absfile != NULL)
	{
		// read relative calibration data
		ESP_LOGI(TAG, "send-image-event - read absolute calibration file.");
		size_t err = fread(abscal, sizeof(float), 6, absfile);
	}else{
		ESP_LOGE(TAG, "Send-image-event: Failed to open absolute calibration file.");
	}
	fclose(absfile);
	ESP_LOGI(TAG, "abs SC:  %+1.3f", abscal[0]);
	ESP_LOGI(TAG, "abs MC:  %+1.3f", abscal[1]);
	ESP_LOGI(TAG, "abs LC:  %+1.3f", abscal[2]);
	ESP_LOGI(TAG, "abs RH:  %+1.3f", abscal[3]);
	ESP_LOGI(TAG, "abs MEL: %+1.3f", abscal[4]);
	ESP_LOGI(TAG, "abs VL:  %+1.3f", abscal[5]);


	static imageformat RGB = PIXFORMAT_RGB;
	static imageformat FLOAT = PIXFORMAT_FLOAT;
	static imageformat HDR = PIXFORMAT_HDR;
	static imageformat AOPIC = PIXFORMAT_AOPIC;
	static imageformat MASK = PIXFORMAT_MASK;
	static imageformat REGIONS = PIXFORMAT_REGIONS;

	while(1)
	{
		// check for event
		send_image_event_status = xQueueReceive(image_data_queue, &(pic), pdMS_TO_TICKS(1000));

		if( send_image_event_status == pdPASS)
		{
			ESP_LOGI(TAG, "Send image data - image data queue -> received request");

			xQueueReceive(target_data_queue, &target, portMAX_DELAY);
			xQueueReceive(format_data_queue, &format, portMAX_DELAY);
			if(format==PIXFORMAT_MASK){
				xQueueReceive(mask_data_queue, &mask, portMAX_DELAY);
			}

			if( format == PIXFORMAT_HDR || format == PIXFORMAT_AOPIC || format == PIXFORMAT_MASK || format == PIXFORMAT_REGIONS ){
				xQueueReceive(image_data_queue, &(expt), pdMS_TO_TICKS(100));
			}


			// open calibration file(s)
			vigfile = fopen("/sdcard/calibration/vignetting.cal", "r");
			if (vigfile != NULL)
			{
				// open vignetting calibration
				ESP_LOGI(TAG, "send-image-event - Vignetting calibration file opened.");
			}else{
				ESP_LOGE(TAG, "Send-image-event: Failed to open vignetting calibration file.");
				vignetting = 0;
			}

			if(DEMOSAICING==BILINEAR){
				ESP_LOGI(TAG,"send-image-event: START BILINEAR INTERPOSLATION DEMOSAICING");
			}else if(DEMOSAICING==DOWNSAMPLING){
				ESP_LOGI(TAG,"send-image-event: START DOWNSAMPLING DEMOSAICING");
			}


			// IMAGE MASKS
			if( format == PIXFORMAT_MASK || format == PIXFORMAT_REGIONS ){
				// open mask files
				// https://github.com/espressif/esp-idf/blob/master/examples/protocols/http_server/file_serving/main/file_server.c#L81-L144
				char entrypath[32];
				const char *entrytype;
				struct dirent *entry;

				DIR *dir = opendir("/sdcard/calibration/masks");

				/* Retrieve the base path of file storage to construct the full path */
				strlcpy(entrypath, "/sdcard/calibration/masks", sizeof(entrypath));

				if (!dir) {
					ESP_LOGE(TAG, "Failed to open: /sdcard/calibration/masks");
					//return ESP_FAIL;
				}else{
					/* Iterate over all files / folders and fetch their names and sizes */
					filenum = 0;
					while (((entry = readdir(dir)) != NULL) && filenum<7) {

						entrytype = (entry->d_type == DT_DIR ? "directory" : "file");

						if(DT_DIR == 2){ // file found

							filenum++;
							ESP_LOGI(TAG, "Mask %d: %s", filenum, entry->d_name);

							// filename
							char folder[] = "/sdcard/calibration/masks/";
							char filename[sizeof(folder)+256];
							sprintf(filename, "/sdcard/calibration/masks/%s", entry->d_name);

							// MASK 1
							if(filenum==1){
								mask1 = fopen(filename, "r");
								if (mask1 != NULL)
								{
									// open mask file
									ESP_LOGI(TAG, "send-image-event - mask1: %s opened", entry->d_name);
								}else{
									ESP_LOGE(TAG, "Send-image-event: Failed to open mask1: %s", filename);
									//m1_factor = 0;
								}
							}
							// MASK 2
							if(filenum==2){
								mask2 = fopen(filename, "r");
								if (mask2 != NULL)
								{
									// open mask file
									ESP_LOGI(TAG, "send-image-event - mask2: %s opened", entry->d_name);
								}else{
									ESP_LOGE(TAG, "Send-image-event: Failed to open mask2: %s", filename);
									//m2_factor = 0;
								}
							}
							// MASK 3
							if(filenum==3){
								mask3 = fopen(filename, "r");
								if (mask3 != NULL)
								{
									// open mask file
									ESP_LOGI(TAG, "send-image-event - mask3: %s opened", entry->d_name);
								}else{
									ESP_LOGE(TAG, "Send-image-event: Failed to open mask3: %s", filename);
									//m3_factor = 0;
								}
							}
							// MASK 4
							if(filenum==4){
								mask4 = fopen(filename, "r");
								if (mask4 != NULL)
								{
									// open mask file
									ESP_LOGI(TAG, "send-image-event - mask4: %s opened", entry->d_name);
								}else{
									ESP_LOGE(TAG, "Send-image-event: Failed to open mask4: %s", filename);
									//m4_factor = 0;
								}
							}
							// MASK 5
							if(filenum==5){
								mask5 = fopen(filename, "r");
								if (mask5 != NULL)
								{
									// open mask file
									ESP_LOGI(TAG, "send-image-event - mask5: %s opened", entry->d_name);
								}else{
									ESP_LOGE(TAG, "Send-image-event: Failed to open mask5: %s", filename);
									//m5_factor = 0;
								}
							}
							// MASK 6
							if(filenum==6){
								mask6 = fopen(filename, "r");
								if (mask6 != NULL)
								{
									// open mask file
									ESP_LOGI(TAG, "send-image-event - mask6: %s opened", entry->d_name);
								}else{
									ESP_LOGE(TAG, "Send-image-event: Failed to open mask6: %s", filename);
									//m6_factor = 0;
								}
							}


						}
					}

					// OMEGA
					omega_file = fopen("/sdcard/calibration/omega.dat", "r");
					if (omega_file != NULL)
					{
						// open omega calibration
						ESP_LOGI(TAG, "send-image-event - omega.dat file opened");
					}else{
						ESP_LOGE(TAG, "Send-image-event: Failed to open omega.dat file");
					}

					// close folder
					//closedir(dir);
				}
			}


			uint8_t rawpixel_r;
			uint8_t rawpixel_g;
			uint8_t rawpixel_b;
			float floatpixel_R;
			float floatpixel_G;
			float floatpixel_B;
			float exptime1;
			float exptime2;
			float exptime3;
			float exptime4;
			float pixel1;
			float pixel2;
			float pixel3;
			float pixel4;
			float calRGB[3] = {1.0, 1.0, 1.0};
			float HDRpixel_R;
			float HDRpixel_G;
			float HDRpixel_B;
			float pixel_sc;
			float pixel_mc;
			float pixel_lc;
			float pixel_rh;
			float pixel_mel;
			float pixel_VL;
			uint8_t m1_factor = 0;
			uint8_t m2_factor = 0;
			uint8_t m3_factor = 0;
			uint8_t m4_factor = 0;
			uint8_t m5_factor = 0;
			uint8_t m6_factor = 0;
			float omega = 1.0;
			float omega_sum1 = 0;
			float omega_sum2 = 0;
			float omega_sum3 = 0;
			float omega_sum4 = 0;
			float omega_sum5 = 0;
			float omega_sum6 = 0;
			float masksum1[6] = {0,0,0,0,0,0};
			float masksum2[6] = {0,0,0,0,0,0};
			float masksum3[6] = {0,0,0,0,0,0};
			float masksum4[6] = {0,0,0,0,0,0};
			float masksum5[6] = {0,0,0,0,0,0};
			float masksum6[6] = {0,0,0,0,0,0};
			uint32_t row;
			uint32_t col;

			//ESP_LOGI(TAG, "Send-image-event: UART send image...");
			len = pic->len;
			//ESP_LOGI(TAG, "Send-image-event - UART send %zu bytes...", len);

			uint32_t logcounter = 1;
			if(SD_SAVE != 0){
				// initialize variables
				FILE *file = NULL;
				uint32_t folder = 1;
				uint32_t format = 0;

				uint8_t length = 255;
				char line[length];

				// read log file in binary mode
				ESP_LOGI(TAG, "Read log file");
				file = fopen("/sdcard/data/log", "r");
				if(file == NULL)
				{
					// create new log file
					fclose(file);
					file = NULL;
					ESP_LOGW(TAG, "Create new log file...");
					file = fopen("/sdcard/data/log", "w");
					// create folder and new log file
					if(file == NULL){
						char *folder_name = (char*)malloc(32*sizeof(char));
						memset(folder_name,0,32*sizeof(char));
						size_t x =  sprintf(folder_name,"/sdcard/data");
						size_t mk_ret = mkdir(folder_name, 0775);
						free(folder_name);
						if(file == NULL){
							fclose(file);
							file = NULL;
							file = fopen("/sdcard/data/log", "w");
						}
					}
				}else{
					// read all lines
					while (fgets(line, length, file))
					{
						//ESP_LOGI(TAG, "new line...");
						logcounter++;
					}
				}
				fclose(file);
				file = NULL;
				ESP_LOGI(TAG, "LOG counter: %zu", logcounter);

				// save file
				// allocate memory for filename string
				char *pic_name = pvPortMalloc(64*sizeof(char));

				switch(SD_SAVE){
				case SDRGB:
					ESP_LOGI(TAG, "Save-event: format RGB");
					sprintf(pic_name, "/sdcard/data/pic_%08zu.rgb", logcounter);
					ESP_LOGI(TAG,"Save-event: pic_name: %s",pic_name);
					break;
				case SDFLOAT:
					ESP_LOGI(TAG, "Save-event: format FLOAT");
					sprintf(pic_name, "/sdcard/data/pic_%08zu.flt", logcounter);
					ESP_LOGI(TAG,"Save-event: pic_name: %s",pic_name);
					break;
				case SDHDR:
					ESP_LOGI(TAG, "Save-event: format HDR");
					sprintf(pic_name, "/sdcard/data/pic_%08zu.hdr", logcounter);
					ESP_LOGI(TAG,"Save-event: pic_name: %s",pic_name);
					break;
				case SDAOPIC:
					ESP_LOGI(TAG, "Save-event: format AOPIC");
					sprintf(pic_name, "/sdcard/data/pic_%08zu.aop", logcounter);
					ESP_LOGI(TAG,"Save-event: pic_name: %s",pic_name);
					break;
				case SDNONE:
					break;
				case SDLOG:
					break;
				case SDALL:
					break;
				case SDGRAY:
					break;
				}

				// open image file
				imagefile = NULL;
				imagefile = fopen(pic_name, "w");
				free(pic_name);
			}

			demosaicing demo;

			if(DEMOSAICING==DOWNSAMPLING){
				ESP_LOGI(TAG, "Processing - %zu bytes in DOWNSAMPLING mode.",pic->len);
			}else if(DEMOSAICING==BILINEAR){
				ESP_LOGI(TAG, "Processing - %zu bytes in BILINEAR INTERPOLATION mode.",pic->len);
			}

			// DEMOSAICING LOOP
			for(size_t n = 0; n < pic->len; n++){

				//ESP_LOGI(TAG, "Processing - byte %zu", n);

				if(DEMOSAICING==DOWNSAMPLING){
					// determine row and column
					row = (uint32_t)(n/RESO1/2);
					col = (uint32_t)(n-row*RESO1*2);
					row++;
					col++;

					// detect odd or even row
					if((row%2 != 0) && (col%2 != 0)){
						// continue iteration
					}else{
						// skip iteration
						continue;
					}
				}

				// VIGNETTING
				if(vignetting && format != PIXFORMAT_RGB){
					// read next RGB subpixels
					size_t err = fread(calRGB, sizeof(float), 3, vigfile);
				}


				// MASK PIXEL
				if( format==PIXFORMAT_MASK || format==PIXFORMAT_REGIONS ){
					//ESP_LOGI(TAG, "Send-image-event - READ MASK VALUES");

					m1_factor = 1;
					m2_factor = 1;
					m3_factor = 1;
					m4_factor = 1;
					m5_factor = 1;
					m6_factor = 1;

					if( filenum > 5 ){
						size_t err = fread(&m6_factor, sizeof(uint8_t), 1, mask6);
					}
					if( filenum > 4 ){
						size_t err = fread(&m5_factor, sizeof(uint8_t), 1, mask5);
					}
					if( filenum > 3 ){
						size_t err = fread(&m4_factor, sizeof(uint8_t), 1, mask4);
					}
					if( filenum > 2 ){
						size_t err = fread(&m3_factor, sizeof(uint8_t), 1, mask3);
					}
					if( filenum > 1 ){
						size_t err = fread(&m2_factor, sizeof(uint8_t), 1, mask2);
					}
					if( filenum > 0 ){
						size_t err = fread(&m1_factor, sizeof(uint8_t), 1, mask1);
						//ESP_LOGI(TAG,"Number of masks: %d", m1_factor);
					}

					// read omega value
					size_t err = fread(&omega, sizeof(float), 1, omega_file);


					if(err!=1){
						omega = 0;
						ESP_LOGW(TAG,"Could not read from omega file!");
					}
				}

				// DEMOSAICING
				switch( DEMOSAICING ){
				// CALL DEMOSAICING FUNCTION
				case BILINEAR:
					// BILINEAR INTERPOLATION
					if( format == PIXFORMAT_MASK || format == PIXFORMAT_REGIONS ){
						demo = bilinear_demosaicing(pic, expt, AOPIC, RESO1, RESO2, n, calRGB[0], calRGB[1], calRGB[2]);
					}else{
						demo = bilinear_demosaicing(pic, expt, format, RESO1, RESO2, n, calRGB[0], calRGB[1], calRGB[2]);
					}
					break;
				case DOWNSAMPLING:
					// DEMOSAICING
					if( format==PIXFORMAT_MASK || format==PIXFORMAT_REGIONS ){
						demo = downsampling(pic, expt, AOPIC, RESO1*2, RESO2*2, n, calRGB[0], calRGB[1], calRGB[2]);
					}else{
						demo = downsampling(pic, expt, format, RESO1*2, RESO2*2, n, calRGB[0], calRGB[1], calRGB[2]);
					}
					break;
				default:
					break;
				}

				if(format==PIXFORMAT_REGIONS){
					// AOPIC PIXEL
					pixel_sc  = ( demo.hdrrgb[0]*relcal[0]  + demo.hdrrgb[1]*relcal[1]  + demo.hdrrgb[2]*relcal[2]  ) * abscal[0];
					pixel_mc  = ( demo.hdrrgb[0]*relcal[3]  + demo.hdrrgb[1]*relcal[4]  + demo.hdrrgb[2]*relcal[5]  ) * abscal[1];
					pixel_lc  = ( demo.hdrrgb[0]*relcal[6]  + demo.hdrrgb[1]*relcal[7]  + demo.hdrrgb[2]*relcal[8]  ) * abscal[2];
					pixel_rh  = ( demo.hdrrgb[0]*relcal[9]  + demo.hdrrgb[1]*relcal[10] + demo.hdrrgb[2]*relcal[11] ) * abscal[3];
					pixel_mel = ( demo.hdrrgb[0]*relcal[12] + demo.hdrrgb[1]*relcal[13] + demo.hdrrgb[2]*relcal[14] ) * abscal[4];
					pixel_VL  = ( demo.hdrrgb[0]*relcal[15] + demo.hdrrgb[1]*relcal[16] + demo.hdrrgb[2]*relcal[17] ) * abscal[5];

					//ESP_LOGI(TAG,"Number of masks: %d", filenum);

					for(uint8_t nmask=1; nmask<=filenum; nmask++){
						switch(nmask){
						case 1:
							omega_sum1  = omega_sum1 + (omega * uint8_2_float(m1_factor));
							masksum1[0] = masksum1[0] + (pixel_sc  *omega * uint8_2_float(m1_factor));
							masksum1[1] = masksum1[1] + (pixel_mc  *omega * uint8_2_float(m1_factor));
							masksum1[2] = masksum1[2] + (pixel_lc  *omega * uint8_2_float(m1_factor));
							masksum1[3] = masksum1[3] + (pixel_rh  *omega * uint8_2_float(m1_factor));
							masksum1[4] = masksum1[4] + (pixel_mel *omega * uint8_2_float(m1_factor));
							masksum1[5] = masksum1[5] + (pixel_VL  *omega * uint8_2_float(m1_factor));
							break;
						case 2:
							omega_sum2  = omega_sum2 + (omega * uint8_2_float(m2_factor));
							masksum2[0] = masksum2[0] + (pixel_sc  *omega * uint8_2_float(m2_factor));
							masksum2[1] = masksum2[1] + (pixel_mc  *omega * uint8_2_float(m2_factor));
							masksum2[2] = masksum2[2] + (pixel_lc  *omega * uint8_2_float(m2_factor));
							masksum2[3] = masksum2[3] + (pixel_rh  *omega * uint8_2_float(m2_factor));
							masksum2[4] = masksum2[4] + (pixel_mel *omega * uint8_2_float(m2_factor));
							masksum2[5] = masksum2[5] + (pixel_VL  *omega * uint8_2_float(m2_factor));
							break;
						case 3:
							omega_sum3  = omega_sum3 + (omega * uint8_2_float(m3_factor));
							masksum3[0] = masksum3[0] + (pixel_sc  *omega * uint8_2_float(m3_factor));
							masksum3[1] = masksum3[1] + (pixel_mc  *omega * uint8_2_float(m3_factor));
							masksum3[2] = masksum3[2] + (pixel_lc  *omega * uint8_2_float(m3_factor));
							masksum3[3] = masksum3[3] + (pixel_rh  *omega * uint8_2_float(m3_factor));
							masksum3[4] = masksum3[4] + (pixel_mel *omega * uint8_2_float(m3_factor));
							masksum3[5] = masksum3[5] + (pixel_VL  *omega * uint8_2_float(m3_factor));
							break;
						case 4:
							omega_sum4  =  omega_sum4 + (omega * uint8_2_float(m4_factor));
							masksum4[0] = masksum4[0] + (pixel_sc  *omega * uint8_2_float(m4_factor));
							masksum4[1] = masksum4[1] + (pixel_mc  *omega * uint8_2_float(m4_factor));
							masksum4[2] = masksum4[2] + (pixel_lc  *omega * uint8_2_float(m4_factor));
							masksum4[3] = masksum4[3] + (pixel_rh  *omega * uint8_2_float(m4_factor));
							masksum4[4] = masksum4[4] + (pixel_mel *omega * uint8_2_float(m4_factor));
							masksum4[5] = masksum4[5] + (pixel_VL  *omega * uint8_2_float(m4_factor));
							break;
						case 5:
							omega_sum5  = omega_sum5 + (omega * uint8_2_float(m5_factor));
							masksum5[0] = masksum5[0] + (pixel_sc  *omega * uint8_2_float(m5_factor));
							masksum5[1] = masksum5[1] + (pixel_mc  *omega * uint8_2_float(m5_factor));
							masksum5[2] = masksum5[2] + (pixel_lc  *omega * uint8_2_float(m5_factor));
							masksum5[3] = masksum5[3] + (pixel_rh  *omega * uint8_2_float(m5_factor));
							masksum5[4] = masksum5[4] + (pixel_mel *omega * uint8_2_float(m5_factor));
							masksum5[5] = masksum5[5] + (pixel_VL  *omega * uint8_2_float(m5_factor));
							break;
						case 6:
							omega_sum6  = omega_sum6 + (omega * uint8_2_float(m6_factor));
							masksum6[0] = masksum6[0] + (pixel_sc  *omega * uint8_2_float(m6_factor));
							masksum6[1] = masksum6[1] + (pixel_mc  *omega * uint8_2_float(m6_factor));
							masksum6[2] = masksum6[2] + (pixel_lc  *omega * uint8_2_float(m6_factor));
							masksum6[3] = masksum6[3] + (pixel_rh  *omega * uint8_2_float(m6_factor));
							masksum6[4] = masksum6[4] + (pixel_mel *omega * uint8_2_float(m6_factor));
							masksum6[5] = masksum6[5] + (pixel_VL  *omega * uint8_2_float(m6_factor));
							break;
						default:
							break;
						}
					}
				}

				// send over UART
				if(UART_SEND == 1){
					//ESP_LOGI(TAG, "Send-image-event - send to UART");
					switch(format)
					{
					case PIXFORMAT_RGB:
						//ESP_LOGI(TAG, "Send-image-event - RAW RGB FORMAT");
						uart_write_bytes(EX_UART_NUM, (const char*)&demo.rawrgb[0], 1);
						uart_write_bytes(EX_UART_NUM, (const char*)&demo.rawrgb[1], 1);
						uart_write_bytes(EX_UART_NUM, (const char*)&demo.rawrgb[2], 1);
						break;
					case PIXFORMAT_nRGB:
						uart_write_bytes(EX_UART_NUM, (const char*)&demo.rawrgb[0], 1);
						uart_write_bytes(EX_UART_NUM, (const char*)&demo.rawrgb[1], 1);
						uart_write_bytes(EX_UART_NUM, (const char*)&demo.rawrgb[2], 1);
						break;
					case PIXFORMAT_FLOAT:
						//ESP_LOGI(TAG, "Send-image-event - FLOAT FORMAT");
						uart_write_bytes(EX_UART_NUM, (const char*)&demo.floatrgb[0], 4);
						uart_write_bytes(EX_UART_NUM, (const char*)&demo.floatrgb[1], 4);
						uart_write_bytes(EX_UART_NUM, (const char*)&demo.floatrgb[2], 4);
						break;
					case PIXFORMAT_nFLOAT:
						uart_write_bytes(EX_UART_NUM, (const char*)&demo.floatrgb[0], 4);
						uart_write_bytes(EX_UART_NUM, (const char*)&demo.floatrgb[1], 4);
						uart_write_bytes(EX_UART_NUM, (const char*)&demo.floatrgb[2], 4);
						break;
					case PIXFORMAT_HDR:
						uart_write_bytes(EX_UART_NUM, (const char*)&demo.hdrrgb[0], 4);
						uart_write_bytes(EX_UART_NUM, (const char*)&demo.hdrrgb[1], 4);
						uart_write_bytes(EX_UART_NUM, (const char*)&demo.hdrrgb[2], 4);
						break;
					case PIXFORMAT_AOPIC:
						pixel_sc = (demo.hdrrgb[0]*relcal[0] + demo.hdrrgb[1]*relcal[1] + demo.hdrrgb[2]*relcal[2]) * abscal[0];
						uart_write_bytes(EX_UART_NUM, (const char*)&pixel_sc, 4);
						pixel_mc = (demo.hdrrgb[0]*relcal[3] + demo.hdrrgb[1]*relcal[4] + demo.hdrrgb[2]*relcal[5]) * abscal[1];
						uart_write_bytes(EX_UART_NUM, (const char*)&pixel_mc, 4);
						pixel_lc = (demo.hdrrgb[0]*relcal[6] + demo.hdrrgb[1]*relcal[7] + demo.hdrrgb[2]*relcal[8]) * abscal[2];
						uart_write_bytes(EX_UART_NUM, (const char*)&pixel_lc, 4);
						pixel_rh = (demo.hdrrgb[0]*relcal[9] + demo.hdrrgb[1]*relcal[10] + demo.hdrrgb[2]*relcal[11]) * abscal[3];
						uart_write_bytes(EX_UART_NUM, (const char*)&pixel_rh, 4);
						pixel_mel = (demo.hdrrgb[0]*relcal[12] + demo.hdrrgb[1]*relcal[13] + demo.hdrrgb[2]*relcal[14]) * abscal[4];
						uart_write_bytes(EX_UART_NUM, (const char*)&pixel_mel, 4);
						pixel_VL = (demo.hdrrgb[0]*relcal[15] + demo.hdrrgb[1]*relcal[16] + demo.hdrrgb[2]*relcal[17]) * abscal[5];
						uart_write_bytes(EX_UART_NUM, (const char*)&pixel_VL, 4);
						break;
					case PIXFORMAT_MASK:
						//ESP_LOGI(TAG, "Send-image-event - SEND MASK FORMAT");
						switch(mask){
						case 1:
							maskfactor = uint8_2_float(m1_factor);
							break;
						case 2:
							maskfactor = uint8_2_float(m2_factor);
							break;
						case 3:
							maskfactor = uint8_2_float(m3_factor);
							break;
						case 4:
							maskfactor = uint8_2_float(m4_factor);
							break;
						case 5:
							maskfactor = uint8_2_float(m5_factor);
							break;
						case 6:
							maskfactor = uint8_2_float(m6_factor);
							break;
						}
						pixel_sc = (demo.hdrrgb[0]*relcal[0] + demo.hdrrgb[1]*relcal[1] + demo.hdrrgb[2]*relcal[2]) * abscal[0] * maskfactor;
						uart_write_bytes(EX_UART_NUM, (const char*)&pixel_sc, 4);
						pixel_mc = (demo.hdrrgb[0]*relcal[3] + demo.hdrrgb[1]*relcal[4] + demo.hdrrgb[2]*relcal[5]) * abscal[1] * maskfactor;
						uart_write_bytes(EX_UART_NUM, (const char*)&pixel_mc, 4);
						pixel_lc = (demo.hdrrgb[0]*relcal[6] + demo.hdrrgb[1]*relcal[7] + demo.hdrrgb[2]*relcal[8]) * abscal[2] * maskfactor;
						uart_write_bytes(EX_UART_NUM, (const char*)&pixel_lc, 4);
						pixel_rh = (demo.hdrrgb[0]*relcal[9] + demo.hdrrgb[1]*relcal[10] + demo.hdrrgb[2]*relcal[11]) * abscal[3] * maskfactor;
						uart_write_bytes(EX_UART_NUM, (const char*)&pixel_rh, 4);
						pixel_mel = (demo.hdrrgb[0]*relcal[12] + demo.hdrrgb[1]*relcal[13] + demo.hdrrgb[2]*relcal[14]) * abscal[4] * maskfactor;
						uart_write_bytes(EX_UART_NUM, (const char*)&pixel_mel, 4);
						pixel_VL = (demo.hdrrgb[0]*relcal[15] + demo.hdrrgb[1]*relcal[16] + demo.hdrrgb[2]*relcal[17]) * abscal[5] * maskfactor;
						uart_write_bytes(EX_UART_NUM, (const char*)&pixel_VL, 4);
						break;
					case PIXFORMAT_REGIONS:
							break;
						default:
							//ESP_LOGE(TAG, "send/save-image-event - unsupported format!");
							break;
					}
				}


				// save to SD card
				if(SD_SAVE != SDNONE){
					switch(SD_SAVE){
					case SDRGB:
						//ESP_LOGI(TAG, "Send-image-event - Write to file");
						fwrite(&demo.rawrgb[0], sizeof(uint8_t), 1, imagefile);
						fwrite(&demo.rawrgb[1], sizeof(uint8_t), 1, imagefile);
						fwrite(&demo.rawrgb[2], sizeof(uint8_t), 1, imagefile);
						break;
					case SDFLOAT:
						fwrite(&demo.floatrgb[0], sizeof(float), 1, imagefile);
						fwrite(&demo.floatrgb[1], sizeof(float), 1, imagefile);
						fwrite(&demo.floatrgb[2], sizeof(float), 1, imagefile);
						break;
					case SDHDR:
						fwrite(&demo.hdrrgb[0], sizeof(float), 1, imagefile);
						fwrite(&demo.hdrrgb[1], sizeof(float), 1, imagefile);
						fwrite(&demo.hdrrgb[2], sizeof(float), 1, imagefile);
						break;
					case SDAOPIC:
						pixel_sc = (demo.hdrrgb[0]*relcal[0] + demo.hdrrgb[1]*relcal[1] + demo.hdrrgb[2]*relcal[2]) * abscal[0];
						fwrite(&pixel_sc, sizeof(float), 1, imagefile);
						pixel_mc = (demo.hdrrgb[0]*relcal[3] + demo.hdrrgb[1]*relcal[4] + demo.hdrrgb[2]*relcal[5]) * abscal[1];
						fwrite(&pixel_mc, sizeof(float), 1, imagefile);
						pixel_lc = (demo.hdrrgb[0]*relcal[6] + demo.hdrrgb[1]*relcal[7] + demo.hdrrgb[2]*relcal[8]) * abscal[2];
						fwrite(&pixel_lc, sizeof(float), 1, imagefile);
						pixel_rh = (demo.hdrrgb[0]*relcal[9] + demo.hdrrgb[1]*relcal[10] + demo.hdrrgb[2]*relcal[11]) * abscal[3];
						fwrite(&pixel_rh, sizeof(float), 1, imagefile);
						pixel_mel = (demo.hdrrgb[0]*relcal[12] + demo.hdrrgb[1]*relcal[13] + demo.hdrrgb[2]*relcal[14]) * abscal[4];
						fwrite(&pixel_mel,sizeof(float), 1, imagefile);
						pixel_VL = (demo.hdrrgb[0]*relcal[15] + demo.hdrrgb[1]*relcal[16] + demo.hdrrgb[2]*relcal[17]) * abscal[5];
						fwrite(&pixel_VL, sizeof(float), 1, imagefile);
						break;
					case SDLOG:
						// log region values after loop
						break;
					default:
						break;
					}
				}

				//ESP_LOGI(TAG,"pixel #%zu completed.", n);

			}

			ESP_LOGI(TAG,"Processing - Image processing finished.");
			if(SD_SAVE != 0){
				ESP_LOGI(TAG, "Close image file.");
				if(format != PIXFORMAT_REGIONS){
					FILE *logfile = fopen("/sdcard/data/log", "a");
					fprintf(logfile, "%zu\n", logcounter);
					fclose(logfile);
				}

				fclose(imagefile);
				imagefile = NULL;
			}

			// close files
			fclose(vigfile);
			if(format==PIXFORMAT_MASK){
				ESP_LOGI(TAG,"Close image-mask files.");
				if( filenum > 5 ){
					fclose(mask6);
				}
				if( filenum > 4 ){
					fclose(mask5);
				}
				if( filenum > 3 ){
					fclose(mask4);
				}
				if( filenum > 2 ){
					fclose(mask3);
				}
				if( filenum > 1 ){
					fclose(mask2);
				}
				if( filenum > 0 ){
					fclose(mask1);
				}
			}

			// REGIONS
			if(format==PIXFORMAT_REGIONS){
				ESP_LOGI(TAG,"Sending region values: %d x 6", filenum);

				// close omega file
				fclose(omega_file);
				//fclose(dir);

				// open log file
				ESP_LOGI(TAG, "Open log file...");
				FILE *logfile = NULL;
				logfile = fopen("/sdcard/data/log", "a");
				// add log counter
				fprintf(logfile, "%08zu ", logcounter);

				// close mask files & send data
				if( filenum > 0 ){
					fclose(mask1);
					// BLE send
					if(BLE == 1){
						float sc = masksum1[0]/omega_sum1;
						VALUES[0] = sc;
						float mc = masksum1[1]/omega_sum1;
						VALUES[1] = mc;
						float lc = masksum1[2]/omega_sum1;
						VALUES[2] = lc;
						float rh = masksum1[3]/omega_sum1;
						VALUES[3] = rh;
						float mel = masksum1[4]/omega_sum1;
						VALUES[4] = mel;
						float VL = masksum1[5]/omega_sum1;
						VALUES[5] = VL;
					}
					// send to UART
					if(UART_SEND == 1){
						//uart_write_bytes(EX_UART_NUM, (const char*)&omega_sum1, 4);
						float sc = masksum1[0]/omega_sum1;
						uart_write_bytes(EX_UART_NUM, (const char*)&sc, 4);
						float mc = masksum1[1]/omega_sum1;
						uart_write_bytes(EX_UART_NUM, (const char*)&mc, 4);
						float lc = masksum1[2]/omega_sum1;
						uart_write_bytes(EX_UART_NUM, (const char*)&lc, 4);
						float rh = masksum1[3]/omega_sum1;
						uart_write_bytes(EX_UART_NUM, (const char*)&rh, 4);
						float mel = masksum1[4]/omega_sum1;
						uart_write_bytes(EX_UART_NUM, (const char*)&mel, 4);
						float VL = masksum1[5]/omega_sum1;
						uart_write_bytes(EX_UART_NUM, (const char*)&VL, 4);
					}
					// save to SD card
					if(SD_SAVE == SDLOG){
						//fprintf(logfile, "%f ", omega_sum1);
						float sc = masksum1[0]/omega_sum1;
						fprintf(logfile, "%f ", sc);
						float mc = masksum1[1]/omega_sum1;
						fprintf(logfile, "%f ", mc);
						float lc = masksum1[2]/omega_sum1;
						fprintf(logfile, "%f ", lc);
						float rh = masksum1[3]/omega_sum1;
						fprintf(logfile, "%f ", rh);
						float mel = masksum1[4]/omega_sum1;
						fprintf(logfile, "%f ", mel);
						float VL = masksum1[5]/omega_sum1;
						fprintf(logfile, "%f ", VL);
					}
				}
				if( filenum > 1 ){
					fclose(mask2);
					// BLE send
					if(BLE == 1){
						float sc = masksum2[0]/omega_sum2;
						VALUES[6] = sc;
						float mc = masksum2[1]/omega_sum2;
						VALUES[7] = mc;
						float lc = masksum2[2]/omega_sum2;
						VALUES[8] = lc;
						float rh = masksum2[3]/omega_sum2;
						VALUES[9] = rh;
						float mel = masksum2[4]/omega_sum2;
						VALUES[10] = mel;
						float VL = masksum2[5]/omega_sum2;
						VALUES[11] = VL;
					}
					// send to UART
					if(UART_SEND == 1){
						float sc = masksum2[0]/omega_sum2;
						uart_write_bytes(EX_UART_NUM, (const char*)&sc, 4);
						float mc = masksum2[1]/omega_sum2;
						uart_write_bytes(EX_UART_NUM, (const char*)&mc, 4);
						float lc = masksum2[2]/omega_sum2;
						uart_write_bytes(EX_UART_NUM, (const char*)&lc, 4);
						float rh = masksum2[3]/omega_sum2;
						uart_write_bytes(EX_UART_NUM, (const char*)&rh, 4);
						float mel = masksum2[4]/omega_sum2;
						uart_write_bytes(EX_UART_NUM, (const char*)&mel, 4);
						float VL = masksum2[5]/omega_sum2;
						uart_write_bytes(EX_UART_NUM, (const char*)&VL, 4);
					}
					// save to SD card
					if(SD_SAVE == SDLOG){
						//fprintf(logfile, "%f ", omega_sum2);
						float sc = masksum2[0]/omega_sum2;
						fprintf(logfile, "%f ", sc);
						float mc = masksum2[1]/omega_sum2;
						fprintf(logfile, "%f ", mc);
						float lc = masksum2[2]/omega_sum2;
						fprintf(logfile, "%f ", lc);
						float rh = masksum2[3]/omega_sum2;
						fprintf(logfile, "%f ", rh);
						float mel = masksum2[4]/omega_sum2;
						fprintf(logfile, "%f ", mel);
						float VL = masksum2[5]/omega_sum2;
						fprintf(logfile, "%f ", VL);
					}
				}
				if( filenum > 2 ){
					fclose(mask3);
					// BLE send
					if(BLE == 1){
						float sc = masksum3[0]/omega_sum3;
						VALUES[12] = sc;
						float mc = masksum3[1]/omega_sum3;
						VALUES[13] = mc;
						float lc = masksum3[2]/omega_sum3;
						VALUES[14] = lc;
						float rh = masksum3[3]/omega_sum3;
						VALUES[15] = rh;
						float mel = masksum3[4]/omega_sum3;
						VALUES[16] = mel;
						float VL = masksum3[5]/omega_sum3;
						VALUES[17] = VL;
					}
					// send to UART
					if(UART_SEND == 1){
						float sc = masksum3[0]/omega_sum3;
						uart_write_bytes(EX_UART_NUM, (const char*)&sc, 4);
						float mc = masksum3[1]/omega_sum3;
						uart_write_bytes(EX_UART_NUM, (const char*)&mc, 4);
						float lc = masksum3[2]/omega_sum3;
						uart_write_bytes(EX_UART_NUM, (const char*)&lc, 4);
						float rh = masksum3[3]/omega_sum3;
						uart_write_bytes(EX_UART_NUM, (const char*)&rh, 4);
						float mel = masksum3[4]/omega_sum3;
						uart_write_bytes(EX_UART_NUM, (const char*)&mel, 4);
						float VL = masksum3[5]/omega_sum3;
						uart_write_bytes(EX_UART_NUM, (const char*)&VL, 4);
					}
					// save to SD card
					if(SD_SAVE == SDLOG){
						float sc = masksum3[0]/omega_sum3;
						fprintf(logfile, "%f ", sc);
						float mc = masksum3[1]/omega_sum3;
						fprintf(logfile, "%f ", mc);
						float lc = masksum3[2]/omega_sum3;
						fprintf(logfile, "%f ", lc);
						float rh = masksum3[3]/omega_sum3;
						fprintf(logfile, "%f ", rh);
						float mel = masksum3[4]/omega_sum3;
						fprintf(logfile, "%f ", mel);
						float VL = masksum3[5]/omega_sum3;
						fprintf(logfile, "%f ", VL);
					}
				}
				if( filenum > 3 ){
					fclose(mask4);
					// BLE send
					if(BLE == 1){
						float sc = masksum4[0]/omega_sum4;
						VALUES[18] = sc;
						float mc = masksum4[1]/omega_sum4;
						VALUES[19] = mc;
						float lc = masksum4[2]/omega_sum4;
						VALUES[20] = lc;
						float rh = masksum4[3]/omega_sum4;
						VALUES[21] = rh;
						float mel = masksum4[4]/omega_sum4;
						VALUES[22] = mel;
						float VL = masksum4[5]/omega_sum4;
						VALUES[23] = VL;
					}
					// send to UART
					if(UART_SEND == 1){
						float sc = masksum4[0]/omega_sum4;
						uart_write_bytes(EX_UART_NUM, (const char*)&sc, 4);
						float mc = masksum4[1]/omega_sum4;
						uart_write_bytes(EX_UART_NUM, (const char*)&mc, 4);
						float lc = masksum4[2]/omega_sum4;
						uart_write_bytes(EX_UART_NUM, (const char*)&lc, 4);
						float rh = masksum4[3]/omega_sum4;
						uart_write_bytes(EX_UART_NUM, (const char*)&rh, 4);
						float mel = masksum4[4]/omega_sum4;
						uart_write_bytes(EX_UART_NUM, (const char*)&mel, 4);
						float VL = masksum4[5]/omega_sum4;
						uart_write_bytes(EX_UART_NUM, (const char*)&VL, 4);
					}
					// save to SD card
					if(SD_SAVE == SDLOG){
						float sc = masksum4[0]/omega_sum4;
						fprintf(logfile, "%f ", sc);
						float mc = masksum4[1]/omega_sum4;
						fprintf(logfile, "%f ", mc);
						float lc = masksum4[2]/omega_sum4;
						fprintf(logfile, "%f ", lc);
						float rh = masksum4[3]/omega_sum4;
						fprintf(logfile, "%f ", rh);
						float mel = masksum4[4]/omega_sum4;
						fprintf(logfile, "%f ", mel);
						float VL = masksum4[5]/omega_sum4;
						fprintf(logfile, "%f ", VL);
					}
				}
				if( filenum > 4 ){
					fclose(mask5);
					// BLE send
					if(BLE == 1){
						float sc = masksum5[0]/omega_sum5;
						VALUES[24] = sc;
						float mc = masksum5[1]/omega_sum5;
						VALUES[25] = mc;
						float lc = masksum5[2]/omega_sum5;
						VALUES[26] = lc;
						float rh = masksum5[3]/omega_sum5;
						VALUES[27] = rh;
						float mel = masksum5[4]/omega_sum5;
						VALUES[28] = mel;
						float VL = masksum5[5]/omega_sum5;
						VALUES[29] = VL;
					}
					// send to UART
					if(UART_SEND == 1){
						float sc = masksum5[0]/omega_sum5;
						uart_write_bytes(EX_UART_NUM, (const char*)&sc, 4);
						float mc = masksum5[1]/omega_sum5;
						uart_write_bytes(EX_UART_NUM, (const char*)&mc, 4);
						float lc = masksum5[2]/omega_sum5;
						uart_write_bytes(EX_UART_NUM, (const char*)&lc, 4);
						float rh = masksum5[3]/omega_sum5;
						uart_write_bytes(EX_UART_NUM, (const char*)&rh, 4);
						float mel = masksum5[4]/omega_sum5;
						uart_write_bytes(EX_UART_NUM, (const char*)&mel, 4);
						float VL = masksum5[5]/omega_sum5;
						uart_write_bytes(EX_UART_NUM, (const char*)&VL, 4);
					}
					// save to SD card
					if(SD_SAVE == SDLOG){
						float sc = masksum5[0]/omega_sum5;
						fprintf(logfile, "%f ", sc);
						float mc = masksum5[1]/omega_sum5;
						fprintf(logfile, "%f ", mc);
						float lc = masksum5[2]/omega_sum5;
						fprintf(logfile, "%f ", lc);
						float rh = masksum5[3]/omega_sum5;
						fprintf(logfile, "%f ", rh);
						float mel = masksum5[4]/omega_sum5;
						fprintf(logfile, "%f ", mel);
						float VL = masksum5[5]/omega_sum5;
						fprintf(logfile, "%f ", VL);
					}
				}
				if( filenum > 5 ){
					fclose(mask6);
					// BLE send
					if(BLE == 1){
						float sc = masksum6[0]/omega_sum6;
						VALUES[30] = sc;
						float mc = masksum6[1]/omega_sum6;
						VALUES[31] = mc;
						float lc = masksum6[2]/omega_sum6;
						VALUES[32] = lc;
						float rh = masksum6[3]/omega_sum6;
						VALUES[33] = rh;
						float mel = masksum6[4]/omega_sum6;
						VALUES[34] = mel;
						float VL = masksum6[5]/omega_sum6;
						VALUES[35] = VL;
					}
					// send to UART
					if(UART_SEND == 1){
						float sc = masksum6[0]/omega_sum6;
						uart_write_bytes(EX_UART_NUM, (const char*)&sc, 4);
						float mc = masksum6[1]/omega_sum6;
						uart_write_bytes(EX_UART_NUM, (const char*)&mc, 4);
						float lc = masksum6[2]/omega_sum6;
						uart_write_bytes(EX_UART_NUM, (const char*)&lc, 4);
						float rh = masksum6[3]/omega_sum6;
						uart_write_bytes(EX_UART_NUM, (const char*)&rh, 4);
						float mel = masksum6[4]/omega_sum6;
						uart_write_bytes(EX_UART_NUM, (const char*)&mel, 4);
						float VL = masksum6[5]/omega_sum6;
						uart_write_bytes(EX_UART_NUM, (const char*)&VL, 4);
					}
					// save to SD card
					if(SD_SAVE == SDLOG){
						float sc = masksum6[0]/omega_sum6;
						fprintf(logfile, "%f ", sc);
						float mc = masksum6[1]/omega_sum6;
						fprintf(logfile, "%f ", mc);
						float lc = masksum6[2]/omega_sum6;
						fprintf(logfile, "%f ", lc);
						float rh = masksum6[3]/omega_sum6;
						fprintf(logfile, "%f ", rh);
						float mel = masksum6[4]/omega_sum6;
						fprintf(logfile, "%f ", mel);
						float VL = masksum6[5]/omega_sum6;
						fprintf(logfile, "%f ", VL);
					}
				}
				// close log file
				ESP_LOGI(TAG, "Close log file...");
				fprintf(logfile, "\n");
				fclose(logfile);
				logfile = NULL;
			}



			// confirm save image/log file
			if(SEND_SAVED==1 && SD_SAVE!=SDNONE){
				uint32_t confirm = 1;
				uart_write_bytes(EX_UART_NUM, (const char*)&confirm, 4);
			}
		}
	}
}


// create HDR image
void HDR_image_event_task(void *pvParameters){

	static target UART = 0;
	static target SD = 1;
	static imageformat sendRGB = PIXFORMAT_RGB;
	static imageformat sendFLOAT = PIXFORMAT_FLOAT;
	static imageformat sendHDR = PIXFORMAT_HDR;
	static imageformat sendAOPIC = PIXFORMAT_AOPIC;
	static imageformat sendnRGB = PIXFORMAT_nRGB;
	static imageformat sendnFLOAT = PIXFORMAT_nFLOAT;
	static imageformat sendMASK = PIXFORMAT_MASK;
	static imageformat sendREGION = PIXFORMAT_REGIONS;

	ESP_LOGI(TAG, "Create-HDR-image: start HDR image task...");

	// HDR image buffer
	ESP_LOGI(TAG, "Create-HDR-image: allocate HDR image 8bit buffer...");
	imagebuffer *hdrbuf = (imagebuffer *)malloc(sizeof(imagebuffer));
	ESP_LOGI(TAG, "Create-HDR-image: allocate exposure 8bit buffer...");
	imagebuffer *expbuf = (imagebuffer *)malloc(sizeof(imagebuffer));
	if(!hdrbuf) {
		ESP_LOGE(TAG, "Create-HDR-image: Could not allocate memory for HDR image buffer creation...");
	}
	if(!expbuf) {
		ESP_LOGE(TAG, "Create-HDR-image: Could not allocate memory for HDR exposure buffer creation...");
	}

	uint32_t bufreso1 = RESO1;
	uint32_t bufreso2 = RESO2;

	if(DEMOSAICING==DOWNSAMPLING){
		bufreso1 = RESO1*2;
		bufreso2 = RESO2*2;
	}

	esp_err_t buffer = init_imbuffer(hdrbuf, bufreso1*bufreso2);
	if(buffer!=ESP_OK){
		ESP_LOGE(TAG, "Create-HDR-image: Could not initialize HDR image buffer...");
	}
	buffer = init_imbuffer(expbuf, bufreso1*bufreso2);
	if(buffer!=ESP_OK){
		ESP_LOGE(TAG, "Create-HDR-image: Could not initialize exposure matrix buffer...");
	}


	// task variables
	BaseType_t image_event_status;

	imagebuffer *im;
	size_t pixel;
	static uint32_t send0 = 0;
	static uint32_t send1 = 1;
	uint32_t pic;
	uint32_t maxpicnumber;
	uint32_t mask;
	imageformat format;


	// Task loop
	while(1)
	{

		// check for event
		image_event_status = xQueueReceive(HDR_image_data_queue, &(im), pdMS_TO_TICKS(100));

		if( image_event_status == pdPASS )
		{
			if(im->format==PIXFORMAT_MASK){
				xQueueReceive(HDR_data_queue, &(mask), pdMS_TO_TICKS(1000));
			}
			xQueueReceive(HDR_data_queue, &(pic), pdMS_TO_TICKS(1000));
			xQueueReceive(HDR_data_queue, &(maxpicnumber), pdMS_TO_TICKS(1000));
			xQueueReceive(format_data_queue, &(format), pdMS_TO_TICKS(1000));

			ESP_LOGI(TAG, "Create-HDR-image: image %zu of %zu", pic, maxpicnumber);

			// process images
			if(pic==1){
				// get first image

				hdrbuf->format = im->format;

				// fill image buffer
				for(size_t n = 0; n <= hdrbuf->len; n++){
					hdrbuf->buf[n] = im->buf[n];
					expbuf->buf[n] = 0;
				}
				ESP_LOGI(TAG, "Create-HDR-image: RAW image nr. %d processed.", pic);
			}else{
				//

				// check for image format
				if(format==PIXFORMAT_nRGB || format==PIXFORMAT_nFLOAT){

					// nRGB or nFLOAT

					// pixel loop
					for(pixel = 0; pixel <= hdrbuf->len; pixel++){

						// average pixel
						uint16_t value = ((hdrbuf->buf[pixel] + im->buf[pixel]) / 2);
						hdrbuf->buf[pixel] = (uint8_t*)value;

					}

				}else if( format==PIXFORMAT_HDR || format==PIXFORMAT_AOPIC || format==PIXFORMAT_MASK || format==PIXFORMAT_REGIONS ){

					// HDR or AOPIC
					uint8_t ind = pic-1;
					uint16_t int_time = dtime[ind]-1;
					ESP_LOGI(TAG, "Create-HDR-image: integration time %d", int_time);
					// HDR loop
					for(pixel = 0; pixel <= hdrbuf->len; pixel++){
						// check sub-pixel signal strength
						if(hdrbuf->buf[pixel] < lo && im->buf[pixel] < hi)
						{
							// overwrite pixel
							hdrbuf->buf[pixel] = im->buf[pixel];
							expbuf->buf[pixel] = (uint8_t*)int_time;
						}
					}
				}
				ESP_LOGI(TAG, "Create-HDR-image: RAW image %d processed.\n", pic);

			}


			// send & save HDR image
			if(pic == maxpicnumber){

				ESP_LOGI(TAG, "Create-HDR-image: image format %d", hdrbuf->format);

				// send image
				switch(format){
				case PIXFORMAT_RGB:
					break;
				case  PIXFORMAT_FLOAT:
					break;
				case PIXFORMAT_HDR: // HDR -> UART
					ESP_LOGI(TAG, "Camera-event: Send HDR to demosaicing.");
					xQueueSendToBack(image_data_queue, (void*)&hdrbuf, portMAX_DELAY); // send image
					xQueueSendToBack(target_data_queue, (target*)&UART, pdMS_TO_TICKS(1000));
					xQueueSendToBack(format_data_queue, (imageformat*)&sendHDR, pdMS_TO_TICKS(1000));
					xQueueSendToBack(image_data_queue, (void*)&expbuf, portMAX_DELAY); // send exposure data
					break;
				case PIXFORMAT_AOPIC: // AOPIC -> UART
					ESP_LOGI(TAG, "Camera-event: Send AOPIC to demosaicing.");
					xQueueSendToBack(image_data_queue, (void*)&hdrbuf, portMAX_DELAY); // send image
					xQueueSendToBack(target_data_queue, (target*)&UART, pdMS_TO_TICKS(1000));
					xQueueSendToBack(format_data_queue, (imageformat*)&sendAOPIC, pdMS_TO_TICKS(1000));
					xQueueSendToBack(image_data_queue, (void*)&expbuf, portMAX_DELAY); // send exposure data
					break;
				case PIXFORMAT_nRGB: // nRGB -> UART
					ESP_LOGI(TAG, "Camera-event: Send nRGB to demosaicing.");
					hdrbuf->format = PIXFORMAT_RGB;
					xQueueSendToBack(image_data_queue, (void*)&hdrbuf, portMAX_DELAY); // send image
					xQueueSendToBack(target_data_queue, (target*)&UART, pdMS_TO_TICKS(1000));
					xQueueSendToBack(format_data_queue, (imageformat*)&sendRGB, pdMS_TO_TICKS(1000));
					break;
				case PIXFORMAT_nFLOAT: //nFLOAT -> UART
					ESP_LOGI(TAG, "Camera-event: Send nFLOAT to demosaicing.");
					hdrbuf->format = PIXFORMAT_FLOAT;
					xQueueSendToBack(image_data_queue, (void*)&hdrbuf, portMAX_DELAY); // send image
					xQueueSendToBack(target_data_queue, (target*)&UART, pdMS_TO_TICKS(1000));
					xQueueSendToBack(format_data_queue, (imageformat*)&sendFLOAT, pdMS_TO_TICKS(1000));
					break;
				case PIXFORMAT_MASK:
					ESP_LOGI(TAG, "Camera-event: Send AOPIC to MASK demosaicing.");
					xQueueSendToBack(image_data_queue, (void*)&hdrbuf, portMAX_DELAY); // send image
					xQueueSendToBack(target_data_queue, (target*)&UART, pdMS_TO_TICKS(1000));
					xQueueSendToBack(format_data_queue, (imageformat*)&sendMASK, pdMS_TO_TICKS(1000));
					xQueueSendToBack(mask_data_queue, (uint32_t*)&mask, pdMS_TO_TICKS(1000));
					xQueueSendToBack(image_data_queue, (void*)&expbuf, portMAX_DELAY); // send exposure data
					break;
				case PIXFORMAT_REGIONS:
					ESP_LOGI(TAG, "Camera-event: Send AOPIC to REGION demosaicing.");
					xQueueSendToBack(image_data_queue, (void*)&hdrbuf, portMAX_DELAY); // send image
					xQueueSendToBack(target_data_queue, (target*)&UART, pdMS_TO_TICKS(1000));
					xQueueSendToBack(format_data_queue, (imageformat*)&sendREGION, pdMS_TO_TICKS(1000));
					xQueueSendToBack(image_data_queue, (void*)&expbuf, portMAX_DELAY); // send exposure data
					break;
				}

				// reset values
				pic = 1;
			}
		}
	}
}




// MAIN APP
void app_main()
{

	// set up clock to estimate time needed for code execution
	clock_t start, end;
	start = clock();
	double cpu_time_used;

	// free RAM info
	ESP_LOGI(TAG, "Free RAM: %d", esp_get_free_heap_size());

	// LED setting
	gpio_set_level(LED_FLASH_1, OFF);
	gpio_set_level(LED_FLASH_2, OFF);


	// initialize SD card
	//vTaskDelay(100 / portTICK_RATE_MS);
	uint8_t sdmode = 1;
	init_sdcard(sdmode);

	// initialize camera settings
	//vTaskDelay(100 / portTICK_RATE_MS);
	camera_config_t camera_config = camera_settings(PIXFORMAT_RAW,FRAMESIZE_QQVGA);


	ESP_LOGI(TAG, "start-event: load settings file...");

	// SETTINGS
	//------------------------------------------------------------------------
	// 1: DOSIMETER MODE = 1, USB-CAM = 0
	// 2: UART SEND IN CAM MODE: 1 = on, 0 = off
	// 3: MEASUREMENT FREQ IN DOSI MODE: x in s (e.g. 360 for 5 min)
	// 4: BAUDRATE IN CAM MODE: 115200 to 1152000 seems to work fine
	// 5: SERIAL LOG: 1 = on, 0 = off
	// 6: IMAGE SD SAVE MODE:
	//       0 = NONE
	//       1 = RGB (no vignetting correction)
	//       2 = FLOAT (float RGB with vignetting correction)
	//       3 = HDR
	//       4 = AOPIC
	//       5 = LOG MASK
	// 7: SEND 4 BYTES WHEN IMAGE WAS SAVED: 1 = on, 0 = off
	// 8: LED: 0 = off (default), 1 = on
	// 9: RESOLUTION:
	//		 1 = QQVGA = 160  x 120
	//		 2 =  QVGA = 320  x 240
	//       3 =   VGA = 640  x 480
	//       4 =   XGA = 1024 x 768
	// 10: Demosaicing mode:
	//		 1 = bilinear interpolation (default)
	//		 2 = downsampling (only for Resolution 1 and 2)
	// 11: BLE: 0 = off (default), 1 = on - no functionality other than device pairing currently
	//------------------------------------------------------------------------


	uint32_t settings[values] = {1,0,120,115200,0,3,0,0,1,2,0}; // standard dosimeter settings
	FILE *setfile = NULL;
	// read settings file
	setfile = fopen("/sdcard/calibration/settings.dat", "r");
	if (setfile != NULL)
	{
		// load settings
		ESP_LOGI(TAG, "start-event: read settings file.");

		// read file
		size_t err = fread(settings, sizeof(uint32_t), values, setfile);
		if(err!=0){
			ESP_LOGI(TAG, "start-event: settings loading done.");
		}else{
			ESP_LOGE(TAG, "start-event: Could not read settings file, using standard settings.");
		}
	}else{
		ESP_LOGE(TAG, "start-event: Failed to load settings file, using standard settings.");
	}
	fclose(setfile);

	// set up
	// WORK MODE
	if(settings[0]==1){
		ESP_LOGI(TAG, "SETTING 1: DOSIMETER MODE");
		DOSIMETER_MODE = 1;
	}
	else if(settings[0]==0){
		ESP_LOGI(TAG, "SETTING 1: CAM MODE");
		DOSIMETER_MODE = 0;
	}
	// UART SEND MODE
	if(settings[1]==1){
		ESP_LOGI(TAG, "SETTING 2: SEND IMAGE OVER UART");
		UART_SEND = 1;
	}
	else{
		ESP_LOGI(TAG, "SETTING 2: DO NOT SEND IMAGE");
		UART_SEND = 0;
	}
	// MEASUREMENT FREQUENCY IN DOSIMETER MODE
	MEAS_FREQ = settings[2];
	ESP_LOGI(TAG, "SETTING 3: MEASUREMENT FREQUENCY %d SECONDS", settings[2]);
	// BAUD RATE
	ESP_LOGI(TAG, "SETTING 4: BAUD RATE: %d", settings[3]);
	BAUD_RATE = settings[3];
	// LOG
	if(settings[4]==1){
		ESP_LOGI(TAG, "SETTING 5: LOG MODE: ON");
		esp_log_level_set("*", ESP_LOG_DEBUG);
	}else{
		ESP_LOGI(TAG, "SETTING 5: LOG MODE: OFF");
		esp_log_level_set("*", ESP_LOG_NONE);
	}
	// SD SAVE MODE
	switch(settings[5]){
	case 0:
		ESP_LOGI(TAG, "SETTING 6: SAVE NONE");
		SD_SAVE = SDNONE;
		break;
	case 1:
		ESP_LOGI(TAG, "SETTING 6: SAVE RAW");
		SD_SAVE = SDRGB;
		break;
	case 2:
		ESP_LOGI(TAG, "SETTING 6: SAVE FLOAT");
		SD_SAVE = SDFLOAT;
		break;
	case 3:
		ESP_LOGI(TAG, "SETTING 6: SAVE HDR");
		SD_SAVE = SDHDR;
		break;
	case 4:
		ESP_LOGI(TAG, "SETTING 6: SAVE AOPIC");
		SD_SAVE = SDAOPIC;
		break;
	case 5:
		ESP_LOGI(TAG, "SETTING 6: SAVE ONLY MASK LOG");
		SD_SAVE = SDLOG;
		break;
	}
	// Send saved
	//ESP_LOGI(TAG, "send saved byte: %d", settings[6]);
	if(settings[6]==1){
		ESP_LOGI(TAG, "SETTING 7: SEND 4 BYTES AFTER SAVING: ON");
		SEND_SAVED = 1;
	}else{
		ESP_LOGI(TAG, "SETTING 7: SEND 4 BYTES AFTER SAVING: OFF");
		SEND_SAVED = 0;
	}

	// LED FLASH
	if(settings[7]==1){
		ESP_LOGI(TAG, "SETTING 9: LED ENABLED");
		// LED is connected to same GPIO as sdcard, to enable LED the sdcard must be deactivated
		deinit_sdcard();
		vTaskDelay(1000/portTICK_RATE_MS);
		gpio_set_direction(LED_FLASH_1, GPIO_MODE_OUTPUT);
		gpio_set_level(LED_FLASH_1, 1);
	}else{
		ESP_LOGI(TAG, "SETTING 9: LED DISABLED");
	}
	// RESOLUTION MODE
	switch(settings[8])
	{
	case 1: // with DOWNSAMPLING ON
		//RESOLUTION = FRAMESIZE_QQVGA;
		if(settings[9]==1){
			// bilinear interpolation
			camera_config.frame_size = FRAMESIZE_QQVGA;
			IMAGESIZE = FRAMESIZE_QQVGA;
			TARGETSIZE = FRAMESIZE_QQVGA;
			DEMOSAICING = BILINEAR;
		}else if(settings[9]==2){
			// downsampling
			camera_config.frame_size = FRAMESIZE_QVGA;
			IMAGESIZE = FRAMESIZE_QVGA;
			TARGETSIZE = FRAMESIZE_QQVGA;
			DEMOSAICING = DOWNSAMPLING;
		}
		RESO1 = 160;
		RESO2 = 120;
		ESP_LOGI(TAG, "SETTING 9: RESOLUTION 160 x 120");
		break;
	case 2: // with DOWNSAMPLING ON
		if(settings[9]==1){
			// bilinear interpolation
			camera_config.frame_size = FRAMESIZE_QVGA;
			IMAGESIZE = FRAMESIZE_QVGA;
			TARGETSIZE = FRAMESIZE_QVGA;
			DEMOSAICING = BILINEAR;
		}else if(settings[9]==2){
			// downsampling
			camera_config.frame_size = FRAMESIZE_VGA;
			IMAGESIZE = FRAMESIZE_QVGA;
			TARGETSIZE = FRAMESIZE_VGA;
			DEMOSAICING = DOWNSAMPLING;
		}
		RESO1 = 320;
		RESO2 = 240;
		ESP_LOGI(TAG, "SETTING 9: RESOLUTION 320 x 240");
		break;
	case 3: // with BILINEAR INTERPOLATION ON
		camera_config.frame_size = FRAMESIZE_VGA;
		IMAGESIZE = FRAMESIZE_VGA;
		TARGETSIZE = FRAMESIZE_VGA;
		DEMOSAICING = BILINEAR;
		RESO1 = 640;
		RESO2 = 480;
		ESP_LOGI(TAG, "SETTING 9: RESOLUTION 640 x 480");
		break;
	case 4: // with BILINEAR INTERPOLATION ON
		camera_config.frame_size = FRAMESIZE_XGA;
		IMAGESIZE = FRAMESIZE_XGA;
		TARGETSIZE = FRAMESIZE_XGA;
		DEMOSAICING = BILINEAR;
		RESO1 = 1024;
		RESO2 = 768;
		ESP_LOGI(TAG, "SETTING 9: RESOLUTION 1024 x 768");
		break;
	default:
		ESP_LOGE(TAG,"HDR image event - unsupported resolution switching to QQVGA: 160 x 120");
		camera_config.frame_size = FRAMESIZE_QQVGA;
		if(settings[9]==1){
			// bilinear interpolation
			camera_config.frame_size = FRAMESIZE_QQVGA;
			IMAGESIZE = FRAMESIZE_QQVGA;
			TARGETSIZE = FRAMESIZE_QQVGA;
			DEMOSAICING = BILINEAR;
		}else if(settings[9]==2){
			// downsampling
			camera_config.frame_size = FRAMESIZE_QVGA;
			IMAGESIZE = FRAMESIZE_QVGA;
			TARGETSIZE = FRAMESIZE_QQVGA;
			DEMOSAICING = DOWNSAMPLING;
		}
		RESO1 = 160;
		RESO2 = 120;
		break;
	}
	// DEMOSAICING MODE
	if(settings[9]==1){
		ESP_LOGI(TAG, "SETTING 10: DEMOSAICING: BILINEAR INTERPOLATION");
	}else if(settings[9]==2){
		if(settings[8]<3){
			ESP_LOGI(TAG, "SETTING 10: DEMOSAICING: DOWN SAMPLING");
		}else{
			ESP_LOGI(TAG, "SETTING 10: DEMOSAICING: BILINEAR INTERPOLATION - DOWN SAMPLING NOT POSSIBLE FOR CHOSEN RESOLUTION.");
		}
	}
	// BLE MODE
	if(settings[10]==1){
		ESP_LOGI(TAG, "SETTING 11: BLE ENABLED");
		BLE = 1;
	}else{
		ESP_LOGI(TAG, "SETTING 11: BLE DISABLED");
		BLE = 0;
	}


	// disable hold GPIO mode for camera power pin
	gpio_hold_dis(camera_config.pin_pwdn);
	// turn camera on
	gpio_set_level(camera_config.pin_pwdn, OFF);

	// 8BIT RAM available
	size_t free8bitram = heap_caps_get_free_size(MALLOC_CAP_8BIT);
	ESP_LOGI(TAG, "free 8bit RAM: %zu", free8bitram);


	// initialize camera
	esp_err_t ret = init_camera(camera_config);


	// Configure parameters of an UART driver,
	// communication pins and install the driver
	uart_config_t uart_config = {
			.baud_rate = BAUD_RATE,
			.data_bits = UART_DATA_8_BITS,
			.parity = UART_PARITY_DISABLE,
			.stop_bits = UART_STOP_BITS_1,
			.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
	};

	//Install UART driver, and get the queue.
	uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);
	uart_param_config(EX_UART_NUM, &uart_config);

	//Set UART log level
	//Set UART pins (using UART0 default pins i.e. no changes.)
	uart_set_pin(EX_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

	//Set UART pattern detect function.
	//	uart_enable_pattern_det_intr(EX_UART_NUM, '+', PATTERN_CHR_NUM, 9, 0, 0);
	//Reset the pattern queue length to record at most 20 pattern positions.
	uart_pattern_queue_reset(EX_UART_NUM, 20);

	// free RAM info
	ESP_LOGI(TAG, "Free RAM: %d", esp_get_free_heap_size());

	ESP_LOGI(TAG, "MAIN - start tasks...");
	// create queues and semaphores for tasks
	hdr_queue = xQueueCreate(5, sizeof(uint32_t));
	camera_queue = xQueueCreate(10, sizeof(uint32_t));
	image_data_queue = xQueueCreate(3, sizeof(camera_fb_t));
	target_data_queue = xQueueCreate(3, sizeof(target));
	format_data_queue = xQueueCreate(3, sizeof(imageformat));
	mask_data_queue = xQueueCreate(3, sizeof(uint32_t));
	HDR_image_data_queue = xQueueCreate(3, sizeof(imagebuffer));
	HDR_data_queue = xQueueCreate(10, sizeof(uint32_t));
	xSDMutex = xSemaphoreCreateMutex();

	ESP_LOGI(TAG, "MAIN - start UART task.");
	//Create a task to handle UART event from ISR - base event
	if (DOSIMETER_MODE!=1){
		xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, 10, NULL);
	}
	// create higher order tasks

	ESP_LOGI(TAG, "MAIN - start camera tasks.");
	xTaskCreatePinnedToCore(camera_event_task, "camera_event_task", 2048, NULL, 13, NULL,0);
	xTaskCreatePinnedToCore(HDR_image_event_task, "HDR_image_event_task", 4096, NULL, 15, NULL,0);
	xTaskCreatePinnedToCore(processing_task, "send_image_event_task", 4096, NULL, 14, NULL,0);


	vTaskDelay(1000 / portTICK_RATE_MS);


	// RESET OR DEEP SLEEP WAKE-UP?
	if(settings[0]==1){
		esp_sleep_source_t wu = esp_sleep_get_wakeup_cause();
		if(wu == ESP_SLEEP_WAKEUP_TIMER){
			ESP_LOGI(TAG, "Wakeup-reason: Deep sleep wake-up.");
		}
		else{
			ESP_LOGI(TAG, "Wakeup-reason: Manual restart or battery change.");

			// read log file in binary mode
			ESP_LOGI(TAG, "Mark 'reset' with newline in log.");
			FILE *file;
			file = fopen("/sdcard/data/log", "a");
			if(file == NULL)
			{
				ESP_LOGE(TAG, "Could not open log to write...");
			}else{
				fprintf(file, "reset\n");
				fclose(file);
				file = NULL;
			}
		}
	}

	// free RAM info
	ESP_LOGI(TAG, "Free RAM: %d", esp_get_free_heap_size());

	// DOSIMETER MODE CODE
	if(DOSIMETER_MODE==1){
		// wait for tasks to load
		vTaskDelay(1000 / portTICK_RATE_MS);
		uint32_t send_request = 1;
		uint32_t send_mode_3 = 3;
		uint32_t logcounter = 1;

		// request AOPIC Picture
		long maxpicnumber = numpics;
		static uint32_t send_mode_RGB = 0;
		static uint32_t send_mode_FLOAT = 1;
		static uint32_t send_mode_HDR = 2;
		static uint32_t send_mode_AOPIC = 3;
		static uint32_t send_mode_nRGB = 4;
		static uint32_t send_mode_nFLOAT = 5;
		static uint32_t send_mode_MASK = 6;
		static uint32_t send_mode_REGIONS = 7;

		if(settings[5]==PIXFORMAT_REGIONS){

			// HDR picture loop
			for (uint32_t n=1; n<=numpics; n++){
				uint32_t ind = n-1;
				uint32_t t = dtime[ind];
				ESP_LOGI(TAG, "HDR-event: request image %d", n);
				xQueueSendToBack(camera_queue, &send_request, pdMS_TO_TICKS(1000));
				ESP_LOGI(TAG, "UART-event - send exposure time: %u", t);
				xQueueSendToBack(camera_queue, &t, portMAX_DELAY);		// exposure time
				ESP_LOGI(TAG, "UART-event - send image format %d", (uint32_t)PIXFORMAT_REGIONS);
				xQueueSendToBack(camera_queue, &send_mode_REGIONS, portMAX_DELAY);
				ESP_LOGI(TAG, "UART-event - send pic number %d", n);
				xQueueSendToBack(camera_queue, &n, portMAX_DELAY);
				ESP_LOGI(TAG, "UART-event: send number of images to HDR task %ld", maxpicnumber);
				xQueueSendToBack(camera_queue, (uint32_t*)&maxpicnumber, pdMS_TO_TICKS(1000));
				vTaskDelay(image_delay/1/portTICK_RATE_MS);
				if(SD_SAVE == SDALL || SD_SAVE == SDRGB){
					//vTaskDelay(image_delay/2/portTICK_RATE_MS);
				}
			}
			vTaskDelay(4000 / portTICK_RATE_MS);

		}else{

			// HDR picture loop
			for (uint32_t n=1; n<=numpics; n++){
				uint32_t ind = n-1;
				uint32_t t = dtime[ind];
				ESP_LOGI(TAG, "HDR-event: request image %d", n);
				xQueueSendToBack(camera_queue, &send_request, pdMS_TO_TICKS(1000));
				ESP_LOGI(TAG, "UART-event - send exposure time: %u", t);
				xQueueSendToBack(camera_queue, &t, portMAX_DELAY);		// exposure time
				ESP_LOGI(TAG, "UART-event - send image format %d", (uint32_t)PIXFORMAT_AOPIC);
				// SD SAVE MODE
				switch(settings[5]){
				case 3:
					xQueueSendToBack(camera_queue, &send_mode_HDR, portMAX_DELAY);
					break;
				case 4:
					xQueueSendToBack(camera_queue, &send_mode_AOPIC, portMAX_DELAY);
					break;
				case 5:
					xQueueSendToBack(camera_queue, &send_mode_REGIONS, portMAX_DELAY);
					break;
				default:
					break;
				}
				ESP_LOGI(TAG, "UART-event - send pic number %d", n);
				xQueueSendToBack(camera_queue, &n, portMAX_DELAY);
				ESP_LOGI(TAG, "UART-event: send number of images to HDR task %ld", maxpicnumber);
				xQueueSendToBack(camera_queue, (uint32_t*)&maxpicnumber, pdMS_TO_TICKS(1000));
				vTaskDelay(image_delay/1/portTICK_RATE_MS);
				if(SD_SAVE == SDALL || SD_SAVE == SDRGB){
					//vTaskDelay(image_delay/2/portTICK_RATE_MS);
				}
			}
			vTaskDelay(15000 / portTICK_RATE_MS);

		}


		// dismounting hardware
		ESP_LOGI(TAG, "Dismounting SD-Card and camera.");
		deinit_sdcard();
		deinit_camera();

		vTaskDelay(500 / portTICK_RATE_MS);

		// shut down camera
		gpio_set_level(camera_config.pin_pwdn, 1);
		gpio_hold_en(camera_config.pin_pwdn);
		gpio_deep_sleep_hold_en();

		// start BLE
		if(BLE==1){
			ESP_LOGI(TAG, "Start BLE...");
			aometer_setup_ble_server();

			vTaskDelay(2000 / portTICK_RATE_MS);

			uint32_t len = 6*MASKS;
			float BLE_values[len];
			for(uint32_t n=0; n<len; n++){
				BLE_values[n] = VALUES[n];
			}

			//ble_tx_data();
			vTaskDelay(10000 / portTICK_RATE_MS);
		}


		ESP_LOGI(TAG, "Starting deep sleep.");
		fflush(stdout);
		float offset_time = -0.84; // -0.84 @ 80Hz
		// stop clock time
		end = clock();
		cpu_time_used = ((double) (end - start)) / CLOCKS_PER_SEC;
		ESP_LOGI(TAG, "CPU time used: %f", cpu_time_used-offset_time);
		ESP_LOGI(TAG, "Measurement frequency: %f.", (float)MEAS_FREQ);
		ESP_LOGI(TAG, "Sleep timer: %f.", (float)MEAS_FREQ-cpu_time_used+offset_time);

		// go to sleep
		esp_sleep_enable_timer_wakeup(((float)((MEAS_FREQ-cpu_time_used+offset_time)*1000000)));
		esp_deep_sleep_start();

		// code after the deep sleep section will not be executed...

	}

	// start BLE
	if(BLE==1){
		ESP_LOGI(TAG, "Start BLE...");
		aometer_setup_ble_server();
	}

}


