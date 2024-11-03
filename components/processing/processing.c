
#include "processing.h"



// FUNCTIONS

// TYPECAST TRANSFORMATION
// https://stackoverflow.com/questions/25767705/how-can-i-convert-uint8-t-and-uint16-t-into-floats-in-c
float uint8_2_float(uint8_t u8) {
	return u8; // implicit conversion to return type
}
float uint16_2_float(uint16_t u16) {
	return u16; // implicit conversion to return type
}

// ALLOCATE IMAGE BUFFER
esp_err_t init_imbuffer(imagebuffer *buffer, uint32_t size){
	// initialize image buffer
	memset(buffer, 0, sizeof(imagebuffer));
	buffer->len = size;
	ESP_LOGI(TAG, "Allocating %d KB image buffer in PSRAM", size/1024);
	buffer->buf = (uint8_t*) heap_caps_calloc(buffer->len, 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
	if(!buffer->buf) {
		free(buffer);
		ESP_LOGE(TAG, "Allocating %d KB frame buffer Failed", size/1024);
		return ESP_FAIL;
	}
	memset(buffer->buf, 0, buffer->len);
	// return OK
	return ESP_OK;
}

camera_config_t camera_settings(imageformat format, framesize_t resolution){
	// camera sensor settings
	camera_config_t camera_config = {
		.pin_pwdn = 32,
		.pin_reset = CONFIG_RESET,
		.pin_xclk = CONFIG_XCLK,
		.pin_sscb_sda = CONFIG_SDA,
		.pin_sscb_scl = CONFIG_SCL,
		.pin_clk_flags = 0,
		.pin_d7 = CONFIG_D7,
		.pin_d6 = CONFIG_D6,
		.pin_d5 = CONFIG_D5,
		.pin_d4 = CONFIG_D4,
		.pin_d3 = CONFIG_D3,
		.pin_d2 = CONFIG_D2,
		.pin_d1 = CONFIG_D1,
		.pin_d0 = CONFIG_D0,
		.pin_vsync = CONFIG_VSYNC,
		.pin_href = CONFIG_HREF,
		.pin_pclk = CONFIG_PCLK,

		//XCLK 20MHz or 10MHz
		.xclk_freq_hz = CAM_FREQ,//CONFIG_XCLK_FREQ,
		.ledc_timer = LEDC_TIMER_0,
		.ledc_channel = LEDC_CHANNEL_0,

		.pixel_format = format,
		.frame_size = resolution,

		.jpeg_quality = 1,  //0-63 lower number means higher quality, default was 12
		.fb_count = 1       //if more than one, i2s runs in continuous mode. Use only with JPEG
	};
	return camera_config;
}


esp_err_t init_camera(camera_config_t camera_config){
	ESP_LOGI(TAG, "Camera initialization...");
	//initialize the camera
	esp_err_t err = esp_camera_init(&camera_config);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "Camera init Failed");
		return err;
	}
	else
	{
		ESP_LOGI(TAG, "Camera init successful!");
		return ESP_OK;
	}
}

esp_err_t deinit_camera()
{
	ESP_LOGI(TAG, "Dismounting Camera...");
	// de-initialize the camera
	esp_err_t err = esp_camera_deinit();
	// log info if camera de-init was successful
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "Camera dismount Failed");
		return err;
	}
	else
	{
		ESP_LOGI(TAG, "Camera dismounted!");
	}
	return ESP_OK;
}

