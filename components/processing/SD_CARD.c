// SD CARD FUNCTION FILE

// THE ORIGINAL SD-CARD EXAMPLE SOFTWARE FROM ESPRESSIF ESP-IDF V4.1.3 WAS CHANGED BY FREDERIC RUDAWSKI


#include "SD_CARD.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#include "processing.h"



esp_err_t init_sdcard(uint8_t mode){

	// init SD card
	esp_err_t ret = ESP_FAIL;
	sdmmc_host_t host = SDMMC_HOST_DEFAULT();

	sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
	// set data line mode to 1, otherwise LED FLASH is active while writing to SD card
	slot_config.width = mode;
	esp_vfs_fat_sdmmc_mount_config_t mount_config = {
			.format_if_mount_failed = false,
			.max_files = 8,
	};
	sdmmc_card_t *card;

	ESP_LOGI(TAG, "Mounting SD card...");
	ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);

	if (ret == ESP_OK)
	{
		ESP_LOGI(TAG, "SD card mount successfully!");
		sdmmc_card_print_info(stdout, card);
		return ESP_OK;
	}
	else
	{
		ESP_LOGE(TAG, "Failed to mount SD card VFAT file system. Error: %s", esp_err_to_name(ret));
		return ESP_FAIL;
	}
}


esp_err_t deinit_sdcard(){
	// initialize return value
	esp_err_t ret = ESP_FAIL;
	// log info task: dismounting SD card
	ESP_LOGI(TAG, "Dismounting SD card...");
	// dismounting SD card
	ret =  esp_vfs_fat_sdmmc_unmount();
	// log info if dismountig was succesfull
	if (ret == ESP_OK)
	{
		ESP_LOGI(TAG, "SD card dismount successfully!");
		return ESP_OK;
	}
	else
	{
		ESP_LOGE(TAG, "Failed to dismount SD card VFAT file system. Error: %s", esp_err_to_name(ret));
		return ESP_FAIL;
	}
}
