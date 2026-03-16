#include "pins.h"
#include "peter_sdcard.h"
#include <sd_protocol_types.h>
#include <driver/spi_common.h>
#include <driver/sdspi_host.h>
#include <esp_vfs_fat.h>
#include "esp_timer.h"

static const char *TAG = "SD";
int bCardMounted = false;
void sdcard_write(float dist)
{
    if (bCardMounted)
    {
        int64_t now_us = esp_timer_get_time();

        ESP_LOGI(TAG, "Write");
        FILE *f = fopen("/sdcard/log.txt", "a");
        if (f == NULL)
        {
            ESP_LOGE(TAG, "Failed to open file");
            return;
        }

        fprintf(f, "%020lld %05.2f \n", now_us, dist);
        fclose(f);
    }
}
void sdcard_init(void)
{
    gpio_set_pull_mode(SD_CS, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(SD_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(SD_MISO, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(SD_SCK, GPIO_PULLUP_ONLY);

    ESP_LOGI(TAG, "SD card Setup");
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.max_freq_khz = 400; // 400 kHz for init (very safe)
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SD_MOSI,
        .miso_io_num = SD_MISO,
        .sclk_io_num = SD_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    ESP_LOGI(TAG, "Init SPI");
    ESP_ERROR_CHECK(spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CH_AUTO));

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SD_CS;
    slot_config.host_id = host.slot;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
    };
    sdmmc_card_t *card;
    ESP_LOGI(TAG, "Mount Card");

    esp_err_t ret = esp_vfs_fat_sdspi_mount("/sdcard",
                                            &host,
                                            &slot_config,
                                            &mount_config,
                                            &card);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to mount SD card (%s)", esp_err_to_name(ret));
        return;
    }
    bCardMounted = true;
    // sdmmc_card_print_info(stdout, card);
}