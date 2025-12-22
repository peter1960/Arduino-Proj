#include <nvs_flash.h>
#include <esp_err.h>
#include <esp_log.h>
#include "peter_flash.h"

static const char *TAG = "FLASH";

static nvs_handle_t my_handle;

void flash_init()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_LOGI(TAG, "\nOpening Non-Volatile Storage (NVS) handle...");

    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
        return;
    }
}

void flash_store(float total, float trip)
{
    int32_t counter = 42;
    ESP_LOGI(TAG, "\nWriting counter to NVS...");
    esp_err_t err = nvs_set_i32(my_handle, "counter", counter);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to write counter!");
    }
}