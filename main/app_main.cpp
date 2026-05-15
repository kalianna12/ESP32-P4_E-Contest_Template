/*
 * ESP32-P4 electric contest minimal template.
 */

#include "esp_check.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "bsp/display.h"
#include "bsp_board_extra.h"
#include "esp_lv_adapter.h"
#include "lvgl.h"
#include "lvgl_adapter_init.h"

#include "ui/test_screen.h"
#include "spilink.h"

#include "wifi/template_wifi.h"
#include "storage/template_sdcard.h"

static const char *TAG = "template";

static void spi_link_task(void *arg)
{
    (void)arg;
    while (true) {
        SpiLink_Task();
    }
}

static esp_err_t template_nvs_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "erase nvs failed");
        ret = nvs_flash_init();
    }
    return ret;
}

extern "C" void app_main(void)
{
    ESP_ERROR_CHECK(template_nvs_init());

    ESP_ERROR_CHECK(bsp_extra_codec_init());
    ESP_ERROR_CHECK(bsp_extra_codec_mute_set(true));

    bsp_display_cfg_t cfg = {
        .hw_cfg = {
            .hdmi_resolution = BSP_HDMI_RES_NONE,
            .dsi_bus = {
                .lane_bit_rate_mbps = BSP_LCD_MIPI_DSI_LANE_BITRATE_MBPS,
            },
        },
    };

    ESP_LOGI(TAG,
             "display init start: internal=%u dma=%u psram=%u",
             heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
             heap_caps_get_free_size(MALLOC_CAP_DMA),
             heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    lv_display_t *disp = lvgl_adapter_init(&cfg);
    ESP_LOGI(TAG, "display init result=%p", disp);
    if (disp == NULL) {
        ESP_LOGE(TAG,
                 "Display init failed: internal=%u dma=%u psram=%u",
                 heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                 heap_caps_get_free_size(MALLOC_CAP_DMA),
                 heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
        return;
    }

    ESP_ERROR_CHECK(bsp_display_backlight_on());

    ESP_ERROR_CHECK(esp_lv_adapter_lock(-1));
    test_screen_create();
    esp_lv_adapter_unlock();

    SpiLink_Init();
    xTaskCreatePinnedToCore(spi_link_task, "spi_link_rx", 4096, nullptr, 4, nullptr, 1);

    ESP_ERROR_CHECK(template_wifi_init());

#if CONFIG_TEMPLATE_ENABLE_SDCARD
    ESP_ERROR_CHECK(template_sdcard_init());
#endif

    ESP_LOGW(TAG, "Template initialized");
}
