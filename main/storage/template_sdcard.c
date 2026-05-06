#include "template_sdcard.h"

#include "bsp/esp-bsp.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "template_sdcard";

esp_err_t template_sdcard_init(void)
{
#if CONFIG_TEMPLATE_ENABLE_SDCARD
    ESP_RETURN_ON_ERROR(bsp_sdcard_mount(), TAG, "sdcard mount failed");
    ESP_LOGW(TAG, "SD card mounted at %s", BSP_SD_MOUNT_POINT);
    return ESP_OK;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t template_sdcard_deinit(void)
{
#if CONFIG_TEMPLATE_ENABLE_SDCARD
    return bsp_sdcard_unmount();
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}
