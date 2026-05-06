#include "template_wifi.h"

#include <string.h>

#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"

static const char *TAG = "template_wifi";

esp_err_t template_wifi_init(void)
{
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "netif init failed");

    esp_err_t ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_RETURN_ON_ERROR(ret, TAG, "event loop create failed");
    }

    esp_netif_t *sta = esp_netif_create_default_wifi_sta();
    ESP_RETURN_ON_FALSE(sta != NULL, ESP_FAIL, TAG, "create sta netif failed");

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), TAG, "wifi init failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "set sta mode failed");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "wifi start failed");

#if CONFIG_TEMPLATE_WIFI_AUTO_CONNECT
    ESP_RETURN_ON_FALSE(strlen(CONFIG_TEMPLATE_WIFI_SSID) > 0, ESP_ERR_INVALID_ARG, TAG, "empty ssid");
    wifi_config_t wifi_config = { 0 };
    strlcpy((char *)wifi_config.sta.ssid, CONFIG_TEMPLATE_WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strlcpy((char *)wifi_config.sta.password, CONFIG_TEMPLATE_WIFI_PASSWORD, sizeof(wifi_config.sta.password));
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &wifi_config), TAG, "wifi config failed");
    ESP_RETURN_ON_ERROR(esp_wifi_connect(), TAG, "wifi connect failed");
#else
    ESP_LOGW(TAG, "Wi-Fi STA started without auto connect");
#endif

    return ESP_OK;
}
