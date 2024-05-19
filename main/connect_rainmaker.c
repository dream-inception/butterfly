#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <nvs_flash.h>

#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_params.h>
#include <esp_rmaker_standard_devices.h>
#include <esp_rmaker_schedule.h>
#include <esp_rmaker_scenes.h>

#include <esp_rmaker_common_events.h>
#include <esp_rmaker_standard_types.h>

#include <app_wifi.h>

// #include "app_priv.h"


static esp_rmaker_device_t *butterfly_device          = NULL;
static const char *TAG = "connect_rainmaker";


// extern esp_err_t light_wing_set_hsv(uint16_t hue, uint8_t saturation, uint8_t value);
extern esp_err_t light_wing_color(uint16_t hue, uint8_t saturation, uint8_t value);
extern esp_err_t light_wing_switch(bool on);

/* Callback to handle param updates received from the RainMaker cloud */
static esp_err_t write_cb(const esp_rmaker_device_t *device, const esp_rmaker_param_t *param,
            const esp_rmaker_param_val_t val, void *priv_data, esp_rmaker_write_ctx_t *ctx)
{
    const char *device_name = esp_rmaker_device_get_name(device);
    const char *param_name = esp_rmaker_param_get_name(param);
    if (strcasecmp(param_name, ESP_RMAKER_DEF_POWER_NAME) == 0) {
        ESP_LOGI(TAG, "Received value = %s for %s - %s",
                val.val.b? "true" : "false", device_name, param_name);
        light_wing_switch(val.val.b);
    } else if (strcasecmp(param_name, ESP_RMAKER_DEF_BRIGHTNESS_NAME) == 0) {
        ESP_LOGI(TAG, "Received value = %d for %s - %s",
                val.val.i, device_name, param_name);
        light_wing_color(-1, -1, val.val.i);
    } else if (strcasecmp(param_name, ESP_RMAKER_DEF_HUE_NAME) == 0) {
        ESP_LOGI(TAG, "Received value = %d for %s - %s",
                val.val.i, device_name, param_name);
        light_wing_color(val.val.i, -1, -1);
    } else if (strcasecmp(param_name, ESP_RMAKER_DEF_SATURATION_NAME) == 0) {
        ESP_LOGI(TAG, "Received value = %d for %s - %s",
                val.val.i, device_name, param_name);
        light_wing_color(-1, val.val.i, -1);
    } else {
        /* Silently ignoring invalid params */
        ESP_LOGW(TAG, "Unknown param %s for %s", param_name, device_name);
        return ESP_OK;
    }

    esp_rmaker_param_update_and_report(param, val);
    return ESP_OK;
}
#include <esp_rmaker_utils.h>

void connect_rainmaker(int runngin_mode)
{
    esp_err_t err = ESP_OK;
     /* Initialize Wi-Fi. Note that, this should be called before esp_rmaker_node_init() */
    app_wifi_init();

    // esp_rmaker_wifi_reset(2, 2);


    /* Initialize the ESP RainMaker Agent */
    esp_rmaker_config_t rainmaker_cfg = {
        .enable_time_sync = true,
    };

    typedef struct {
        const char *param_name;
        uint8_t properties;
        const char *ui_type;
        esp_rmaker_param_val_t val;
        esp_rmaker_param_val_t min;
        esp_rmaker_param_val_t max;
        esp_rmaker_param_val_t step;
    } radar_param_t;
    esp_rmaker_param_val_t invalid_val = {.type = RMAKER_VAL_TYPE_INVALID};

    const radar_param_t param_list_light[] = {
        {"power", PROP_FLAG_READ | PROP_FLAG_WRITE, ESP_RMAKER_UI_TOGGLE, esp_rmaker_bool(0),invalid_val,invalid_val,invalid_val},
        {"brightness", PROP_FLAG_READ | PROP_FLAG_WRITE, ESP_RMAKER_UI_SLIDER, esp_rmaker_int(0),esp_rmaker_int(0), esp_rmaker_int(100), esp_rmaker_int(1)},
        {"saturation", PROP_FLAG_READ | PROP_FLAG_WRITE, ESP_RMAKER_UI_SLIDER, esp_rmaker_int(0), esp_rmaker_int(0), esp_rmaker_int(100), esp_rmaker_int(1)},
        {"hue", PROP_FLAG_READ | PROP_FLAG_WRITE, ESP_RMAKER_UI_HUE_CIRCLE, esp_rmaker_int(0), esp_rmaker_int(0), esp_rmaker_int(360), esp_rmaker_int(1)},
    };
    const radar_param_t param_list_air_conditioner[] = {
        {"power", PROP_FLAG_READ | PROP_FLAG_WRITE, ESP_RMAKER_UI_TOGGLE, esp_rmaker_bool(0),invalid_val,invalid_val,invalid_val},
        {"mode", PROP_FLAG_READ | PROP_FLAG_WRITE, ESP_RMAKER_UI_DROPDOWN, esp_rmaker_int(1), esp_rmaker_int(0), esp_rmaker_int(5), esp_rmaker_int(1)},
        {"fan_speed", PROP_FLAG_READ | PROP_FLAG_WRITE, ESP_RMAKER_UI_DROPDOWN, esp_rmaker_int(1), esp_rmaker_int(0), esp_rmaker_int(5), esp_rmaker_int(1)},
        {"temperature", PROP_FLAG_READ | PROP_FLAG_WRITE, ESP_RMAKER_UI_SLIDER, esp_rmaker_int(25), esp_rmaker_int(16), esp_rmaker_int(30), esp_rmaker_int(1)},
    };

    esp_rmaker_node_t *node = NULL;
    const radar_param_t *param_list = NULL;

    node = esp_rmaker_node_init(&rainmaker_cfg, "Butterfly", "Butterfly");

    if (runngin_mode == 1) {
        if (!node) {
            ESP_LOGE(TAG, "Could not initialise node. Aborting!!!");
            vTaskDelay(pdMS_TO_TICKS(5000));
            abort();
        }
        /* Create a device and add the relevant parameters to it */
        butterfly_device = esp_rmaker_device_create("butterfly_light", ESP_RMAKER_DEVICE_LIGHTBULB, NULL);
        ESP_ERROR_CHECK(esp_rmaker_device_add_cb(butterfly_device, write_cb, NULL));
        param_list = param_list_light;
    } else {
        /* Create a device and add the relevant parameters to it */
        butterfly_device = esp_rmaker_device_create("butterfly_air_conditioner", ESP_RMAKER_DEVICE_AIR_CONDITIONER, NULL);
        param_list = param_list_air_conditioner;
    }

    for (int i = 0; i < 4; ++i) {
        char radar_param_type[64] = "esp.param.";
        strcat(radar_param_type, param_list[i].param_name);

        ESP_LOGW(TAG, "radar_param_type: %s", radar_param_type);
        esp_rmaker_param_t *param = esp_rmaker_param_create(param_list[i].param_name, radar_param_type,
                                    param_list[i].val, param_list[i].properties);

        if(param_list[i].ui_type) {
            esp_rmaker_param_add_ui_type(param, param_list[i].ui_type);
        }

        if (param_list[i].min.type != RMAKER_VAL_TYPE_INVALID) {
            esp_rmaker_param_add_bounds(param, param_list[i].min, param_list[i].max, param_list[i].step);
        }

        ESP_ERROR_CHECK(esp_rmaker_device_add_param(butterfly_device, param));
    }

    esp_rmaker_param_t *param_powor = esp_rmaker_device_get_param_by_name(butterfly_device, "power");
    esp_rmaker_device_assign_primary_param(butterfly_device, param_powor);

    ESP_ERROR_CHECK(esp_rmaker_node_add_device(node, butterfly_device));

    /* Enable OTA */
    // ESP_ERROR_CHECK(esp_rmaker_ota_enable_default());

    /* Enable timezone service which will be require for setting appropriate timezone
     * from the phone apps for scheduling to work correctly.
     * For more information on the various ways of setting timezone, please check
     * https://rainmaker.espressif.com/docs/time-service.html.
     */
    esp_rmaker_timezone_service_enable();

    /* Enable scheduling. */
    esp_rmaker_schedule_enable();

    /* Enable Scenes */
    esp_rmaker_scenes_enable();

    /* Start the ESP RainMaker Agent */
    esp_rmaker_start();

    /* Start the Wi-Fi./
     * If the node is provisioned, it will start connection attempts,
     * else, it will start Wi-Fi provisioning. The function will return
     * after a connection has been successfully established
     */
    err = app_wifi_start(POP_TYPE_NONE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not start Wifi. Aborting!!!");
        vTaskDelay(pdMS_TO_TICKS(5000));
        abort();
    }
}
