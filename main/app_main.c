/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "storage.h"
#include "led_common.h"
#include "led_pwm.h"
#include "led_ws2812.h"
#include "sensor_mpu6050.h"
#include "sensor_vibration.h"
#include "touchpad_button.h"
#include "touchpad_sensor.h"
#include "speech_recognition.h"

#define INDICATOR_WING_FRONT_LEFT  GPIO_NUM_13
#define INDICATOR_WING_FRONT_RIGHT GPIO_NUM_45
#define INDICATOR_WING_BACK_LEFT   GPIO_NUM_17
#define INDICATOR_WING_BACK_RIGHT  GPIO_NUM_41

#define INDICATOR_EYE              GPIO_NUM_9

#define LIGHT_WING_FRONT_LEFT      GPIO_NUM_14
#define LIGHT_WING_FRONT_RIGHT     GPIO_NUM_43
#define LIGHT_WING_BACK_LEFT       GPIO_NUM_21
#define LIGHT_WING_BACK_RIGHT      GPIO_NUM_44

#define LIGHT_NUM_WING             6
#define INDICATOR_NUM_EYE          2

#define BUTTON_WING_FRONT_LEFT      GPIO_NUM_7
#define BUTTON_WING_FRONT_RIGHT     GPIO_NUM_1
#define BUTTON_WING_BACK_LEFT       GPIO_NUM_5
#define BUTTON_WING_BACK_RIGHT      GPIO_NUM_2

#define BUTTON_TAIL                 GPIO_NUM_3

#define TOUCH_SENSOR_EYE_LEFT       GPIO_NUM_6
#define TOUCH_SENSOR_EYE_RIGHT      GPIO_NUM_4

#define MPU6050_SDA                 GPIO_NUM_10
#define MPU6050_SCL                 GPIO_NUM_12
#define MPU6050_INT                 GPIO_NUM_11

#define MIC_SD                      GPIO_NUM_40
#define MIC_CLK                     GPIO_NUM_39
#define MIC_WS                      GPIO_NUM_38

typedef enum {
    MODE_INVALID = 0,
    MODE_LIGHT,
    MODE_REMOTE_CONTROLLER,
    MODE_SENSER,
    MODE_MUSIC_RHYTHM,
    MODE_MAX,
} running_mode_t;

typedef enum {
    COMMANDS_ID_LING_ON = 0,
    COMMANDS_ID_LIGTH_OFF,
    COMMANDS_ID_LIGTH_WHITE,
    COMMANDS_ID_LIGTH_RED,
    COMMANDS_ID_LIGTH_GREEN,
    COMMANDS_ID_LIGTH_BLUE,
    COMMANDS_ID_LIGTH_BLINK,
    COMMANDS_ID_CONDITIONER_ON,
    COMMANDS_ID_CONDITIONER_OFF,
    COMMANDS_ID_MODE_LIGHT,
    COMMANDS_ID_MODE_REMOTE,
    COMMANDS_ID_MODE_SENSER,
    COMMANDS_ID_MODE_MUSIC_RHYTHM,
    COMMANDS_ID_MAX,
} commands_id_t;

typedef struct {
    commands_id_t id;
    const char *name;
    const char *voice[4];
} commands_t;

// https://zhongwenzhuanpinyin.bmcx.com/
static const commands_t g_commands[] = {
    {COMMANDS_ID_LING_ON, "light_on", {"da kai dian deng", "kai deng"}},
    {COMMANDS_ID_LIGTH_OFF, "light_off", {"guan bi dian deng", "guan deng"}},
    {COMMANDS_ID_LIGTH_WHITE, "light_white", {"ba deng tiao cheng bai se", "bai deng", "bai se"}},
    {COMMANDS_ID_LIGTH_RED, "light_red", {"ba deng tiao cheng hong se", "hong deng", "hong se", "yuan liang ta"}},
    {COMMANDS_ID_LIGTH_GREEN, "light_green", {"ba deng tiao cheng lv se", "lv deng", "lv se", "ai de yan se"}},
    {COMMANDS_ID_LIGTH_BLUE, "light_blue", {"ba deng tiao cheng lan se", "lan deng", "lan se", "yi yu de yan se"}},
    {COMMANDS_ID_LIGTH_BLINK, "light_blink", {"di er hun ji die shen zhi guang", "die shen zhi guang", "di er hun ji"}},
    {COMMANDS_ID_CONDITIONER_ON, "conditioner_on", {"da kai kong tiao", "kai kong tiao"}},
    {COMMANDS_ID_CONDITIONER_OFF, "conditioner_off", {"guan bi kong tiao", "guan kong tiao"}},
    {COMMANDS_ID_MODE_LIGHT, "mode_light", {"deng guang mo shi", "ye deng mo shi"}},
    {COMMANDS_ID_MODE_REMOTE, "mode_remote", {"yao kong mo shi", "yuan cheng kong zhi mo shi"}},
    {COMMANDS_ID_MODE_SENSER, "mode_senser", {"chuan gan qi mo shi", "jian ce mo shi"}},
    {COMMANDS_ID_MODE_MUSIC_RHYTHM, "mode_music_rhythm", {"yin yue lv dong mo shi", "yin yue lv dong"}},
};

static running_mode_t g_running_mode = MODE_LIGHT;
static bool g_voice_wakenet_flag     = false;
static const char *TAG = "butterfly";

struct {
    uint16_t hue;
    uint8_t saturation;
    uint8_t value;
    bool state;
} g_light_wing = {
    .hue = 0,
    .saturation = 0,
    .value = 0,
    .state = false,
};


esp_err_t light_wing_color(uint16_t hue, uint8_t saturation, uint8_t value)
{
    hue = hue <= 360 ? hue :g_light_wing.hue;
    saturation = saturation <= 100 ? saturation :g_light_wing.saturation;
    value = value <= 100 ? value :g_light_wing.value;
    
    uint8_t red, green, blue;
    extern esp_err_t led_hsv2rgb(uint16_t hue, uint8_t saturation, uint8_t value, uint8_t *red, uint8_t *green, uint8_t *blue);
    led_hsv2rgb(hue, saturation, value, &red, &green, &blue);

    // ESP_LOGW(TAG, "hue: %d, saturation: %d, value: %d, red: %d, green: %d, blue: %d", hue, saturation, value, red, green, blue);

    gpio_num_t wing_gpio[] = {LIGHT_WING_FRONT_LEFT, LIGHT_WING_FRONT_RIGHT, LIGHT_WING_BACK_LEFT, LIGHT_WING_BACK_RIGHT};
    for (int i = 0; i < sizeof(wing_gpio) / sizeof(wing_gpio[0]); i++) {
        led_ws2812_set_rgb(wing_gpio[i], LIGHT_NUM_WING, red, green, blue, 500);
    }

    if (red != 0 && green != 0 && blue != 0) {
        g_light_wing.hue = hue;
        g_light_wing.saturation = saturation;
        g_light_wing.value = value;
        g_light_wing.state = true;
    } else {
        g_light_wing.state = false;
    }

    return ESP_OK;
}

void light_wing_switch(bool on)
{
    if (on) {
        if (g_light_wing.value < 5) {
            g_light_wing.value = 100;
        }

        light_wing_color(g_light_wing.hue, g_light_wing.saturation, g_light_wing.value);
    } else {
        light_wing_color(0, 0, 0);

    }
}

static void eye_switch(bool on)
{
    if (on) {
        switch (g_running_mode) {
            case MODE_LIGHT:
                led_ws2812_set_rgb(INDICATOR_EYE, INDICATOR_NUM_EYE, 16, 16, 16, 250);
                break;
            case MODE_REMOTE_CONTROLLER:
                led_ws2812_set_rgb(INDICATOR_EYE, INDICATOR_NUM_EYE, 16, 0, 0, 250);
                break;
            case MODE_SENSER:
                led_ws2812_set_rgb(INDICATOR_EYE, INDICATOR_NUM_EYE, 0, 16, 0, 250);
                break;
            case MODE_MUSIC_RHYTHM:
                led_ws2812_blink_start(INDICATOR_EYE, INDICATOR_NUM_EYE, 255, 16, 200, 1000);
                break;
            default:
                break;
        }
    } else {
        led_ws2812_set_rgb(INDICATOR_EYE, INDICATOR_NUM_EYE, 0, 0, 0, 250);
    }
}

static void eye_blink_start()
{
    switch (g_running_mode) {
        case MODE_LIGHT:
            led_ws2812_blink_start(INDICATOR_EYE, INDICATOR_NUM_EYE, 16, 16, 16, 1000);
            break;
        case MODE_REMOTE_CONTROLLER:
            led_ws2812_blink_start(INDICATOR_EYE, INDICATOR_NUM_EYE, 16, 0, 0, 1000);
            break;
        case MODE_SENSER:
            led_ws2812_blink_start(INDICATOR_EYE, INDICATOR_NUM_EYE, 0, 16, 0, 1000);
            break;
        case MODE_MUSIC_RHYTHM:
            led_ws2812_blink_start(INDICATOR_EYE, INDICATOR_NUM_EYE, 0, 16, 255, 1000); 
            break;
        default:
            break;
    }
}

static void eye_blink_stop()
{
    led_ws2812_blink_stop(INDICATOR_EYE);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    eye_switch(true);
}

double calculate_rms(const int *samples, size_t num) 
{
    double sum = 0.0;

    for (size_t i = 0; i < num; i++) {
        sum += (abs(samples[i]) / num);
    }
    return sum;
}


// The light strip follows the rhythm of the music hue,saturation,value
void music_rhythm(int *audio_data, size_t audio_len)
{
    double mean = calculate_rms(audio_data, audio_len);

    double sum = 0.0;
    for (size_t i = 0; i < audio_len; i++) {
        sum += (audio_data[i] - mean) * (audio_data[i] - mean);
    }
    double std = sqrt(sum / audio_len);

    static double s_mean = 0;
    static double s_std = 0;
    static uint32_t s_mean_size = 0;

    if (!s_mean && !s_std) {
        s_mean = mean;
        s_std = std;
        return;
    }
   
    mean = (mean - s_mean) / s_mean;
    std = (std - s_std) / s_std;

    if (s_mean_size < 100) {
        s_mean_size++;
        return;
    }

    if (mean < 0) {
        return;
    }

    s_mean_size++;

    // ESP_LOGW(TAG, "mean: %.3f, std: %.3f, s_mean_size: %d", mean, std, s_mean_size);


    uint16_t hue = 50;
    uint8_t saturation = std * 20;
    uint8_t value = mean * mean * mean * 10;
    hue = s_mean_size / 3 % 360;
    saturation = saturation > 100 ? 100 : saturation;
    value = value > 100 ? 100 : value;

    // ESP_LOGW(TAG, "hue: %d, saturation: %d, value: %d", hue, saturation, value);

    gpio_num_t wing_gpio_pwm[] = {INDICATOR_WING_FRONT_LEFT, INDICATOR_WING_FRONT_RIGHT, INDICATOR_WING_BACK_LEFT, INDICATOR_WING_BACK_RIGHT};
    for (int i = 0; i < sizeof(wing_gpio_pwm) / sizeof(wing_gpio_pwm[0]); i++) {
        led_pwm_set(wing_gpio_pwm[i], value * 2, 0);
    }

    gpio_num_t wing_gpio[] = {LIGHT_WING_FRONT_LEFT, LIGHT_WING_FRONT_RIGHT, LIGHT_WING_BACK_LEFT, LIGHT_WING_BACK_RIGHT};

    uint8_t red, green, blue;
    led_hsv2rgb(hue, saturation, value, &red, &green, &blue);

    for (int i = 0; i < 4; i++) {
        led_ws2812_set_rgb(wing_gpio[i], LIGHT_NUM_WING, red, green, blue, 0);
    }
}

static void voice_control_task(void *arg)
{
    speech_mic_config_t config = SPEECH_MIC_CONFIG_DEFAULT();
    speech_mic_init(&config);
    speech_recognition_init();

    esp_mn_commands_clear();

    for (int i = 0; i < COMMANDS_ID_MAX; i++) {
        for (int j = 0; j < sizeof(g_commands[i].voice) / sizeof(g_commands[i].voice[0]); j++) {
            if (g_commands[i].voice[j] == NULL) {
                break;
            }
            esp_mn_commands_add(g_commands[i].id, (char *)g_commands[i].voice[j]);
        }
    }

    esp_mn_commands_update();

    while (1) {
        int *audio_data;
        size_t audio_len;

        bool wakenet_flag = false;

        esp_err_t ret = speech_mic_record(&audio_data, &audio_len);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "ear mic record failed, ret: %s", esp_err_to_name(ret));
            break;
        }

        if (g_running_mode == MODE_MUSIC_RHYTHM) {
            music_rhythm(audio_data, audio_len / 4);
        }

        for (int x = 0; x < audio_len / 16; x++) {
            int s1 = ((audio_data[x * 4] + audio_data[x * 4 + 1]) >> 13) & 0x0000FFFF;
            int s2 = ((audio_data[x * 4 + 2] + audio_data[x * 4 + 3]) << 3) & 0xFFFF0000;
            audio_data[x] = s1 | s2;
        }

        ret = speech_recognition_wakenet((const int16_t *)audio_data, audio_len, &wakenet_flag);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "ear recognition wakenet failed, ret: %s", esp_err_to_name(ret));
            break;
        }

        if (!g_voice_wakenet_flag && !wakenet_flag) {
            continue;
        }

        eye_blink_start();

        while (true) {
            ret = speech_mic_record(&audio_data, &audio_len);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "ear mic record failed, ret: %s", esp_err_to_name(ret));
                break;
            }

            for (int x = 0; x < audio_len / 16; x++) {
                int s1 = ((audio_data[x * 4] + audio_data[x * 4 + 1]) >> 13) & 0x0000FFFF;
                int s2 = ((audio_data[x * 4 + 2] + audio_data[x * 4 + 3]) << 3) & 0xFFFF0000;
                audio_data[x] = s1 | s2;
            }

            commands_id_t commands_id;
            ret = speech_recognition_command((const int16_t *)audio_data, audio_len, (int *)&commands_id);

            if (ret == ESP_ERR_TIMEOUT) {
                break;
            } else if (ret == ESP_ERR_NOT_FINISHED) {
                continue;
            }

            ESP_LOGI(TAG, "ear recognition command: %s", g_commands[commands_id].name);

            running_mode_t running_mode = MODE_INVALID;

            switch (commands_id) {
                case COMMANDS_ID_LING_ON:
                    light_wing_switch(true);
                    break;
                case COMMANDS_ID_LIGTH_OFF:
                    light_wing_switch(false);
                    break;
                case COMMANDS_ID_LIGTH_WHITE:
                    light_wing_color(0, 0, 100);
                    break;
                case COMMANDS_ID_LIGTH_RED:
                    light_wing_color(0, 100, 100);
                    break;
                case COMMANDS_ID_LIGTH_GREEN:
                    light_wing_color(120, 50, 100);
                    break;
                case COMMANDS_ID_LIGTH_BLUE:
                    light_wing_color(240, 100, 50);
                    break;

                case COMMANDS_ID_LIGTH_BLINK: {
                        gpio_num_t wing_gpio[] = {LIGHT_WING_FRONT_LEFT, LIGHT_WING_FRONT_RIGHT, LIGHT_WING_BACK_LEFT, LIGHT_WING_BACK_RIGHT};
                        for (int j = 0; j < 5; j++) {
                            for (int i = 0; i < 4; i++) {
                                led_ws2812_set_rgb(wing_gpio[i], LIGHT_NUM_WING, 255, 255, 255, 0);
                            }
                            vTaskDelay(250 / portTICK_PERIOD_MS);
                            
                            for (int i = 0; i < 4; i++) {
                                led_ws2812_set_rgb(wing_gpio[i], LIGHT_NUM_WING, 0, 0, 0, 0);
                            }

                            vTaskDelay(250 / portTICK_PERIOD_MS);
                        }
                    }
                    break;

                case COMMANDS_ID_MODE_LIGHT:
                    running_mode = MODE_LIGHT;
                    break;
                case COMMANDS_ID_MODE_REMOTE:
                    running_mode = MODE_REMOTE_CONTROLLER;
                    break;
                case COMMANDS_ID_MODE_SENSER:
                    running_mode = MODE_SENSER;
                    break;

                case COMMANDS_ID_MODE_MUSIC_RHYTHM:
                    running_mode = MODE_MUSIC_RHYTHM;
                    break;

                case COMMANDS_ID_CONDITIONER_OFF:
                case COMMANDS_ID_CONDITIONER_ON:{
                        gpio_num_t indicator_gpio[] = {INDICATOR_WING_FRONT_LEFT, INDICATOR_WING_FRONT_RIGHT, INDICATOR_WING_BACK_LEFT, INDICATOR_WING_BACK_RIGHT};
                        for (int i = 0; i < 4; i++) {
                            led_pwm_blink_start(indicator_gpio[i], 100, 500);
                        }

                        vTaskDelay(2000 / portTICK_PERIOD_MS);

                        for (int i = 0; i < 4; i++) {
                            led_pwm_blink_stop(indicator_gpio[i]);
                            led_pwm_set(indicator_gpio[i], 0, 0);
                        }
                    }
                    break;

                // case COMMANDS_ID_CONDITIONER_OFF:
                //     break;
                
                default:
                    break;
            }

            if (running_mode != MODE_INVALID) {
                g_running_mode = running_mode;
                 ESP_LOGW(TAG, "running_mode: %s", g_commands[commands_id].name);
                eye_blink_start();
                storage_set("running_mode", &g_running_mode, sizeof(running_mode_t));
            }

        }
        eye_blink_stop();
    }

    vTaskDelete(NULL);
}

static void touchpad_button_cb(gpio_num_t gpio, touchpad_button_event_t event)
{
    uint32_t indecators[] = {INDICATOR_WING_FRONT_LEFT, INDICATOR_WING_FRONT_RIGHT, INDICATOR_WING_BACK_LEFT, INDICATOR_WING_BACK_RIGHT};
    uint32_t buttons[] = {BUTTON_WING_FRONT_LEFT, BUTTON_WING_FRONT_RIGHT, BUTTON_WING_BACK_LEFT, BUTTON_WING_BACK_RIGHT, BUTTON_TAIL};

    switch (event) {
        case TOUCHPAD_BUTTON_EVT_ON_PRESS:
            ESP_LOGI(TAG, "Touch Button[%d] Press", gpio);

            if (g_running_mode == MODE_REMOTE_CONTROLLER) {
                for (int i = 0; i < sizeof(buttons) / sizeof(buttons[0]); i++) {
                    if (buttons[i] == gpio) {
                        if (gpio == BUTTON_TAIL) {
                            for (int j = 0; j < sizeof(indecators) / sizeof(indecators[0]); j++) {
                                led_pwm_set(indecators[j], 255, 500);
                            }
                        } else {
                            led_pwm_set(indecators[i], 255, 500);
                        }
                    }
                }
            }

            g_voice_wakenet_flag = true;
            ESP_LOGW(TAG, "g_voice_wakenet_flag: %d", g_voice_wakenet_flag);
            break;

        case TOUCHPAD_BUTTON_EVT_ON_RELEASE:
            ESP_LOGI(TAG, "Touch Button[%d] Release", gpio);

            if (g_running_mode == MODE_REMOTE_CONTROLLER) {
                for (int i = 0; i < sizeof(buttons) / sizeof(buttons[0]); i++) {
                    if (buttons[i] == gpio) {
                        if (gpio == BUTTON_TAIL) {
                            for (int j = 0; j < sizeof(indecators) / sizeof(indecators[0]); j++) {
                                led_pwm_set(indecators[j], 0, 0);
                            }
                        } else {
                            led_pwm_set(indecators[i], 0, 0);
                        }
                    }
                }
            }

            g_voice_wakenet_flag = false;
            break;

        case TOUCHPAD_BUTTON_EVT_ON_LONGPRESS:
            ESP_LOGI(TAG, "Touch Button[%d] LongPress", gpio);
            break;
        default:
            break;
    }
}

float avg(const float *a, size_t len)
{
    if (!len) {
        return 0.0;
    }

    float sum = 0;

    for (int i = 0; i < len; i++) {
        sum += a[i];
    }

    return sum / len;
}
float std(const float *a, size_t len)
{
    float avg_data = avg(a, len);
    float pow_sum  = 0;

    for (int i = 0; i < len; i++) {
        pow_sum += powf(a[i] - avg_data, 2);
    }

    return sqrtf(pow_sum / len);
}
static void touchpad_sensor_read_task(void *arg)
{
    gpio_num_t touchpad_sensor_gpio[] ={TOUCH_SENSOR_EYE_LEFT, TOUCH_SENSOR_EYE_RIGHT};
    uint32_t value_baseline[] = {0, 0};

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    for (int i = 0; i < sizeof(touchpad_sensor_gpio) / sizeof(touchpad_sensor_gpio[0]); i++) {
        ESP_ERROR_CHECK(touchpad_sensor_read(touchpad_sensor_gpio[i], &value_baseline[i]));
    }

    while (1) {
        uint32_t value[] = {0, 0};
        for (int i = 0; i < sizeof(touchpad_sensor_gpio) / sizeof(touchpad_sensor_gpio[0]); i++) {
            ESP_ERROR_CHECK(touchpad_sensor_read(touchpad_sensor_gpio[i], &value[i]));

            if (g_running_mode == MODE_LIGHT) {
                static uint32_t s_value_last[] = {0, 0};
                if (value[i] < s_value_last[i] || s_value_last[i] == 0) {
                    s_value_last[i] = value[i];
                    continue;
                }

                int change = (value[i] - s_value_last[i]) * 100 / s_value_last[i];

                if (change > 200) {
                    ESP_LOGI(TAG, "Touch Sensor, pin: %d, baseline: %d, value: %d, change: %d%%", touchpad_sensor_gpio[i],value_baseline[i], value[i], change);
                
                    if (i == 0) {
                        g_light_wing.value = g_light_wing.value + 10 > 100 ? 100 : g_light_wing.value + 10;
                        light_wing_color(g_light_wing.hue, g_light_wing.saturation, g_light_wing.value);
                    } else {
                        g_light_wing.value = g_light_wing.value - 10 < 0 ? 0 : g_light_wing.value - 10;
                        light_wing_color(g_light_wing.hue, g_light_wing.saturation, g_light_wing.value);
                    }

                    ESP_LOGW(TAG, "Touch Sensor, value: %d", g_light_wing.value);
                }

                s_value_last[i] = value[i];
            } else if (g_running_mode == MODE_SENSER) {
                if (i != 0) {
                    continue;
                }

                static uint32_t s_value_last[] = {0, 0};
                if (value[i] < s_value_last[i] || s_value_last[i] == 0) {
                    s_value_last[i] = value[i];
                    continue;
                }

                if (value[i] <  value_baseline[i]) {
                    continue;
                }

                int change = (value[i] - s_value_last[i]) * 100 / s_value_last[i];

                change = (value[i] - value_baseline[i]) * 100 / value_baseline[i];
                ESP_LOGW(TAG, "Touch Sensor, pin: %d, baseline: %d, value: %d, change: %d%%",
                         touchpad_sensor_gpio[i],value_baseline[i], value[i], change);

                s_value_last[i] = value[i];

                uint8_t led_value = change / 2 > 255 ? 255 : change / 2;
                led_pwm_set(INDICATOR_WING_FRONT_LEFT, led_value, 0);
                led_pwm_set(INDICATOR_WING_FRONT_RIGHT, led_value, 0);
                led_pwm_set(INDICATOR_WING_BACK_LEFT, led_value, 0);
                led_pwm_set(INDICATOR_WING_BACK_RIGHT, led_value, 0);
            }
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

static float mean(float *buffer, uint32_t size)
{
    float sum = 0;
    for (int i = 0; i < size; i++) {
        sum += buffer[i];
    }

    return sum / size;
}

#define MUP6050_BUFFER_SIZE 10
static void sensor_mpu6050_read_task(void *arg)
{
    float gyro_total_buffer[MUP6050_BUFFER_SIZE] = {0};
    float accer_total_buffer[MUP6050_BUFFER_SIZE] = {0};
    uint32_t buffer_count = 0;

    bool indicator_wing_blink_flag = false;
    uint32_t indicator_wing_blink_timestmap = 0;

    bool light_wing_flag = false;
    uint32_t light_wing_timestmap = 0;

    while (1) {
        vTaskDelay(100 / portTICK_PERIOD_MS);

        mpu6050_acce_value_t acce;
        mpu6050_gyro_value_t gyro;
        ESP_ERROR_CHECK(sensor_mpu6050_read(&acce, &gyro));

        float gyro_total = sqrt(gyro.gyro_x * gyro.gyro_x + gyro.gyro_y * gyro.gyro_y + gyro.gyro_z * gyro.gyro_z);
        float accer_total = sqrt(acce.acce_x * acce.acce_x + acce.acce_y * acce.acce_y + acce.acce_z * acce.acce_z);
        float accer_total_mean = mean(accer_total_buffer, MUP6050_BUFFER_SIZE);
        float gyro_total_mean = mean(gyro_total_buffer, MUP6050_BUFFER_SIZE);

        if (g_running_mode == MODE_LIGHT) {
            if (buffer_count < MUP6050_BUFFER_SIZE 
                || (gyro_total < gyro_total_mean * 1.5 && accer_total < accer_total_mean * 1.5)) {
                uint8_t buffer_index = buffer_count % MUP6050_BUFFER_SIZE;
                gyro_total_buffer[buffer_index] = gyro_total;
                accer_total_buffer[buffer_index] = accer_total;
                buffer_count++;

                if (indicator_wing_blink_flag && esp_log_timestamp() - indicator_wing_blink_timestmap > 3000) {
                    led_pwm_blink_stop(INDICATOR_WING_FRONT_LEFT);
                    led_pwm_set(INDICATOR_WING_FRONT_LEFT, 0, 0);
                    led_pwm_blink_stop(INDICATOR_WING_FRONT_RIGHT);
                    led_pwm_set(INDICATOR_WING_FRONT_RIGHT, 0, 0);
                    led_pwm_blink_stop(INDICATOR_WING_BACK_LEFT);
                    led_pwm_set(INDICATOR_WING_BACK_LEFT, 0, 0);
                    led_pwm_blink_stop(INDICATOR_WING_BACK_RIGHT);
                    led_pwm_set(INDICATOR_WING_BACK_RIGHT, 0, 0);
                    indicator_wing_blink_flag = false;
                }

                if (light_wing_flag && esp_log_timestamp() - light_wing_timestmap > 5000) {
                    light_wing_flag = false;
                    light_wing_switch(false);
                }

                continue;
            }

            if (gyro_total > gyro_total_mean * 20) {
                light_wing_flag = true;
                light_wing_timestmap = esp_log_timestamp();
                light_wing_switch(true);
            // } else if (!light_wing_flag && gyro_total > gyro_total_mean * 10) {
            } else if (!light_wing_flag && gyro_total > gyro_total_mean * 1.5) {
                ESP_LOGW(TAG, "gyro_total: %f, mean: %f", gyro_total, gyro_total_mean);
                if (esp_log_timestamp() - light_wing_timestmap > 500) {
                    light_wing_switch(!g_light_wing.state);
                }
                light_wing_timestmap = esp_log_timestamp();

                g_voice_wakenet_flag = true;
            }

            if (accer_total > accer_total_mean * 1.5) {
                ESP_LOGW(TAG, "accer_total: %f, mean: %f", accer_total, accer_total_mean);

                if (!indicator_wing_blink_flag) {
                    led_pwm_blink_start(INDICATOR_WING_FRONT_LEFT, 50, 500);
                    led_pwm_blink_start(INDICATOR_WING_FRONT_RIGHT, 50, 500);;
                    led_pwm_blink_start(INDICATOR_WING_BACK_LEFT, 50, 500);
                    led_pwm_blink_start(INDICATOR_WING_BACK_RIGHT, 50, 500);
                    indicator_wing_blink_flag = true;
                    indicator_wing_blink_timestmap = esp_log_timestamp();
                }
            }
        }
    }
}

void app_main()
{
    /**
     * @brief Construct a new storage init object
     */
    storage_init();
    if (storage_get("running_mode", &g_running_mode, sizeof(running_mode_t)) != ESP_OK) {
        g_running_mode = MODE_LIGHT;
    }

    /**
     * @brief l
     */
    led_ws2812_config_t config = LED_WS2812_CONFIG_DEFAULT();
    led_ws2812_init(&config);
    gpio_num_t led_ws2812_gpio[] ={LIGHT_WING_FRONT_LEFT, LIGHT_WING_FRONT_RIGHT, LIGHT_WING_BACK_LEFT, LIGHT_WING_BACK_RIGHT, INDICATOR_EYE};
    uint8_t led_ws2812_num[] = {LIGHT_NUM_WING, LIGHT_NUM_WING, LIGHT_NUM_WING, LIGHT_NUM_WING, INDICATOR_NUM_EYE};

    for (int i = 0; i < sizeof(led_ws2812_gpio) / sizeof(led_ws2812_gpio[0]); i++) {
        led_ws2812_create(led_ws2812_gpio[i], led_ws2812_num[i]);
        led_ws2812_set_rgb(led_ws2812_gpio[i], led_ws2812_num[i], 0, 0, 0, 0);
    }

    led_pwm_config_t led_config = LED_PWM_CONFIG_DEFAULT();
    led_pwm_init(&led_config);

    gpio_num_t led_pwm_gpio[] = {INDICATOR_WING_FRONT_LEFT, INDICATOR_WING_FRONT_RIGHT, INDICATOR_WING_BACK_LEFT, INDICATOR_WING_BACK_RIGHT};
    for (int i = 0; i < sizeof(led_pwm_gpio) / sizeof(led_pwm_gpio[0]); i++) {
        led_pwm_create(led_pwm_gpio[i]);
    }

    touchpad_button_config_t button_config = TOUCHPAD_BUTTON_CONFIG_DEFAULT();
    touchpad_button_init(&button_config);
    gpio_num_t touchpad_button_gpio[] ={BUTTON_WING_FRONT_LEFT, BUTTON_WING_FRONT_RIGHT, BUTTON_WING_BACK_LEFT, BUTTON_WING_BACK_RIGHT, BUTTON_TAIL};

    for (int i = 0; i < sizeof(touchpad_button_gpio) / sizeof(touchpad_button_gpio[0]); i++) {
        ESP_LOGI(TAG, "Touch Button[%d] Create", touchpad_button_gpio[i]);
        touchpad_button_create(touchpad_button_gpio[i], 0.05, touchpad_button_cb);
    }

    touchpad_sensor_config_t touch_config = TOUCHPAD_SENSOR_CONFIG_DEFAULT();
    touchpad_sensor_init(&touch_config);
    gpio_num_t touchpad_sensor_gpio[] ={TOUCH_SENSOR_EYE_LEFT, TOUCH_SENSOR_EYE_RIGHT};
    for (int i = 0; i < sizeof(touchpad_sensor_gpio) / sizeof(touchpad_sensor_gpio[0]); i++) {
        touchpad_sensor_create(touchpad_sensor_gpio[i]);
    }
    xTaskCreate(touchpad_sensor_read_task, "touchpad_sensor_read_task", 1024 * 4, NULL, 5, NULL);

    sensor_mpu6050_config_t mpu6050_config = SENSOR_MPU6050_CONFIG_DEFAULT();
    mpu6050_config.i2c_sda = MPU6050_SDA;
    mpu6050_config.i2c_scl = MPU6050_SCL;
    sensor_mpu6050_init(&mpu6050_config);
    xTaskCreate(sensor_mpu6050_read_task, "sensor_mpu6050_read_task", 1024 * 4, NULL, 5, NULL);
    xTaskCreate(voice_control_task, "voice_control_task", 1024 * 4, NULL, 15, NULL);

    eye_blink_start();

    extern void connect_rainmaker(int running_mode);
    connect_rainmaker(g_running_mode);
}
