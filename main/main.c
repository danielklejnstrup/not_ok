#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"

// Servo configuration
#define SERVO_PIN 18
#define SERVO_PWM_FREQ     50    // 50 Hz frequency
#define SERVO_PWM_RES      LEDC_TIMER_16_BIT  // 16-bit resolution
#define SERVO_MIN_PULSEWIDTH  500  // Minimum pulse width in microseconds
#define SERVO_MAX_PULSEWIDTH  2500 // Maximum pulse width in microseconds

// Function Prototypes
void configure_servo();
void setServoAngle(uint32_t angle);
void toggle_charging();
void wifi_init_sta();

static const char *TAG = "main";

// Function to set the servo angle
void setServoAngle(uint32_t angle) {
    // Convert angle to pulse width (microseconds)
    uint32_t pulseWidth = (angle * (SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) / 180 + SERVO_MIN_PULSEWIDTH);
    uint32_t duty = (pulseWidth * (1 << SERVO_PWM_RES) / 20000); // 20000us = 20ms period
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
}

// Function to toggle charging by moving the servo
void toggle_charging() {
    ESP_LOGI(TAG, "Moving in keyfob...");
    setServoAngle(180);
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "Moving away keyfob...");
    setServoAngle(0);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
}

// Function to configure the LEDC PWM timer and channel
void configure_servo() {
    // Local variable for configuration
    ledc_timer_config_t servo_timer = {
        .speed_mode       = LEDC_HIGH_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = SERVO_PWM_RES,
        .freq_hz          = SERVO_PWM_FREQ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&servo_timer);

    // Local variable for channel configuration
    ledc_channel_config_t servo_channel = {
        .channel    = LEDC_CHANNEL_0,
        .duty       = 0,
        .gpio_num   = SERVO_PIN,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0
    };
    ledc_channel_config(&servo_channel);
}

// WiFi event handler
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "retry to connect to the AP");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        // Extract IP address components
        uint32_t ip_addr = event->ip_info.ip.addr;
        uint8_t octet[4];
        octet[0] = (ip_addr >> 0) & 0xFF;
        octet[1] = (ip_addr >> 8) & 0xFF;
        octet[2] = (ip_addr >> 16) & 0xFF;
        octet[3] = (ip_addr >> 24) & 0xFF;
        
        // Print IP address
        ESP_LOGI(TAG, "got ip: %d.%d.%d.%d", octet[0], octet[1], octet[2], octet[3]);
    
    }
}

// Function to initialize WiFi
void wifi_init_sta() {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    esp_wifi_start();
}

void app_main() {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize WiFi
    wifi_init_sta();

    // Configure the servo
    configure_servo();

    // Toggle charging
    while (1) {
        toggle_charging();
    }
}
