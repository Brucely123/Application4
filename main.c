/* --------------------------------------------------------------
   Application: 04 - Rev1
   Release Type: Use of Memory Based Task Communication
   Class: Real Time Systems - Su 2025
   Author: [M Borowczak] 
   Email: [mike.borowczak@ucf.edu]
   Company: [University of Central Florida]
   Website: theDRACOlab.com
   AI Use: Commented inline where AI was used
---------------------------------------------------------------*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_log.h"

#define LED_GREEN GPIO_NUM_5
#define LED_RED   GPIO_NUM_4
#define BUTTON_PIN GPIO_NUM_18
#define POT_ADC_CHANNEL ADC1_CHANNEL_6 // GPIO34

#define MAX_COUNT_SEM 10 
#define SENSOR_THRESHOLD 3000
#define DEBOUNCE_DELAY_MS 50

SemaphoreHandle_t sem_button;
SemaphoreHandle_t sem_sensor;
SemaphoreHandle_t print_mutex;

volatile int SEMCNT = 0;

void heartbeat_task(void *pvParameters) {
    while (1) {
        gpio_set_level(LED_GREEN, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(LED_GREEN, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void sensor_task(void *pvParameters) {
    static bool threshold_exceeded = false;
    while (1) {
        int val = adc1_get_raw(POT_ADC_CHANNEL);

        xSemaphoreTake(print_mutex, portMAX_DELAY);
        printf("\U0001F4E1 Radiation Level Read: %d\n", val);
        xSemaphoreGive(print_mutex);

        if (val > SENSOR_THRESHOLD && !threshold_exceeded) {
            if(SEMCNT < MAX_COUNT_SEM+1) SEMCNT++;
            threshold_exceeded = true;
            xSemaphoreGive(sem_sensor);
        } else if (val <= SENSOR_THRESHOLD) {
            threshold_exceeded = false;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void button_task(void *pvParameters) {
    static TickType_t last_press_time = 0;
    while (1) {
        int state = gpio_get_level(BUTTON_PIN);
        TickType_t now = xTaskGetTickCount();
        if (state == 0 && (now - last_press_time) * portTICK_PERIOD_MS > DEBOUNCE_DELAY_MS) {
            last_press_time = now;
            xSemaphoreGive(sem_button);
            xSemaphoreTake(print_mutex, portMAX_DELAY);
            printf("\U0001F6F8 Ground Control Button Press Detected\n");
            xSemaphoreGive(print_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void event_handler_task(void *pvParameters) {
    while (1) {
        if (xSemaphoreTake(sem_sensor, 0)) {
            SEMCNT--;
            xSemaphoreTake(print_mutex, portMAX_DELAY);
            printf("\u26A0\uFE0F ALERT: Radiation Threshold Exceeded!\n");
            xSemaphoreGive(print_mutex);
            gpio_set_level(LED_RED, 1);

            // [BONUS - INDUCED FAILURE]:
            // Simulate a long delay in the event handler that causes starvation
            // of lower-priority tasks like the heartbeat. This mimics a blocking
            // operation such as a lengthy I/O process or computation.
            vTaskDelay(pdMS_TO_TICKS(4000));  // STARVATION INDUCED HERE

            gpio_set_level(LED_RED, 0);
        }

        if (xSemaphoreTake(sem_button, 0)) {
            xSemaphoreTake(print_mutex, portMAX_DELAY);
            printf("\U0001F504 System Mode Toggled by Ground Control\n");
            xSemaphoreGive(print_mutex);
            gpio_set_level(LED_RED, 1);
            vTaskDelay(pdMS_TO_TICKS(300));
            gpio_set_level(LED_RED, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GREEN) | (1ULL << LED_RED),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_conf);

    gpio_config_t btn_conf = {
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE
    };
    gpio_config(&btn_conf);

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(POT_ADC_CHANNEL, ADC_ATTEN_DB_11);

    sem_button = xSemaphoreCreateBinary();
    sem_sensor = xSemaphoreCreateCounting(MAX_COUNT_SEM, 0);
    print_mutex = xSemaphoreCreateMutex();

    xTaskCreate(heartbeat_task, "heartbeat", 2048, NULL, 1, NULL);
    xTaskCreate(sensor_task, "sensor", 2048, NULL, 2, NULL);
    xTaskCreate(button_task, "button", 2048, NULL, 3, NULL);
    xTaskCreate(event_handler_task, "event_handler", 2048, NULL, 2, NULL);
}
