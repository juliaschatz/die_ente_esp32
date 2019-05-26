#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "task_util.h"

#include "driver/adc.h"
#include "battery.h"

void vBatteryTask(void * arg) {
    TickType_t timeLast = xTaskGetTickCount();

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0,ADC_ATTEN_DB_11);

    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_SEL_2;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    while (1) {
        int adc_val = adc1_get_raw(ADC1_CHANNEL_0);
        float voltage = 4 * 3.3 * adc_val / 4095;
        if (voltage > 3.3 && voltage < 10.5) {
            static bool ledOn = false;
            gpio_set_level(GPIO_NUM_2, ledOn = !ledOn);
        }
        else {
            gpio_set_level(GPIO_NUM_2, 0);
        }

        delayUntil(&timeLast, 500);
    }
}