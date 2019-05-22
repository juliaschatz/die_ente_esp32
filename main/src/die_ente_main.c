#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "driver/i2c.h"
#include <driver/adc.h>
#include "network.h"

#include "motor_control.h"
#include "flight_controller.h"

void test_motor(void *arg) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0,ADC_ATTEN_DB_11);
    Motor* motor = createMotor(23);
    printf("Setting neutral\n");
    setMotorSpeed(motor, 0);
    vTaskDelay(1000);
    while (1) {
        int reading = adc1_get_raw(ADC1_CHANNEL_0);
        printf("%d\n",reading);
        setMotorSpeed(motor, 2 * reading / 4095.0 - 1);
        vTaskDelay(10);
    }
}

void app_main()
{
    //xTaskCreate(vTaskServer, "server", 4096, NULL, 5, NULL);
    //xTaskCreate(test_motor, "test_motor", 4096, NULL, 5, NULL);
    xTaskCreate(flightControllerTask, "flight_controller", 4096, NULL, 5, NULL);
}