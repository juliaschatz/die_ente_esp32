#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "driver/i2c.h"
#include <driver/adc.h>
#include "network.h"

#include "motor_control.h"
#include "flight_controller.h"
#include "battery.h"


void app_main()
{
    init_ap();
    xTaskCreate(flightControllerTask, "flight_controller", 4096, NULL, 5, NULL);
    xTaskCreate(vServerTask, "server", 4096, NULL, 5, NULL);
    xTaskCreate(vBatteryTask, "battery_watch", 4096, NULL, 5, NULL);
}