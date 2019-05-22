#include "task_util.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


long ms_to_tick(int ms) {
    return ms / portTICK_PERIOD_MS;
}

void delay(int ms) {
    vTaskDelay(ms_to_tick(ms));
}

void delayUntil(TickType_t * prevTime, int ms) {
    vTaskDelayUntil(prevTime, ms_to_tick(ms));
}