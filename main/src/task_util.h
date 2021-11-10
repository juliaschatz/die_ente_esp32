#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void delay(int ms);

void delayUntil(TickType_t * prevTime, int ms);