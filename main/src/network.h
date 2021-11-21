#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <stdlib.h>

#define AP_SSID "ente"
#define AP_PASS "entemeansduck"
#define STA_SSID "Unit2"
#define STA_PASS "spock987"
#define STA_MAX_RETRY 5
#define PORT 4357
#define NET_TAG "NETWORK"

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

#define PACKET_LEN 5
typedef struct {
    int8_t commandYaw;
    int8_t commandPitch;
    int8_t commandRoll;
    int8_t commandThrottle;
    int8_t commandother;
    TickType_t timestamp;
} Packet;


void init_ap();
void init_station();

void vServerTask(void * params);