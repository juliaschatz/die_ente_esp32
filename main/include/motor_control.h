#pragma once

#define OFF_PULSE_US 1000
#define MIN_PULSE_US 1080
#define MAX_PULSE_US 2000
#define MAX_POWER 0.51 // Safe limit

typedef struct {
    int unit;
    int timer;
    int op;
} Motor;

void initUnit(int unit, int timer);
Motor* createMotor(int gpio_pwm);
void setMotorSpeed(Motor* motor, float speed);