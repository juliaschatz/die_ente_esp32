#include "flight_controller.h"
#include "motor_control.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "task_util.h"
#include "driver/uart.h"
#include "ahrs.h"
#include "linear.h"

Motor* fr;
Motor* fl;
Motor* rr;
Motor* rl;
Adafruit_BNO055 * ahrs;

float K_data[] = {
   0.025503,  0.020031, -0.034232,  0.003534,  0.002783, -0.002457, 
  -0.025503,  0.020031,  0.034232, -0.003534,  0.002783,  0.002457, 
   0.025503, -0.020031,  0.034232,  0.003534, -0.002783,  0.002457, 
  -0.025503, -0.020031, -0.034232, -0.003534, -0.002783, -0.002457
};
float A_data[] = {
  1.000000, 0.000000, 0.000000,  0.005000, 0.000000, 0.000000, 
  0.000000,  1.000000, 0.000000, 0.000000,  0.005000, 0.000000, 
  0.000000, 0.000000,  1.000000, 0.000000, 0.000000,  0.005000, 
  0.000000, 0.000000, 0.000000,  1.000000, 0.000000, 0.000000, 
  0.000000, 0.000000, 0.000000, 0.000000,  1.000000, 0.000000, 
  0.000000, 0.000000, 0.000000, 0.000000, 0.000000,  1.000000
};
float B_data[] = {
   0.172625, -0.172625,  0.172625, -0.172625, 
   0.205230,  0.205230, -0.205230, -0.205230, 
  -0.258648,  0.258648,  0.258648, -0.258648, 
   69.049803, -69.049803,  69.049803, -69.049803, 
   82.092120,  82.092120, -82.092120, -82.092120, 
  -103.459044,  103.459044,  103.459044, -103.459044
};

Matrix K = {.data = K_data, .rows = 4, .cols = 6};
Matrix A = {.data = A_data, .rows = 6, .cols = 6};
Matrix B = {.data = B_data, .rows = 6, .cols = 4};

void setupMotors() {
    /*fr = createMotor(21);
    fl = createMotor(22);
    rr = createMotor(23);
    rl = createMotor(24);*/
}

int setupAHRS() {
    ahrs = createBNO055(UART_NUM_2);
    return ahrsBegin(ahrs, 23, 22, OPERATION_MODE_NDOF);
}

void flightControllerTask(void* arg) {
    setupMotors();
    if (!setupAHRS()) {
        printf("AHRS setup failed\n");
    }
    TickType_t lastTime = xTaskGetTickCount();
    while (1) {
        // Get state from IMU
        double state[6];
        getVector(ahrs, VECTOR_EULER, state);
        getVector(ahrs, VECTOR_GYROSCOPE, state+3);
        // x is in degree and degree/sec
        printf("%f %f %f %f %f %f\n", state[0], state[1], state[2], state[3], state[4], state[5]);
        Vector x = {.data = state, .length = 6};

        // Get reference state from controls
        Vector r = {.data = NULL, .length = 6};

        // Apply control effort
        Vector dr = vectorSub(r, x);
        Vector u = matVecMul(K, dr);

        free(u.data);
        free(dr.data);
        delayUntil(&lastTime, 20);
    }
}