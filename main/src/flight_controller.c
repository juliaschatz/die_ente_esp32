#include "flight_controller.h"
#include "motor_control.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "task_util.h"
#include "ahrs.h"

Motor* fr;
Motor* fl;
Motor* rr;
Motor* rl;
Adafruit_BNO055 * ahrs;

void setupMatrices() {
    /*K << 
   0.025503,  0.020031, -0.034232,  0.003534,  0.002783, -0.002457, 
  -0.025503,  0.020031,  0.034232, -0.003534,  0.002783,  0.002457, 
   0.025503, -0.020031,  0.034232,  0.003534, -0.002783,  0.002457, 
  -0.025503, -0.020031, -0.034232, -0.003534, -0.002783, -0.002457;
  A << 
   1.000000, 0.000000, 0.000000,  0.005000, 0.000000, 0.000000, 
  0.000000,  1.000000, 0.000000, 0.000000,  0.005000, 0.000000, 
  0.000000, 0.000000,  1.000000, 0.000000, 0.000000,  0.005000, 
  0.000000, 0.000000, 0.000000,  1.000000, 0.000000, 0.000000, 
  0.000000, 0.000000, 0.000000, 0.000000,  1.000000, 0.000000, 
  0.000000, 0.000000, 0.000000, 0.000000, 0.000000,  1.000000;
  B << 
   0.172625, -0.172625,  0.172625, -0.172625, 
   0.205230,  0.205230, -0.205230, -0.205230, 
  -0.258648,  0.258648,  0.258648, -0.258648, 
   69.049803, -69.049803,  69.049803, -69.049803, 
   82.092120,  82.092120, -82.092120, -82.092120, 
  -103.459044,  103.459044,  103.459044, -103.459044;
  L << 
   0.002200, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 
  0.000000,  0.002000, 0.000000, 0.000000, 0.000000, 0.000000, 
  0.000000, 0.000000,  0.001800, 0.000000, 0.000000, 0.000000, 
   1.000000, 0.000000, 0.000000,  0.001600, 0.000000, 0.000000, 
  0.000000,  1.000000, 0.000000, 0.000000,  0.001400, 0.000000, 
  0.000000, 0.000000,  1.000000, 0.000000, 0.000000,  0.001200;

  x_hat << 0,0,0,0,0,0;
  y_hat = x_hat;*/
}

void setupMotors() {
    /*fr = createMotor(21);
    fl = createMotor(22);
    rr = createMotor(23);
    rl = createMotor(24);*/
}

void setupAHRS() {
    ahrs = createBNO055(I2C_NUM_0, BNO055_ADDRESS_A);
    ahrsBegin(ahrs, 23, 22, OPERATION_MODE_NDOF);
}

void flightControllerTask(void* arg) {
    setupMotors();
    setupMatrices();
    setupAHRS();
    TickType_t lastTime = xTaskGetTickCount();
    while (1) {
        // Get state from IMU
        double state[6];
        getVector(ahrs, VECTOR_EULER, state);
        getVector(ahrs, VECTOR_GYROSCOPE, state+3);
        // x is in degree and degree/sec
        printf("%f %f %f %f %f %f\n", state[0], state[1], state[2], state[3], state[4], state[5]);

        // Get reference state from controls

        // Apply control effort


        delayUntil(&lastTime, 20);
    }
}