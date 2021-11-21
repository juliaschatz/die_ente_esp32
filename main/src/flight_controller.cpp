#include "flight_controller.h"
#include "motor_control.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "task_util.h"
#include "driver/uart.h"
#include "ahrs.h"
#include "network.h"
#include "math.h"
#include "gnc.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <string.h>

using namespace Eigen;

Motor fr(19);
Motor fl(18);
Motor rr(5);
Motor rl(17);
Adafruit_BNO055 ahrs(UART_NUM_2);
extern QueueHandle_t xNetQueue;

const double Kl = 0.9968/4; // Throttle lift constant. Per motor required to maintain altitude
adafruit_bno055_offsets_t ahrs_cal;

void setupMotors() {
  fr.begin();
  fl.begin();
  rl.begin();
  rr.begin();
}

void modifyThrottle(Vector4d& u, double throttle) {
  // Modify the control input so that the minimum is zero
  double min = fmin(u(0), fmin(u(1), fmin(u(2), u(3))));
  if (min < 0) {
    u += -1. * min * Vector4d(1.0, 1.0, 1.0, 1.0);
  }
  // Scale control inputs so that they remain sane relative to the applied throttle
  double max = fmax(u(0), fmax(u(1), fmax(u(2), u(3))));
  u *= throttle / max;
}

void updateState(Vector3d& rates, Quaterniond& attitude) {
  // Get state from IMU
  attitude = ahrs.getQuat();
  rates = ahrs.getVector(Adafruit_BNO055::adafruit_vector_type_t::VECTOR_GYROSCOPE);
}

bool setupAHRS() {
  if (ahrs.begin(Adafruit_BNO055::adafruit_bno055_opmode_t::OPERATION_MODE_NDOF, GPIO_NUM_23, GPIO_NUM_22)) {
    // any addl setup needed here? axis remap?
    return true;
  }
  else {
    return false;
  }
}

void flightControllerTask(void* arg) {
  setupController();
  setupMotors();

  // Arming/calibration sequence
  fr.setSpeed(0.0);
  fl.setSpeed(0.0);
  rl.setSpeed(0.0);
  rr.setSpeed(0.0);
  vTaskDelay(200);
  fr.setSpeed(1.0);
  fl.setSpeed(1.0);
  rl.setSpeed(1.0);
  rr.setSpeed(1.0);
  vTaskDelay(200);
  fr.setSpeed(0.0);
  fl.setSpeed(0.0);
  rl.setSpeed(0.0);
  rr.setSpeed(0.0);

  if (!setupAHRS()) {
    printf("AHRS setup failed\n");
    return;
  }
  TickType_t lastTime = xTaskGetTickCount();

  // AHRS calibration
  // Check if calibration stored in nvs
  nvs_handle_t my_handle;
  esp_err_t nvs_err;

  // Open
  nvs_err = nvs_open(NVS_AHRS_NS, NVS_READWRITE, &my_handle);
  if (nvs_err != ESP_OK) return;
  size_t read_size = 0;  // value will default to 0, if not set yet in NVS
  nvs_err = nvs_get_blob(my_handle, NVS_AHRS_KEY_CAL, NULL, &read_size);
  if (nvs_err != ESP_OK && nvs_err != ESP_ERR_NVS_NOT_FOUND) return;

  // Read previously saved blob if available
  if (read_size > 0) {
    if (read_size != sizeof(adafruit_bno055_offsets_t)) {
      printf("Wrong size calibration\n");
      return;
    }
    nvs_get_blob(my_handle, NVS_AHRS_KEY_CAL, &ahrs_cal, &read_size);
    ahrs.setSensorOffsets(ahrs_cal);
    printf("Loaded calibration\n");
  }
  else {
    uint8_t sysCal;
    uint8_t gyroCal;
    uint8_t accelCal;
    uint8_t magCal;
    while (!ahrs.getSensorOffsets(ahrs_cal)) {
      
      ahrs.getCalibration(&sysCal, &gyroCal, &accelCal, &magCal);

      printf("Sys: %d Gyro: %d Accel: %d Mag: %d\n", sysCal, gyroCal, accelCal, magCal);
      delayUntil(&lastTime, 10);
    }
    // Save to NVS
    nvs_set_blob(my_handle, NVS_AHRS_KEY_CAL, &ahrs_cal, sizeof(adafruit_bno055_offsets_t));
    nvs_commit(my_handle);
    printf("Saved calibration\n");
  }
  nvs_close(my_handle);
  delay(1000); // Wait for calibration to take effect and AHRS to setup

  Packet lastControlState = {.timestamp = lastTime};
  Quaterniond attitude;
  Quaterniond desiredAttitude(0.0,0.0,0.0,0.0);
  Vector3d rates;
  // Init desired attitude to match yaw of current attitude
  updateState(rates, attitude); 
  double yaw = atan2f(attitude.z(), attitude.w());
  desiredAttitude.w() = cosf(yaw);
  desiredAttitude.z() = sinf(yaw);
  while (1) {
    updateState(rates, attitude);

    // Get reference state from controls
    xQueueReceive(xNetQueue, &lastControlState, 0);

    /*desired_state << 0,
                      15.0 * lastControlState.commandPitch / 127.0,
                      15.0 * lastControlState.commandRoll / 127.0,
                      0.,0.,0.;*/

    // Calculate control effort
    Vector3d desired_angvel = attitude_controller(attitude, desiredAttitude);
    Vector4d u = angvel_controller(rates, desired_angvel);

    // Apply throttle
    double throttle = Kl + lastControlState.commandThrottle * 0.25 / 127.0;
    //modifyThrottle(u, throttle);
    
    u += Vector4d(1.0, 1.0, 1.0, 1.0) * throttle;
    uint8_t sysCal;
    uint8_t gyroCal;
    uint8_t accelCal;
    uint8_t magCal;
    ahrs.getCalibration(&sysCal, &gyroCal, &accelCal, &magCal);
    printf("%d %d %d %d : %f %f %f %f : %f %f %f %f : %f %f %f %f : %f %f %f\n", sysCal, gyroCal, accelCal, magCal, u(0), u(1), u(2), u(3), desiredAttitude.w(), desiredAttitude.x(), desiredAttitude.y(), desiredAttitude.z(), attitude.w(), attitude.x(), attitude.y(), attitude.z(), rates(0), rates(1), rates(2));
    if (lastTime - lastControlState.timestamp > CONTROL_TIMEOUT || lastControlState.commandThrottle < 0.1) {
        u *= 0.0;
    }
    fr.setSpeed(u(0));
    fl.setSpeed(u(1));
    rl.setSpeed(u(2));
    rr.setSpeed(u(3));

    delayUntil(&lastTime, 10);
  }
}