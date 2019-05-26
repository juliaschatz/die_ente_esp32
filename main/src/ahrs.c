/*!
 * @file Adafruit_BNO055.cpp
 *
 *  @mainpage Adafruit BNO055 Orientation Sensor
 *
 *  @section intro_sec Introduction
 *
 *  This is a library for the BNO055 orientation sensor
 *
 *  Designed specifically to work with the Adafruit BNO055 Breakout.
 *
 *  Pick one up today in the adafruit shop!
 *  ------> https://www.adafruit.com/product/2472
 *
 *  These sensors use I2C to communicate, 2 pins are required to interface.
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  @section author Author
 *
 *  K.Townsend (Adafruit Industries)
 *
 *  @section license License
 *
 *  MIT license, all text above must be included in any redistribution
 */


#include <limits.h>
#include <math.h>
#include <string.h>

#include "task_util.h"
#include "ahrs.h"
#include "driver/uart.h"

/*!
 *  @brief  Instantiates a new Adafruit_BNO055 class
 *  @param  sensorID
 *          sensor ID
 *  @param  address
 *          i2c address
 *  @param  *theWire
 *          Wire object
 */
Adafruit_BNO055* createBNO055(uart_port_t uart_port) {
  Adafruit_BNO055* imu = malloc(sizeof(Adafruit_BNO055));
  imu->uart_port = uart_port;
  imu->_mode = OPERATION_MODE_NDOF;
  return imu;
}

/*!
 *  @brief  Sets up the HW
 *  @param  mode
 *          mode values
 *           [OPERATION_MODE_CONFIG,
 *            OPERATION_MODE_ACCONLY,
 *            OPERATION_MODE_MAGONLY,
 *            OPERATION_MODE_GYRONLY,
 *            OPERATION_MODE_ACCMAG,
 *            OPERATION_MODE_ACCGYRO,
 *            OPERATION_MODE_MAGGYRO,
 *            OPERATION_MODE_AMG,
 *            OPERATION_MODE_IMUPLUS,
 *            OPERATION_MODE_COMPASS,
 *            OPERATION_MODE_M4G,
 *            OPERATION_MODE_NDOF_FMC_OFF,
 *            OPERATION_MODE_NDOF]
 *  @return true if process is successful
 */
int ahrsBegin(Adafruit_BNO055* imu, gpio_num_t tx_num, gpio_num_t rx_num, adafruit_bno055_opmode_t mode) {
  uart_config_t uart_config = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .rx_flow_ctrl_thresh = 122,
  };
  ESP_ERROR_CHECK(uart_param_config(imu->uart_port, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(imu->uart_port, tx_num, rx_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  // Setup UART buffered IO with event queue
  const int uart_buffer_size = (1024 * 2);
  // Install UART driver using an event queue here
  ESP_ERROR_CHECK(uart_driver_install(imu->uart_port, uart_buffer_size, \
                                          uart_buffer_size, 10, &imu->uart_queue, 0));

  /* Make sure we have the right device */
  uint8_t id = read8(imu, BNO055_CHIP_ID_ADDR);
  printf("Read AHRS id %d\n", id);

  /* Switch to config mode (just in case since this is the default) */
  setMode(imu, OPERATION_MODE_CONFIG);

  /* Reset */
  write8(imu, BNO055_SYS_TRIGGER_ADDR, 0x20);
  while (read8(imu, BNO055_CHIP_ID_ADDR) != BNO055_ID) {
    delay(10);
  }
  delay(50);

  /* Set to normal power mode */
  write8(imu, BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
  delay(10);

  write8(imu, BNO055_PAGE_ID_ADDR, 0);

  /* Set the output units */
  /*
  uint8_t unitsel = (0 << 7) | // Orientation = Android
                    (0 << 4) | // Temperature = Celsius
                    (0 << 2) | // Euler = Degrees
                    (1 << 1) | // Gyro = Rads
                    (0 << 0);  // Accelerometer = m/s^2
  write8(BNO055_UNIT_SEL_ADDR, unitsel);
  */

  /* Configure axis mapping (see section 3.4) */
  /*
  write8(BNO055_AXIS_MAP_CONFIG_ADDR, REMAP_CONFIG_P2); // P0-P7, Default is P1
  delay(10);
  write8(BNO055_AXIS_MAP_SIGN_ADDR, REMAP_SIGN_P2); // P0-P7, Default is P1
  delay(10);
  */

  write8(imu, BNO055_SYS_TRIGGER_ADDR, 0x0);
  delay(10);
  /* Set the requested operating mode (see section 3.3) */
  setMode(imu, mode);
  delay(20);

  return 1;
}

/*!
 *  @brief  Puts the chip in the specified operating mode
 *  @param  mode
 *          mode values
 *           [OPERATION_MODE_CONFIG,
 *            OPERATION_MODE_ACCONLY,
 *            OPERATION_MODE_MAGONLY,
 *            OPERATION_MODE_GYRONLY,
 *            OPERATION_MODE_ACCMAG,
 *            OPERATION_MODE_ACCGYRO,
 *            OPERATION_MODE_MAGGYRO,
 *            OPERATION_MODE_AMG,
 *            OPERATION_MODE_IMUPLUS,
 *            OPERATION_MODE_COMPASS,
 *            OPERATION_MODE_M4G,
 *            OPERATION_MODE_NDOF_FMC_OFF,
 *            OPERATION_MODE_NDOF]
 */
void setMode(Adafruit_BNO055 * imu, adafruit_bno055_opmode_t mode) {
  imu->_mode = mode;
  write8(imu, BNO055_OPR_MODE_ADDR, imu->_mode);
  delay(30);
}

/*!
 *  @brief  Changes the chip's axis remap
 *  @param  remapcode
 *          remap code possible values
 *          [REMAP_CONFIG_P0
 *           REMAP_CONFIG_P1 (default)
 *           REMAP_CONFIG_P2
 *           REMAP_CONFIG_P3
 *           REMAP_CONFIG_P4
 *           REMAP_CONFIG_P5
 *           REMAP_CONFIG_P6
 *           REMAP_CONFIG_P7]
 */
void setAxisRemap(Adafruit_BNO055 * imu,
    adafruit_bno055_axis_remap_config_t remapcode) {
  adafruit_bno055_opmode_t modeback = imu->_mode;

  setMode(imu, OPERATION_MODE_CONFIG);
  delay(25);
  write8(imu, BNO055_AXIS_MAP_CONFIG_ADDR, remapcode);
  delay(10);
  /* Set the requested operating mode (see section 3.3) */
  setMode(imu, modeback);
  delay(20);
}

/*!
 *  @brief  Changes the chip's axis signs
 *  @param  remapsign
 *          remap sign possible values
 *          [REMAP_SIGN_P0
 *           REMAP_SIGN_P1 (default)
 *           REMAP_SIGN_P2
 *           REMAP_SIGN_P3
 *           REMAP_SIGN_P4
 *           REMAP_SIGN_P5
 *           REMAP_SIGN_P6
 *           REMAP_SIGN_P7]
 */
void setAxisSign(Adafruit_BNO055 * imu, adafruit_bno055_axis_remap_sign_t remapsign) {
  adafruit_bno055_opmode_t modeback = imu->_mode;

  setMode(imu, OPERATION_MODE_CONFIG);
  delay(25);
  write8(imu, BNO055_AXIS_MAP_SIGN_ADDR, remapsign);
  delay(10);
  /* Set the requested operating mode (see section 3.3) */
  setMode(imu, modeback);
  delay(20);
}

/*!
 *  @brief  Use the external 32.768KHz crystal
 *  @param  usextal
 *          use external crystal boolean
 */
void setExtCrystalUse(Adafruit_BNO055 * imu, int usextal) {
  adafruit_bno055_opmode_t modeback = imu->_mode;

  /* Switch to config mode (just in case since this is the default) */
  setMode(imu, OPERATION_MODE_CONFIG);
  delay(25);
  write8(imu, BNO055_PAGE_ID_ADDR, 0);
  if (usextal) {
    write8(imu, BNO055_SYS_TRIGGER_ADDR, 0x80);
  } else {
    write8(imu, BNO055_SYS_TRIGGER_ADDR, 0x00);
  }
  delay(10);
  /* Set the requested operating mode (see section 3.3) */
  setMode(imu, modeback);
  delay(20);
}

/*!
 *   @brief  Gets the latest system status info
 *   @param  system_status
 *           system status info
 *   @param  self_test_result
 *           self test result
 *   @param  system_error
 *           system error info
 */
void getSystemStatus(Adafruit_BNO055 * imu, uint8_t *system_status,
                                      uint8_t *self_test_result,
                                      uint8_t *system_error) {
  write8(imu, BNO055_PAGE_ID_ADDR, 0);

  /* System Status (see section 4.3.58)
     0 = Idle
     1 = System Error
     2 = Initializing Peripherals
     3 = System Iniitalization
     4 = Executing Self-Test
     5 = Sensor fusio algorithm running
     6 = System running without fusion algorithms
   */

  if (system_status != 0)
    *system_status = read8(imu, BNO055_SYS_STAT_ADDR);

  /* Self Test Results
     1 = test passed, 0 = test failed

     Bit 0 = Accelerometer self test
     Bit 1 = Magnetometer self test
     Bit 2 = Gyroscope self test
     Bit 3 = MCU self test

     0x0F = all good!
   */

  if (self_test_result != 0)
    *self_test_result = read8(imu, BNO055_SELFTEST_RESULT_ADDR);

  /* System Error (see section 4.3.59)
     0 = No error
     1 = Peripheral initialization error
     2 = System initialization error
     3 = Self test result failed
     4 = Register map value out of range
     5 = Register map address out of range
     6 = Register map write error
     7 = BNO low power mode not available for selected operat ion mode
     8 = Accelerometer power mode not available
     9 = Fusion algorithm configuration error
     A = Sensor configuration error
   */

  if (system_error != 0)
    *system_error = read8(imu, BNO055_SYS_ERR_ADDR);

  delay(200);
}

/*!
 *  @brief  Gets the chip revision numbers
 *  @param  info
 *          revision info
 */
void getRevInfo(Adafruit_BNO055 * imu, adafruit_bno055_rev_info_t *info) {
  uint8_t a, b;

  memset(info, 0, sizeof(adafruit_bno055_rev_info_t));

  /* Check the accelerometer revision */
  info->accel_rev = read8(imu, BNO055_ACCEL_REV_ID_ADDR);

  /* Check the magnetometer revision */
  info->mag_rev = read8(imu, BNO055_MAG_REV_ID_ADDR);

  /* Check the gyroscope revision */
  info->gyro_rev = read8(imu, BNO055_GYRO_REV_ID_ADDR);

  /* Check the SW revision */
  info->bl_rev = read8(imu, BNO055_BL_REV_ID_ADDR);

  a = read8(imu, BNO055_SW_REV_ID_LSB_ADDR);
  b = read8(imu, BNO055_SW_REV_ID_MSB_ADDR);
  info->sw_rev = (((uint16_t)b) << 8) | ((uint16_t)a);
}

/*!
 *  @brief  Gets current calibration state.  Each value should be a uint8_t
 *          pointer and it will be set to 0 if not calibrated and 3 if
 *          fully calibrated.
 *          See section 34.3.54
 *  @param  sys
 *          Current system calibration status, depends on status of all sensors,
 * read-only
 *  @param  gyro
 *          Current calibration status of Gyroscope, read-only
 *  @param  accel
 *          Current calibration status of Accelerometer, read-only
 *  @param  mag
 *          Current calibration status of Magnetometer, read-only
 */
void getCalibration(Adafruit_BNO055 * imu, uint8_t *sys, uint8_t *gyro,
                                     uint8_t *accel, uint8_t *mag) {
  uint8_t calData = read8(imu, BNO055_CALIB_STAT_ADDR);
  if (sys != NULL) {
    *sys = (calData >> 6) & 0x03;
  }
  if (gyro != NULL) {
    *gyro = (calData >> 4) & 0x03;
  }
  if (accel != NULL) {
    *accel = (calData >> 2) & 0x03;
  }
  if (mag != NULL) {
    *mag = calData & 0x03;
  }
}

/*!
 *  @brief  Gets the temperature in degrees celsius
 *  @return temperature in degrees celsius
 */
int8_t getTemp(Adafruit_BNO055 * imu) {
  int8_t temp = (int8_t)(read8(imu, BNO055_TEMP_ADDR));
  return temp;
}

/*!
 *  @brief   Gets a vector reading from the specified source
 *  @param   vector_type
 *           possible vector type values
 *           [VECTOR_ACCELEROMETER
 *            VECTOR_MAGNETOMETER
 *            VECTOR_GYROSCOPE
 *            VECTOR_EULER
 *            VECTOR_LINEARACCEL
 *            VECTOR_GRAVITY]
 *  @return  vector from specified source
 */
void getVector(Adafruit_BNO055 * imu, adafruit_vector_type_t vector_type, float* _buffer) {
  float* xyz = _buffer;
  uint8_t buffer[6];
  memset(buffer, 0, 6);

  int16_t x, y, z;
  x = y = z = 0;

  /* Read vector data (6 bytes) */
  readLen(imu, (adafruit_bno055_reg_t)vector_type, buffer, 6);

  x = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
  y = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
  z = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);

  /*!
   * Convert the value to an appropriate range (section 3.6.4)
   * and assign the value to the Vector type
   */
  switch (vector_type) {
  case VECTOR_MAGNETOMETER:
    /* 1uT = 16 LSB */
    xyz[0] = ((float)x) / 16.0;
    xyz[1] = ((float)y) / 16.0;
    xyz[2] = ((float)z) / 16.0;
    break;
  case VECTOR_GYROSCOPE:
    /* 1dps = 16 LSB */
    xyz[0] = ((float)x) / 16.0;
    xyz[1] = ((float)y) / 16.0;
    xyz[2] = ((float)z) / 16.0;
    break;
  case VECTOR_EULER:
    /* 1 degree = 16 LSB */
    xyz[0] = ((float)x) / 16.0;
    xyz[1] = ((float)y) / 16.0;
    xyz[2] = ((float)z) / 16.0;
    break;
  case VECTOR_ACCELEROMETER:
    /* 1m/s^2 = 100 LSB */
    xyz[0] = ((float)x) / 100.0;
    xyz[1] = ((float)y) / 100.0;
    xyz[2] = ((float)z) / 100.0;
    break;
  case VECTOR_LINEARACCEL:
    /* 1m/s^2 = 100 LSB */
    xyz[0] = ((float)x) / 100.0;
    xyz[1] = ((float)y) / 100.0;
    xyz[2] = ((float)z) / 100.0;
    break;
  case VECTOR_GRAVITY:
    /* 1m/s^2 = 100 LSB */
    xyz[0] = ((float)x) / 100.0;
    xyz[1] = ((float)y) / 100.0;
    xyz[2] = ((float)z) / 100.0;
    break;
  }
}

/*!
 *  @brief  Gets a quaternion reading from the specified source
 *  @return quaternion reading
 */
void getQuat(Adafruit_BNO055 * imu, float* _buffer) {
  uint8_t buffer[8];
  memset(buffer, 0, 8);

  int16_t x, y, z, w;
  x = y = z = w = 0;

  /* Read quat data (8 bytes) */
  readLen(imu, BNO055_QUATERNION_DATA_W_LSB_ADDR, buffer, 8);
  w = (((uint16_t)buffer[1]) << 8) | ((uint16_t)buffer[0]);
  x = (((uint16_t)buffer[3]) << 8) | ((uint16_t)buffer[2]);
  y = (((uint16_t)buffer[5]) << 8) | ((uint16_t)buffer[4]);
  z = (((uint16_t)buffer[7]) << 8) | ((uint16_t)buffer[6]);

  /*!
   * Assign to Quaternion
   * See
   * http://ae-bst.resource.bosch.com/media/products/dokumente/bno055/BST_BNO055_DS000_12~1.pdf
   * 3.6.5.5 Orientation (Quaternion)
   */
  const float scale = (1.0 / (1 << 14));
  _buffer[0] = scale * w;
  _buffer[1] = scale * x;
  _buffer[2] = scale * y;
  _buffer[3] = scale * z;
}

/*!
 *  @brief  Reads the sensor's offset registers into a byte array
 *  @param  calibData
 *  @return true if read is successful
 */
int getSensorOffsets(Adafruit_BNO055 * imu, uint8_t *calibData) {
  if (isFullyCalibrated(imu)) {
    adafruit_bno055_opmode_t lastMode = imu->_mode;
    setMode(imu, OPERATION_MODE_CONFIG);

    readLen(imu, ACCEL_OFFSET_X_LSB_ADDR, calibData, NUM_BNO055_OFFSET_REGISTERS);

    setMode(imu, lastMode);
    return 1;
  }
  return 0;
}

/*!
 *  @brief  Writes an array of calibration values to the sensor's offset
 *  @param  *calibData
 *          calibration data
 */
void setSensorOffsets(Adafruit_BNO055 * imu, const uint8_t *calibData) {
  adafruit_bno055_opmode_t lastMode = imu->_mode;
  setMode(imu, OPERATION_MODE_CONFIG);
  delay(25);

  /* Note: Configuration will take place only when user writes to the last
     byte of each config data pair (ex. ACCEL_OFFSET_Z_MSB_ADDR, etc.).
     Therefore the last byte must be written whenever the user wants to
     changes the configuration. */

  /* A writeLen() would make this much cleaner */
  write8(imu, ACCEL_OFFSET_X_LSB_ADDR, calibData[0]);
  write8(imu, ACCEL_OFFSET_X_MSB_ADDR, calibData[1]);
  write8(imu, ACCEL_OFFSET_Y_LSB_ADDR, calibData[2]);
  write8(imu, ACCEL_OFFSET_Y_MSB_ADDR, calibData[3]);
  write8(imu, ACCEL_OFFSET_Z_LSB_ADDR, calibData[4]);
  write8(imu, ACCEL_OFFSET_Z_MSB_ADDR, calibData[5]);

  write8(imu, MAG_OFFSET_X_LSB_ADDR, calibData[6]);
  write8(imu, MAG_OFFSET_X_MSB_ADDR, calibData[7]);
  write8(imu, MAG_OFFSET_Y_LSB_ADDR, calibData[8]);
  write8(imu, MAG_OFFSET_Y_MSB_ADDR, calibData[9]);
  write8(imu, MAG_OFFSET_Z_LSB_ADDR, calibData[10]);
  write8(imu, MAG_OFFSET_Z_MSB_ADDR, calibData[11]);

  write8(imu, GYRO_OFFSET_X_LSB_ADDR, calibData[12]);
  write8(imu, GYRO_OFFSET_X_MSB_ADDR, calibData[13]);
  write8(imu, GYRO_OFFSET_Y_LSB_ADDR, calibData[14]);
  write8(imu, GYRO_OFFSET_Y_MSB_ADDR, calibData[15]);
  write8(imu, GYRO_OFFSET_Z_LSB_ADDR, calibData[16]);
  write8(imu, GYRO_OFFSET_Z_MSB_ADDR, calibData[17]);

  write8(imu, ACCEL_RADIUS_LSB_ADDR, calibData[18]);
  write8(imu, ACCEL_RADIUS_MSB_ADDR, calibData[19]);

  write8(imu, MAG_RADIUS_LSB_ADDR, calibData[20]);
  write8(imu, MAG_RADIUS_MSB_ADDR, calibData[21]);

  setMode(imu, lastMode);
}

/*!
 *  @brief  Checks of all cal status values are set to 3 (fully calibrated)
 *  @return status of calibration
 */
int isFullyCalibrated(Adafruit_BNO055 * imu) {
  uint8_t system, gyro, accel, mag;
  getCalibration(imu, &system, &gyro, &accel, &mag);

  switch (imu->_mode) {
  case OPERATION_MODE_ACCONLY:
    return (accel == 3);
  case OPERATION_MODE_MAGONLY:
    return (mag == 3);
  case OPERATION_MODE_GYRONLY:
  case OPERATION_MODE_M4G: /* No magnetometer calibration required. */
    return (gyro == 3);
  case OPERATION_MODE_ACCMAG:
  case OPERATION_MODE_COMPASS:
    return (accel == 3 && mag == 3);
  case OPERATION_MODE_ACCGYRO:
  case OPERATION_MODE_IMUPLUS:
    return (accel == 3 && gyro == 3);
  case OPERATION_MODE_MAGGYRO:
    return (mag == 3 && gyro == 3);
  default:
    return (system == 3 && gyro == 3 && accel == 3 && mag == 3);
  }
}

/*!
 *  @brief  Enter Suspend mode (i.e., sleep)
 */
void enterSuspendMode(Adafruit_BNO055 * imu) {
  adafruit_bno055_opmode_t modeback = imu->_mode;

  /* Switch to config mode (just in case since this is the default) */
  setMode(imu, OPERATION_MODE_CONFIG);
  delay(25);
  write8(imu, BNO055_PWR_MODE_ADDR, 0x02);
  /* Set the requested operating mode (see section 3.3) */
  setMode(imu, modeback);
  delay(20);
}

/*!
 *  @brief  Enter Normal mode (i.e., wake)
 */
void enterNormalMode(Adafruit_BNO055 * imu) {
  adafruit_bno055_opmode_t modeback = imu->_mode;

  /* Switch to config mode (just in case since this is the default) */
  setMode(imu, OPERATION_MODE_CONFIG);
  delay(25);
  write8(imu, BNO055_PWR_MODE_ADDR, 0x00);
  /* Set the requested operating mode (see section 3.3) */
  setMode(imu, modeback);
  delay(20);
}

/*!
 *  @brief  Writes an 8 bit value over I2C
 */
int write8(Adafruit_BNO055 * imu, adafruit_bno055_reg_t reg, uint8_t value) {
  uint8_t packet[] = {0xAA, 0x00, reg, 1, value};
  uart_flush(imu->uart_port);
  uart_write_bytes(imu->uart_port, (const char*)packet, 5);
  uint8_t buf[130];
  int avail = uart_read_bytes(imu->uart_port, buf, 2, 100);
  if (avail > 0) {
    if (buf[0] == 0xEE) {
      // Do something with response status buf[1]
    }
  }

  /* ToDo: Check for error! */
  return 1;
}

/*!
 *  @brief  Reads an 8 bit value over I2C
 */
uint8_t read8(Adafruit_BNO055 * imu, adafruit_bno055_reg_t reg) {
  uint8_t value = 0;
  readLen(imu, reg, &value, 1);
  return value;
}

/*!
 *  @brief  Reads the specified number of bytes over I2C
 */
int readLen(Adafruit_BNO055 * imu, adafruit_bno055_reg_t reg, uint8_t *buffer,
                              uint8_t len) {
  uint8_t packet[] = {0xAA, 0x01, reg, len};
  uart_flush(imu->uart_port);
  uart_write_bytes(imu->uart_port, (const char*)packet, 4);
  uint8_t available = 0;
  do {
    uart_get_buffered_data_len(imu->uart_port, &available);
    delay(1);
  } while (!available);
  uint8_t buf[130];
  int avail = uart_read_bytes(imu->uart_port, buf, available, 100);
  if (avail > 0) {
    if (buf[0] == 0xBB) {
      uint8_t recvd = buf[1];
      memcpy(buffer, buf+2, len);
      return 1;
    }
    else if (buf[0] == 0xEE) {
    }
  }

  return 0;
}
