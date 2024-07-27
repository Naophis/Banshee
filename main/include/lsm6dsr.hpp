#ifndef LSM6DSR_HPP
#define LSM6DSR_HPP

#include "defines.hpp"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>
#include <string.h>

class LSM6DSR {
public:
  LSM6DSR();
  virtual ~LSM6DSR();

  void init();
  uint8_t write1byte(const uint8_t address, const uint8_t data);
  uint8_t read1byte(const uint8_t address);
  int16_t read2byte(const uint8_t address);
  int16_t read2byte_2(const uint8_t address);
  int16_t IRAM_ATTR read_2byte(const uint8_t address);

  void req_read1byte_itr(const uint8_t address);
  uint8_t read_1byte_itr();

  void req_read2byte_itr(const uint8_t address);
  int16_t read_2byte_itr();
  signed short read_2byte_itr2(std::vector<int> &list);
  void setup();
  int read_gyro_z();
  int read_accel_x();
  int read_accel_y();
  void begin();
  void enable_g();

private:
  spi_device_handle_t spi;
  spi_transaction_t itr_t;
  spi_transaction_t *r_trans;
};

#endif