#ifndef AS5147P_HPP
#define AS5147P_HPP

#include "defines.hpp"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <bitset>
#include <cstring>
#include <string.h>

class AS5147P {
public:
  AS5147P();
  virtual ~AS5147P();

  void init();
  uint8_t write1byte(const uint8_t address, const uint8_t data);
  uint8_t read1byte(const uint8_t address);
  int16_t read2byte(const uint16_t address);
  int16_t read2byte(const uint8_t address1, const uint8_t address2);
  int32_t read2byte(const uint8_t address1, const uint8_t address2, bool rorl);

  int16_t read2byte_2(const uint8_t address1, const uint8_t address2);

  void req_read1byte_itr(const uint8_t address);
  uint8_t read_1byte_itr();

  void req_read2byte_itr(const uint8_t address);
  int16_t read_2byte_itr();
  signed short read_2byte_itr2(std::vector<int> &list);
  void setup();
  int read_gyro_z();
  int read_accel_x();
  int read_accel_y();

private:
  spi_device_handle_t spi_l;
  spi_device_handle_t spi_r;
  spi_transaction_t itr_t;
  spi_transaction_t *r_trans;
  bool _spiCalcEvenParity(uint16_t value);
};

#endif