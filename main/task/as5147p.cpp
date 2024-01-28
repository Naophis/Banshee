#include "include/as5147p.hpp"

AS5147P::AS5147P() {}
AS5147P::~AS5147P() {}
void AS5147P::init() {
  esp_err_t ret;
  spi_bus_config_t buscfg = {
      .mosi_io_num = ENC_MOSI,
      .miso_io_num = ENC_MISO,
      .sclk_io_num = ENC_CLK,
      .quadwp_io_num = -1,  // unused
      .quadhd_io_num = -1,  // unused
      .max_transfer_sz = 3, // bytes
      .flags = SPICOMMON_BUSFLAG_MASTER,
      .intr_flags = 0 // 割り込みをしない
  };
  spi_device_interface_config_t devcfg = {
      .mode = 3,
      .clock_speed_hz = 10 * 1000 * 1000,
      // .clock_speed_hz = 1 * 1000 * 1000,
      .spics_io_num = ENC_L_CS,
      .queue_size = 1,
  };
  // Initialize the SPI bus
  ret = spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_DISABLED);
  ESP_ERROR_CHECK(ret);
  // Attach the LCD to the SPI bus
  ret = spi_bus_add_device(SPI3_HOST, &devcfg, &spi_l);
  devcfg.spics_io_num = ENC_R_CS;
  ret = spi_bus_add_device(SPI3_HOST, &devcfg, &spi_r);

  // high keep
  GPIO.out_w1ts = BIT(6);
  GPIO.out1_w1ts.val = BIT(33 - 32);

  ESP_ERROR_CHECK(ret);
}
uint8_t AS5147P::write1byte(const uint8_t address, const uint8_t data) {
  return 0;
}
bool AS5147P::_spiCalcEvenParity(uint16_t value) {
  bool even = 0;
  uint8_t i;
  for (i = 0; i < 16; i++) {
    if (value & (0x0001 << i)) {
      even = !even;
    }
  }
  return even;
}
uint8_t AS5147P::read1byte(const uint8_t address) { return 0; }

int16_t AS5147P::read2byte(const uint16_t address) { return 0; }

uint32_t AS5147P::read2byte(const uint8_t address1, const uint8_t address2,
                            bool rorl) {
  esp_err_t ret;
  spi_transaction_t t;

  memset(&t, 0, sizeof(t)); // Zero out the transaction
  t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
  t.length = 16; // SPI_ADDRESS(8bit) + SPI_DATA(8bit)
  auto res = _spiCalcEvenParity(address1 | READ_FLAG2);
  if (res) {
    t.tx_data[0] = (address1 | READ_FLAG2 | PARITY_FLAG);
  } else {
    t.tx_data[0] = (address1 | READ_FLAG2);
  }
  t.tx_data[1] = (address2);
  t.tx_data[2] = 0;
  t.tx_data[3] = 0;
  if (!rorl) {
    ret = spi_device_polling_transmit(spi_l, &t); // Transmit!
  } else {
    ret = spi_device_polling_transmit(spi_r, &t); // Transmit!
  }
  assert(ret == ESP_OK);

  return (int32_t)((uint16_t)(t.rx_data[0]) << 8) | (uint16_t)(t.rx_data[1]);

  // uint16_t before = (uint16_t)((t.rx_data[0] << 8) | (t.rx_data[1]));
  // uint16_t after = (uint16_t)(t.rx_data[0] << 6) | (t.rx_data[1] >> 2);

  // printf("before: %d, after: %d\n", before, after);

  // return after; //(uint32_t)((uint16_t)(t.rx_data[0]) << 8) |
  //(uint16_t)(t.rx_data[1]);
}

int16_t AS5147P::read2byte_2(const uint8_t address1, const uint8_t address2) {
  return 0;
}

void AS5147P::setup() {
  // uint8_t whoami = read1byte(0x0F); //
}
int AS5147P::read_gyro_z() { return read2byte(0x47); }
int AS5147P::read_accel_x() { return read2byte(0x3B); }
int AS5147P::read_accel_y() { return read2byte(0x3D); }
void AS5147P::req_read1byte_itr(const uint8_t address) {}
uint8_t AS5147P::read_1byte_itr() {
  // spi_device_get_trans_result(spi_r, &r_trans, 1 / portTICK_RATE_MS);
  return (uint8_t)(((unsigned short)(r_trans->rx_data[1] & 0xff)));
}

void AS5147P::req_read2byte_itr(const uint8_t address) {
  memset(&itr_t, 0, sizeof(itr_t)); // Zero out the transaction
  itr_t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
  itr_t.length = 24; // SPI_ADDRESS(8bit) + SPI_DATA(8bit)
  itr_t.tx_data[0] = (address | READ_FLAG2);
  itr_t.tx_data[1] = 0;
  itr_t.tx_data[2] = 0;
  // spi_device_queue_trans(spi, &itr_t, 1 / portTICK_RATE_MS); // Transmit!
}
int16_t AS5147P::read_2byte_itr() {
  // spi_device_get_trans_result(spi, &r_trans, 1 / portTICK_RATE_MS);
  return (signed short)((((unsigned short)(r_trans->rx_data[1] & 0xff)) << 8) |
                        ((unsigned short)(r_trans->rx_data[2] & 0xff)));
}
signed short AS5147P::read_2byte_itr2(std::vector<int> &list) {
  // spi_device_get_trans_result(spi, &r_trans, 1 / portTICK_RATE_MS);
  list[0] = r_trans->rx_data[0];
  list[1] = r_trans->rx_data[1];
  list[2] = r_trans->rx_data[2];
  list[3] = r_trans->rx_data[3];
  return (signed short)((((unsigned short)(r_trans->rx_data[1] & 0xff)) << 8) |
                        ((unsigned short)(r_trans->rx_data[2] & 0xff)));
}