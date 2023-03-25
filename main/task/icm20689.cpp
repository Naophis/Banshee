#include "include/icm20689.hpp"

ICM20689::ICM20689() {}
ICM20689::~ICM20689() {}
void ICM20689::init() {
  esp_err_t ret;
  spi_bus_config_t buscfg = {
      .mosi_io_num = EN_MOSI,
      .miso_io_num = EN_MISO,
      .sclk_io_num = EN_CLK,
      .quadwp_io_num = -1,  // unused
      .quadhd_io_num = -1,  // unused
      .max_transfer_sz = 2, // bytes
      .flags = SPICOMMON_BUSFLAG_MASTER,
      .intr_flags = 0 // 割り込みをしない
  };
  spi_device_interface_config_t devcfg = {
      .mode = 3,
      .clock_speed_hz = 7 * 1000 * 1000, // aaaaaaaaaaa
      .spics_io_num = EN_GN_SSL,
      .queue_size = 7,
  };
  // Initialize the SPI bus
  ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_DISABLED);
  ESP_ERROR_CHECK(ret);
  // Attach the LCD to the SPI bus
  ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
  ESP_ERROR_CHECK(ret);
}
uint8_t ICM20689::write1byte(const uint8_t address, const uint8_t data) {
  esp_err_t ret;
  spi_transaction_t t;
  memset(&t, 0, sizeof(t)); // Zero out the transaction

  // printf("%2x, %2x\n", address, data);
  // printf("%2x, %2x\n", address, 0x0f & data);
  t.length = 24; // SPI_ADDRESS(8bit) + SPI_DATA(8bit)
  t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
  t.tx_data[0] = address;
  t.tx_data[1] = (uint8_t)(0xff & data);
  t.tx_data[2] = 0;

  // uint16_t tx_data = (address) << 8 | (0xff & data);
  // tx_data = SPI_SWAP_DATA_TX(tx_data, 16);
  // t.tx_buffer = &(t.tx_data);
  ret = spi_device_polling_transmit(spi, &t); // Transmit!
  assert(ret == ESP_OK);                      // Should have had no issues.
  return 0;
}

uint8_t ICM20689::read1byte(const uint8_t address) {
  esp_err_t ret;
  spi_transaction_t t;
  memset(&t, 0, sizeof(t)); // Zero out the transaction
  t.flags = SPI_TRANS_USE_RXDATA;
  t.length = 16; // SPI_ADDRESS(8bit) + SPI_DATA(8bit)
  uint16_t tx_data = (address | READ_FLAG) << 8;
  tx_data = SPI_SWAP_DATA_TX(tx_data, 16);
  t.tx_buffer = &tx_data;
  ret = spi_device_polling_transmit(spi, &t); // Transmit!
  assert(ret == ESP_OK);                      // Should have had no issues.
  uint8_t data =
      SPI_SWAP_DATA_RX(*(uint16_t *)t.rx_data, 16) & 0x00FF; // FF + Data
  return data;
}

int16_t ICM20689::read2byte(const uint8_t address) {
  esp_err_t ret;
  spi_transaction_t t;
  memset(&t, 0, sizeof(t)); // Zero out the transaction
  t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
  t.length = 24; // SPI_ADDRESS(8bit) + SPI_DATA(8bit)
  t.tx_data[0] = (address | READ_FLAG);
  t.tx_data[1] = 0;
  t.tx_data[2] = 0;
  ret = spi_device_polling_transmit(spi, &t); // Transmit!
  assert(ret == ESP_OK);                      // Should have had no issues.
  return (signed short)((((unsigned short)(t.rx_data[1] & 0xff)) << 8) |
                        ((unsigned short)(t.rx_data[2] & 0xff)));
}

void ICM20689::setup() {
  uint8_t whoami = read1byte(0x0F);
  printf("%d\n", whoami);
  write1byte(0x6B, 0x80); //スリープ解除?
  vTaskDelay(10.0/ portTICK_PERIOD_MS);
  write1byte(0x68, 0x04); //ジャイロリセット
  vTaskDelay(10.0/ portTICK_PERIOD_MS);
  write1byte(0x6A, 0x10); // uercontrol i2c=disable
  vTaskDelay(10.0/ portTICK_PERIOD_MS);
  write1byte(0x1B, 0x18); // 2000
  vTaskDelay(10.0/ portTICK_PERIOD_MS);
  // write1byte(0x1C, 0x08); // 4g
  // write1byte(0x1C, 0x10); // 8g
  // write1byte(0x1C, 0x18); // 16g
  // vTaskDelay(10.0/ portTICK_PERIOD_MS);
  // write1byte(0x1D, 0x00); // 4kHz
  // vTaskDelay(10.0/ portTICK_PERIOD_MS);
}
int ICM20689::read_gyro_z() {
  return read2byte(0x47);
}
int ICM20689::read_accel_x() {
  return read2byte(0x3B);
}
int ICM20689::read_accel_y() {
  return read2byte(0x3D);
}
void ICM20689::req_read1byte_itr(const uint8_t address) {
  memset(&itr_t, 0, sizeof(itr_t)); // Zero out the transaction
  itr_t.flags = SPI_TRANS_USE_RXDATA;
  itr_t.length = 16; // SPI_ADDRESS(8bit) + SPI_DATA(8bit)
  uint16_t tx_data = (address | READ_FLAG) << 8;
  tx_data = SPI_SWAP_DATA_TX(tx_data, 16);
  itr_t.tx_buffer = &tx_data;
  spi_device_queue_trans(spi, &itr_t, 1 / portTICK_RATE_MS); // Transmit!
}
uint8_t ICM20689::read_1byte_itr() {
  spi_device_get_trans_result(spi, &r_trans, 1 / portTICK_RATE_MS);
  return (uint8_t)(((unsigned short)(r_trans->rx_data[1] & 0xff)));
}

void ICM20689::req_read2byte_itr(const uint8_t address) {
  memset(&itr_t, 0, sizeof(itr_t)); // Zero out the transaction
  itr_t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
  itr_t.length = 24; // SPI_ADDRESS(8bit) + SPI_DATA(8bit)
  itr_t.tx_data[0] = (address | READ_FLAG);
  itr_t.tx_data[1] = 0;
  itr_t.tx_data[2] = 0;
  spi_device_queue_trans(spi, &itr_t, 1 / portTICK_RATE_MS); // Transmit!
}
int16_t ICM20689::read_2byte_itr() {
  spi_device_get_trans_result(spi, &r_trans, 1 / portTICK_RATE_MS);
  return (signed short)((((unsigned short)(r_trans->rx_data[1] & 0xff)) << 8) |
                        ((unsigned short)(r_trans->rx_data[2] & 0xff)));
}
signed short ICM20689::read_2byte_itr2(std::vector<int> &list) {
  spi_device_get_trans_result(spi, &r_trans, 1 / portTICK_RATE_MS);
  list[0] = r_trans->rx_data[0];
  list[1] = r_trans->rx_data[1];
  list[2] = r_trans->rx_data[2];
  list[3] = r_trans->rx_data[3];
  return (signed short)((((unsigned short)(r_trans->rx_data[1] & 0xff)) << 8) |
                        ((unsigned short)(r_trans->rx_data[2] & 0xff)));
}