#include "include/ui.hpp"

void UserInterface::set_sensing_entity(
    std::shared_ptr<sensing_result_entity_t> &_sensing_result) {
  sensing_result = _sensing_result;
}

void UserInterface::set_tgt_val(std::shared_ptr<motion_tgt_val_t> &_tgt_val) {
  tgt_val = _tgt_val;
}

bool UserInterface::button_state() { return !gpio_get_level(SW1); }

bool UserInterface::button_state_hold() {
  if (!gpio_get_level(SW1)) {
    while (!gpio_get_level(SW1))
      ;
    return true;
  } else {
    return false;
  }
}
void UserInterface::init_i2c_master() {
  i2c_port_t port = 0;
  i2c_config_t config;

  config.mode = I2C_MODE_MASTER;
  config.sda_io_num = SDA_PIN;
  config.scl_io_num = SCL_PIN;
  config.sda_pullup_en = true;
  config.scl_pullup_en = true;
  config.clk_flags = 0;
  config.master.clk_speed = CONFIG_I2C_MASTER_FREQUENCY;

  i2c_param_config(port, &config);
  i2c_driver_install(port, config.mode, 0, 0, 0);
}

uint8_t UserInterface::SCCB_Write(uint8_t slv_addr, uint8_t reg, uint8_t data) {
  i2c_port_t port = 0;
  esp_err_t ret = ESP_FAIL;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, slv_addr, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(port, cmd, 1 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return ret == ESP_OK ? 0 : -1;
}

int UserInterface::encoder_operation() {
  float v_r = sensing_result->ego.v_r;
  if (v_r > ENC_OPE_V_R_TH) {
    music_sync(MUSIC::G6_, 75);
    return 1;
  }
  if (v_r < -ENC_OPE_V_R_TH) {
    music_sync(MUSIC::C6_, 75);
    return -1;
  }
  return 0;
}
void UserInterface::music_async(MUSIC m, int time) {
  tgt_val->buzzer.hz = (int)m;
  tgt_val->buzzer.time = time;
  int buzzer_timestamp = tgt_val->buzzer.timstamp;
  tgt_val->buzzer.timstamp = ++buzzer_timestamp;
}
void UserInterface::music_sync(MUSIC m, int time) {
  music_async(m, time);
  vTaskDelay(time / portTICK_PERIOD_MS);
}
void UserInterface::motion_check() {
  int c = 0;
  tgt_val->nmr.motion_type = MotionType::READY;
  tgt_val->nmr.timstamp++;
  xQueueReset(*qh);
  xQueueSendToFront(*qh, &tgt_val, 1);
  vTaskDelay(1.0 / portTICK_PERIOD_MS);
  while (1) {
    c++;
    if (c % 2 == 0) {
      LED_on_all();
    } else {
      LED_off_all();
    }
    if (button_state_hold()) {
      break;
    }
    if (sensing_result->ego.left90_mid_dist < 60 &&
        sensing_result->ego.right90_mid_dist < 60 &&
        sensing_result->ego.left90_mid_dist > 10 &&
        sensing_result->ego.right90_mid_dist > 10) {
      LED_off_all();
      for (int i = 0; i < 2; i++) {
        music_sync(MUSIC::C6_, 100);
        vTaskDelay(tgt_val->buzzer.time / portTICK_PERIOD_MS);
      }
      break;
    }
    vTaskDelay(50.0 / portTICK_PERIOD_MS);
  }
}

void UserInterface::coin(int time) {
  music_sync(MUSIC::B5_, time);
  music_sync(MUSIC::E6_, 2 * time);
}

void UserInterface::hello_exia() {
  int time = 120;
  music_sync(MUSIC::A6_, time);
  vTaskDelay(10.0 / portTICK_PERIOD_MS);
  music_sync(MUSIC::A6_, time);
  vTaskDelay(10.0 / portTICK_PERIOD_MS);
  music_sync(MUSIC::A6_, time);
  vTaskDelay(10.0 / portTICK_PERIOD_MS);
  music_sync(MUSIC::A6_, 3 * time);
  vTaskDelay(time / portTICK_PERIOD_MS);
  music_sync(MUSIC::A6_, time);
  vTaskDelay(10.0 / portTICK_PERIOD_MS);
  music_sync(MUSIC::A6_, time);
  vTaskDelay(10.0 / portTICK_PERIOD_MS);
  music_sync(MUSIC::A6_, time);
  vTaskDelay(10.0 / portTICK_PERIOD_MS);
  music_sync(MUSIC::A6_, time);
  vTaskDelay(10.0 / portTICK_PERIOD_MS);
  music_sync(MUSIC::C7_, 1.5 * time);
  vTaskDelay(time / 3 / portTICK_PERIOD_MS);
  music_sync(MUSIC::G6_, 2 * time);
  vTaskDelay(10.0 / portTICK_PERIOD_MS);
  music_sync(MUSIC::F6_, 2 * time);
  vTaskDelay(10.0 / portTICK_PERIOD_MS);
  music_sync(MUSIC::G6_, 2 * time);
  vTaskDelay(10.0 / portTICK_PERIOD_MS);
}

void UserInterface::LED_on_off(char idx, bool state) {
  uint8_t blight = state ? 0x1F : 0x00;
  writeBuffer[0] = (idx << 5) | blight;
  writeBuffer[1] = 0x00;
  SCCB_Write(0x9A, writeBuffer[0], writeBuffer[0]);
}

void UserInterface::LED_bit(int b0, int b1, int b2, int b3, int b4, int b5) {
  LED_on_off(1, (b5 == 1));
  LED_on_off(2, (b4 == 1));
  LED_on_off(3, (b3 == 1));
  LED_on_off(4, (b2 == 1));
  LED_on_off(5, (b1 == 1));
  LED_on_off(6, (b0 == 1));
}
void UserInterface::LED_otherwise(int byte, int state) {}
void UserInterface::LED(int byte, int state) {}
void UserInterface::LED_on(int byte) {}

void UserInterface::LED_off(int byte) {}
void UserInterface::LED_off_all() {
  const bool state = false;
  LED_on_off(1, state);
  LED_on_off(2, state);
  LED_on_off(3, state);
  LED_on_off(4, state);
  LED_on_off(5, state);
  LED_on_off(6, state);
}
void UserInterface::LED_on_all() {
  const bool state = true;
  LED_on_off(1, state);
  LED_on_off(2, state);
  LED_on_off(3, state);
  LED_on_off(4, state);
  LED_on_off(5, state);
  LED_on_off(6, state);
}

TurnDirection UserInterface::select_direction() {
  TurnDirection td = TurnDirection::None;
  bool b = true;
  while (1) {
    if (sensing_result->ego.v_r > ENC_OPE_V_R_TH) {
      music_sync(MUSIC::G6_, 75);
      td = TurnDirection::Right;
      LED_bit(1, 0, 0, 0, 0, 0);
    }
    if (sensing_result->ego.v_l > ENC_OPE_V_R_TH) {
      music_sync(MUSIC::C6_, 75);
      td = TurnDirection::Left;
      LED_bit(0, 0, 0, 0, 0, 1);
    }

    if (td != TurnDirection::None) {
      if (button_state_hold()) {
        coin(80);
        return td;
      }
    } else {
      LED_bit((int)b, 0, 0, 0, 0, (int)b);
      b = b ? false : true;
    }
    vTaskDelay(25.0 / portTICK_PERIOD_MS);
  }
}

TurnDirection UserInterface::select_direction2() {
  return select_direction(); //
}
void UserInterface::error() {
  int time = 120;
  for (int i = 0; i < 4; i++)
    music_sync(MUSIC::C4_, time);
}