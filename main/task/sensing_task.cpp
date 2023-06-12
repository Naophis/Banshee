#include "include/sensing_task.hpp"

SensingTask::SensingTask() {}

SensingTask::~SensingTask() {}

void SensingTask::timer_200ms_callback(void *arg) {
  SensingTask *instance = static_cast<SensingTask *>(arg);
  instance->timer_200ms_callback_main();
}
void SensingTask::timer_10ms_callback(void *arg) {
  SensingTask *instance = static_cast<SensingTask *>(arg);
  instance->timer_10ms_callback_main();
}

void SensingTask::timer_10ms_callback_main() {
  const auto se = get_sensing_entity();
  switch (cnt_a) {
  case 0:
    adc2_get_raw(BATTERY, ADC_WIDTH_BIT_12, &se->battery.raw);
    se->gyro_list[0] = gyro_if.read_2byte_itr();
    break;
  case 1:
    adc2_get_raw(SEN_R90, width, &se->led_sen_after.right90.raw);
    se->gyro_list[1] = gyro_if.read_2byte_itr();
    break;
  case 2:
    adc2_get_raw(SEN_L90, width, &se->led_sen_after.left90.raw);
    se->gyro_list[2] = gyro_if.read_2byte_itr();
    break;
  case 3:
    adc2_get_raw(SEN_R45, width, &se->led_sen_after.right45.raw);
    adc2_get_raw(SEN_R45_2, width, &se->led_sen_after.right45_2.raw);
    se->gyro_list[3] = gyro_if.read_2byte_itr();
    break;
  case 4:
    adc2_get_raw(SEN_L45, width, &se->led_sen_after.left45.raw);
    adc2_get_raw(SEN_L45_2, width, &se->led_sen_after.left45_2.raw);
    se->gyro_list[4] = gyro_if.read_2byte_itr();
    se->led_sen.right90.raw = std::max(
        se->led_sen_after.right90.raw - se->led_sen_before.right90.raw, 0);
    se->led_sen.right45.raw = std::max(
        se->led_sen_after.right45.raw - se->led_sen_before.right45.raw, 0);
    se->led_sen.right45_2.raw = std::max(
        se->led_sen_after.right45_2.raw - se->led_sen_before.right45_2.raw, 0);
    se->led_sen.left45_2.raw = std::max(
        se->led_sen_after.left45_2.raw - se->led_sen_before.left45_2.raw, 0);
    se->led_sen.left45.raw = std::max(
        se->led_sen_after.left45.raw - se->led_sen_before.left45.raw, 0);
    se->led_sen.left90.raw = std::max(
        se->led_sen_after.left90.raw - se->led_sen_before.left90.raw, 0);
    se->led_sen.front.raw =
        (se->led_sen.left90.raw + se->led_sen.right90.raw) / 2;
    cnt_a = -1;
    break;
  }
  esp_timer_stop(timer_50us);
  gpio_set_level(LED_EN, false);
  cnt_a++;
}

void SensingTask::timer_200ms_callback_main() {
  const auto se = get_sensing_entity();
  switch (cnt_a) {
  case 0:
    adc2_get_raw(BATTERY, ADC_WIDTH_BIT_12, &se->battery.raw);
    adc2_get_raw(SEN_L90, width, &se->led_sen_before.left90.raw);
    adc2_get_raw(SEN_R45, width, &se->led_sen_before.right45.raw);
    adc2_get_raw(SEN_R45_2, width, &se->led_sen_before.right45_2.raw);
    adc2_get_raw(SEN_L45, width, &se->led_sen_before.left45.raw);
    adc2_get_raw(SEN_L45_2, width, &se->led_sen_before.left45_2.raw);
    gyro_if.req_read2byte_itr(0x26);
    break;
  case 1:
    gpio_set_level(LED_A0, false);
    gpio_set_level(LED_A1, false);
    gpio_set_level(LED_EN, true);
    gyro_if.req_read2byte_itr(0x26);
    break;
  case 2:
    gpio_set_level(LED_A0, true);
    gpio_set_level(LED_A1, false);
    gpio_set_level(LED_EN, true);
    gyro_if.req_read2byte_itr(0x26);
    break;
  case 3:
    gpio_set_level(LED_A0, false);
    gpio_set_level(LED_A1, true);
    gpio_set_level(LED_EN, true);
    gyro_if.req_read2byte_itr(0x26);
    break;
  case 4:
    gpio_set_level(LED_A0, true);
    gpio_set_level(LED_A1, true);
    gpio_set_level(LED_EN, true);
    gyro_if.req_read2byte_itr(0x26);
    break;
  }
  esp_timer_stop(timer_50us);
  esp_timer_start_once(timer_50us, 10); // 1ms/4
}

void SensingTask::create_task(const BaseType_t xCoreID) {
  xTaskCreatePinnedToCore(task_entry_point, "sensing_task", 8192, this, 2,
                          &handle, xCoreID);
  const esp_timer_create_args_t timer_200us_args = {
      .callback = &SensingTask::timer_200ms_callback,
      .arg = this,
      .name = "timer_200us"};
  esp_timer_create(&timer_200us_args, &timer_200us);

  const esp_timer_create_args_t timer_50us_args = {
      .callback = &SensingTask::timer_10ms_callback,
      .arg = this,
      .name = "timer_50us"};
  esp_timer_create(&timer_50us_args, &timer_50us);
}
void SensingTask::set_input_param_entity(
    std::shared_ptr<input_param_t> &_param) {
  param = _param;
}

void SensingTask::task_entry_point(void *task_instance) {
  static_cast<SensingTask *>(task_instance)->task();
}

void SensingTask::set_sensing_entity(
    std::shared_ptr<sensing_result_entity_t> &_sensing_result) {
  sensing_result = _sensing_result;
}

void SensingTask::set_tgt_val(std::shared_ptr<motion_tgt_val_t> &_tgt_val) {
  tgt_val = _tgt_val;
}
void SensingTask::encoder_init(const pcnt_unit_t unit, const gpio_num_t pinA,
                               const gpio_num_t pinB) {
  pcnt_config_0.pulse_gpio_num = pinA;
  pcnt_config_0.ctrl_gpio_num = pinB;
  pcnt_config_0.unit = unit;

  pcnt_config_1.pulse_gpio_num = pinB;
  pcnt_config_1.ctrl_gpio_num = pinA;
  pcnt_config_1.unit = unit;

  pcnt_unit_config(&pcnt_config_0);
  pcnt_unit_config(&pcnt_config_1);

  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);

  pcnt_counter_resume(unit);
}

void SensingTask::task() {
  // timer_init_grp0_timer0();
  if (!GY_MODE) {
    gyro_if.init();
    gyro_if.setup();
    enc_if.init();
  }
  ready = true;

  // sensing init
  adc2_config_channel_atten(SEN_R90, atten);
  adc2_config_channel_atten(SEN_R45, atten);
  adc2_config_channel_atten(SEN_R45_2, atten);
  adc2_config_channel_atten(SEN_L45, atten);
  adc2_config_channel_atten(SEN_L45_2, atten);
  adc2_config_channel_atten(SEN_L90, atten);
  adc2_config_channel_atten(BATTERY, atten);

  esp_timer_start_periodic(timer_200us, 200);

  while (1) {
    // gyro_if.req_read2byte_itr(0x26);
    // LED_OFF ADC
    // 超信地旋回中は発光をサボる
    bool led_on = true;
    if (tgt_val->motion_type == MotionType::PIVOT ||
        tgt_val->motion_type == MotionType::SLALOM) {
      led_on = false;
    }

    auto enc_l = enc_if.read2byte(0x3F, 0xFF, false) & 0x3FFF;
    auto enc_r = enc_if.read2byte(0x3F, 0xFF, true) & 0x3FFF;

    sensing_result->battery.data =
        BATTERY_GAIN * 4.2 * sensing_result->battery.raw / 4096;

    sensing_result->encoder.right_old = sensing_result->encoder.right;
    sensing_result->encoder.left_old = sensing_result->encoder.left;
    sensing_result->encoder.right = -enc_r;
    sensing_result->encoder.left = -enc_l;

    vTaskDelay(xDelay);
  }
}

float SensingTask::calc_sensor(float data, float a, float b) {
  auto res = a / std::log(data) - b;
  if (res < 5 || res > 180)
    return 180;
  return res;
}