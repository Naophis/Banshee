#include "include/sensing_task.hpp"

SensingTask::SensingTask() {}

SensingTask::~SensingTask() {}

void SensingTask::timer_250us_callback(void *arg) {
  SensingTask *instance = static_cast<SensingTask *>(arg);
  instance->timer_250us_callback_main();
}
void SensingTask::timer_200us_callback(void *arg) {
  SensingTask *instance = static_cast<SensingTask *>(arg);
  instance->timer_200us_callback_main();
}
void SensingTask::timer_10us_callback(void *arg) {
  SensingTask *instance = static_cast<SensingTask *>(arg);
  instance->timer_10us_callback_main();
}

void SensingTask::timer_10us_callback_main() {
  const auto se = get_sensing_entity();
  int32_t enc = 0;
  switch (cnt_a) {
  case 0:
    adc2_get_raw(BATTERY, ADC_WIDTH_BIT_12, &se->battery.raw);
    se->battery.data = BATTERY_GAIN * 4.2 * sensing_result->battery.raw / 4096;
    // se->gyro_list[0] = gyro_if.read_2byte_itr();
    break;
  case 1:
    adc2_get_raw(SEN_R90, width, &se->led_sen_after.right90.raw);
    // se->gyro_list[1] = gyro_if.read_2byte_itr();
    break;
  case 2:
    adc2_get_raw(SEN_L90, width, &se->led_sen_after.left90.raw);
    // se->gyro_list[2] = gyro_if.read_2byte_itr();

    // enc = enc_if.read2byte(0x3F, 0xFF, false) & 0x3FFF;
    // se->encoder.left_old = se->encoder.left;
    // se->encoder.left = -enc;
    break;
  case 3:
    adc2_get_raw(SEN_R45, width, &se->led_sen_after.right45.raw);
    adc2_get_raw(SEN_R45_2, width, &se->led_sen_after.right45_2.raw);
    // se->gyro_list[3] = gyro_if.read_2byte_itr();

    // enc = enc_if.read2byte(0x3F, 0xFF, true) & 0x3FFF;
    // se->encoder.right_old = se->encoder.right;
    // se->encoder.right = -enc;
    break;
  case 4:
    adc2_get_raw(SEN_L45, width, &se->led_sen_after.left45.raw);
    adc2_get_raw(SEN_L45_2, width, &se->led_sen_after.left45_2.raw);
    // se->gyro_list[4] = gyro_if.read_2byte_itr();
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
    // mt->notify();
    // mt->mp->notify();
    cnt_a = -1;
    break;
  }
  set_gpio_state(LED_EN, false);
  cnt_a++;
  // esp_timer_stop(timer_10us);
}

// 壁切れ時に必要ないセンシング処理をやめて、本来ほしいデータにまわす

void SensingTask::timer_200us_callback_main() {
  const auto se = get_sensing_entity();
  switch (cnt_a) {
  case 0:
    adc2_get_raw(BATTERY, ADC_WIDTH_BIT_12, &se->battery.raw);
    break;
  case 1:
    adc2_get_raw(SEN_R90, width, &se->led_sen_before.left90.raw);
    set_gpio_state(LED_A0, false);
    set_gpio_state(LED_A1, false);
    set_gpio_state(LED_EN, true);
    break;
  case 2:
    adc2_get_raw(SEN_L90, width, &se->led_sen_before.left90.raw);
    set_gpio_state(LED_A0, true);
    set_gpio_state(LED_A1, false);
    set_gpio_state(LED_EN, true);
    break;
  case 3:
    adc2_get_raw(SEN_R45, width, &se->led_sen_before.right45.raw);
    adc2_get_raw(SEN_R45_2, width, &se->led_sen_before.right45_2.raw);
    set_gpio_state(LED_A0, false);
    set_gpio_state(LED_A1, true);
    set_gpio_state(LED_EN, true);
    break;
  case 4:
    adc2_get_raw(SEN_L45, width, &se->led_sen_before.left45.raw);
    adc2_get_raw(SEN_L45_2, width, &se->led_sen_before.left45_2.raw);
    set_gpio_state(LED_A0, true);
    set_gpio_state(LED_A1, true);
    set_gpio_state(LED_EN, true);
    break;
  }
  esp_timer_stop(timer_10us);
  esp_timer_start_once(timer_10us, 10); // 1ms/4
}

void SensingTask::timer_250us_callback_main() {}

void SensingTask::create_task(const BaseType_t xCoreID) {
  xTaskCreatePinnedToCore(task_entry_point, "sensing_task", 8192, this, 2,
                          &handle, xCoreID);
  const esp_timer_create_args_t timer_200us_args = {
      .callback = &SensingTask::timer_200us_callback,
      .arg = this,
      .name = "timer_100us"};
  esp_timer_create(&timer_200us_args, &timer_200us);

  const esp_timer_create_args_t timer_10us_args = {
      .callback = &SensingTask::timer_10us_callback,
      .arg = this,
      .name = "timer_10us"};
  esp_timer_create(&timer_10us_args, &timer_10us);
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
  // esp_timer_start_periodic(timer_250us, 250);

  while (1) {
    // gyro_if.req_read2byte_itr(0x26);
    // LED_OFF ADC
    // 超信地旋回中は発光をサボる
    bool led_on = true;
    if (tgt_val->motion_type == MotionType::PIVOT ||
        tgt_val->motion_type == MotionType::SLALOM) {
      led_on = false;
    };

    gyro_if.req_read2byte_itr(0x26);
    int32_t enc_r = enc_if.read2byte(0x3F, 0xFF, true) & 0x3FFF;
    sensing_result->encoder.right_old = sensing_result->encoder.right;
    sensing_result->encoder.right = -enc_r;

    int32_t enc_l = enc_if.read2byte(0x3F, 0xFF, false) & 0x3FFF;
    sensing_result->encoder.left_old = sensing_result->encoder.left;
    sensing_result->encoder.left = -enc_l;

    sensing_result->gyro_list[4] = gyro_if.read_2byte_itr();
    sensing_result->gyro.raw = sensing_result->gyro_list[4];
    sensing_result->gyro.data = (float)(sensing_result->gyro_list[4]);
    // vTaskDelay(xDelay);
    vTaskDelay(1.0 / portTICK_PERIOD_MS);
  }
}

float SensingTask::calc_sensor(float data, float a, float b) {
  auto res = a / std::log(data) - b;
  if (res < 5 || res > 180)
    return 180;
  return res;
}