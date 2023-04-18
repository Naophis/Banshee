#include "include/sensing_task.hpp"

SensingTask::SensingTask() {}

SensingTask::~SensingTask() {}

void SensingTask::create_task(const BaseType_t xCoreID) {
  xTaskCreatePinnedToCore(task_entry_point, "sensing_task", 8192, this, 2,
                          &handle, xCoreID);
  // xTaskCreatePinnedToCore(task_entry_point0, "sensing_task0", 8192, this, 2,
  //                         &handle, xCoreID);
  // xTaskCreatePinnedToCore(task_entry_point1, "sensing_task1", 8192, this, 2,
  //                         &handle, xCoreID);
  // xTaskCreatePinnedToCore(task_entry_point2, "sensing_task2", 8192, this, 2,
  //                         &handle, xCoreID);
  // xTaskCreatePinnedToCore(task_entry_point3, "sensing_task3", 8192, this, 2,
  //                         &handle, xCoreID);
  // xTaskCreatePinnedToCore(task_entry_point4, "sensing_task4", 8192, this, 2,
  //                         &handle, xCoreID);
}
void SensingTask::set_input_param_entity(
    std::shared_ptr<input_param_t> &_param) {
  param = _param;
}

void SensingTask::task_entry_point(void *task_instance) {
  static_cast<SensingTask *>(task_instance)->task();
}
void SensingTask::task_entry_point0(void *task_instance) {
  static_cast<SensingTask *>(task_instance)->task0();
}
void SensingTask::task_entry_point1(void *task_instance) {
  static_cast<SensingTask *>(task_instance)->task1();
}
void SensingTask::task_entry_point2(void *task_instance) {
  static_cast<SensingTask *>(task_instance)->task2();
}
void SensingTask::task_entry_point3(void *task_instance) {
  static_cast<SensingTask *>(task_instance)->task3();
}
void SensingTask::task_entry_point4(void *task_instance) {
  static_cast<SensingTask *>(task_instance)->task4();
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
  adc2_config_channel_atten(SEN_L45, atten);
  adc2_config_channel_atten(SEN_L90, atten);
  adc2_config_channel_atten(BATTERY, atten);

  while (1) {
    gyro_if.req_read2byte_itr(0x26);
    adc2_get_raw(BATTERY, width, &sensing_result->battery.raw);
    // LED_OFF ADC
    adc2_get_raw(SEN_R90, width, &sensing_result->led_sen_before.right90.raw);
    adc2_get_raw(SEN_L90, width, &sensing_result->led_sen_before.left90.raw);
    adc2_get_raw(SEN_R45, width, &sensing_result->led_sen_before.right45.raw);
    adc2_get_raw(SEN_L45, width, &sensing_result->led_sen_before.left45.raw);

    // 超信地旋回中は発光をサボる
    bool led_on = true;
    if (tgt_val->motion_type == MotionType::PIVOT ||
        tgt_val->motion_type == MotionType::SLALOM) {
      led_on = false;
    }
    // if (led_on) {
    // GPIO.out_w1ts = BIT(LED_R90_BIT);
    // GPIO.out_w1ts = BIT(LED_L90_BIT);
    // }

    lec_cnt = 0;
    for (int i = 0; i < param->led_light_delay_cnt; i++) {
      lec_cnt++;
    }
    // R90
    // GPIO.out_w1ts = BIT(LED_A0_BIT); // low
    // GPIO.out_w1ts = BIT(LED_A1_BIT); // low
    // GPIO.out_w1tc = BIT(LED_EN_BIT); // high

    gpio_set_level(LED_A0, false);
    gpio_set_level(LED_A1, false);
    gpio_set_level(LED_EN, true);

    lec_cnt = 0;
    for (int i = 0; i < 400; i++) {
      lec_cnt++;
    }
    adc2_get_raw(SEN_R90, width, &sensing_result->led_sen_after.right90.raw);
    // GPIO.out_w1ts = BIT(LED_EN_BIT); // low

    gpio_set_level(LED_EN, false);

    // // L90

    gpio_set_level(LED_A0, true);
    gpio_set_level(LED_A1, false);
    gpio_set_level(LED_EN, true);
    lec_cnt = 0;
    for (int i = 0; i < 400; i++) {
      lec_cnt++;
    }
    adc2_get_raw(SEN_L90, width, &sensing_result->led_sen_after.left90.raw);
    gpio_set_level(LED_EN, false);
    // GPIO.out_w1ts = BIT(LED_EN_BIT);// low
    // GPIO.out_w1tc = BIT(LED_R90_BIT);
    // GPIO.out_w1tc = BIT(LED_L90_BIT);
    // if (led_on) {
    //   GPIO.out_w1ts = BIT(LED_R45_BIT);
    //   GPIO.out_w1ts = BIT(LED_L45_BIT);
    // }

    lec_cnt = 0;
    // sensing_result->gyro.raw = gyro_if.read_2byte_itr();
    // gyro_if.req_read2byte_itr(0x3B);
    for (int i = 0; i < param->led_light_delay_cnt; i++) {
      lec_cnt++;
    }
    adc2_get_raw(SEN_R45, width, &sensing_result->led_sen_after.right45.raw);
    adc2_get_raw(SEN_L45, width, &sensing_result->led_sen_after.left45.raw);

    // GPIO.out_w1tc = BIT(LED_R45_BIT);
    // GPIO.out_w1tc = BIT(LED_L45_BIT);

    sensing_result->led_sen.right90.raw =
        std::max(sensing_result->led_sen_after.right90.raw -
                     sensing_result->led_sen_before.right90.raw,
                 0);
    sensing_result->led_sen.right45.raw =
        std::max(sensing_result->led_sen_after.right45.raw -
                     sensing_result->led_sen_before.right45.raw,
                 0);
    sensing_result->led_sen.left45.raw =
        std::max(sensing_result->led_sen_after.left45.raw -
                     sensing_result->led_sen_before.left45.raw,
                 0);
    sensing_result->led_sen.left90.raw =
        std::max(sensing_result->led_sen_after.left90.raw -
                     sensing_result->led_sen_before.left90.raw,
                 0);
    sensing_result->led_sen.front.raw = (sensing_result->led_sen.left90.raw +
                                         sensing_result->led_sen.right90.raw) /
                                        2;
    // GPIO.out_w1tc = BIT(LED_R90_BIT);
    // GPIO.out_w1tc = BIT(LED_R45_BIT);
    // GPIO.out_w1tc = BIT(LED_R45_BIT);
    // GPIO.out_w1tc = BIT(LED_L45_BIT);
    // pcnt_get_counter_value(PCNT_UNIT_0, &sensing_result->encoder_raw.right);
    // pcnt_counter_clear(PCNT_UNIT_0);
    // pcnt_get_counter_value(PCNT_UNIT_1, &sensing_result->encoder_raw.left);
    // pcnt_counter_clear(PCNT_UNIT_1);

    if (GY_MODE == 0) {
      // sensing_result->gyro.raw = gyro_if.read2byte(0x26);
      // sensing_result->accel_x.raw = gyro_if.read2byte(0x28);
      sensing_result->gyro.raw = gyro_if.read_2byte_itr();
      // gyro_if.req_read2byte_itr(0x28);
      sensing_result->accel_x.raw = 0;
      sensing_result->accel_y.raw = 0;
      // gyro_if.read_2byte_itr();
      // gyro_if.req_read2byte_itr(0x3D);
      // sensing_result->accel_y.raw = gyro_if.read_2byte_itr();

    } else {
      if (gyro_q.size() == GY_DQ_SIZE) {
        sensing_result->gyro.raw = gyro_q[gyro_q.size() - 1];
        for (int i = 0; i < GY_DQ_SIZE; i++) {
          sensing_result->gyro_list[i] = gyro_q[i];
        }
      } else {
        sensing_result->gyro.raw = 0;
      }
    }

    auto enc_l = enc_if.read2byte(0x3F, 0xFF, false) & 0x3FFF;
    auto enc_r = enc_if.read2byte(0x3F, 0xFF, true) & 0x3FFF;

    sensing_result->battery.data =
        BATTERY_GAIN * 3 * sensing_result->battery.raw / 4096;

    sensing_result->encoder.right_old = sensing_result->encoder.right;
    sensing_result->encoder.left_old = sensing_result->encoder.left;
    sensing_result->encoder.right = enc_r;
    sensing_result->encoder.left = enc_l;

    vTaskDelay(xDelay);
  }
}

float SensingTask::calc_sensor(float data, float a, float b) {
  auto res = a / std::log(data) - b;
  if (res < 5 || res > 180)
    return 180;
  return res;
}
void SensingTask::task0() {}
void SensingTask::task1() {}
void SensingTask::task2() {}
void SensingTask::task3() {}
void SensingTask::task4() {}