#include "include/sensing.hpp"

Sensing::Sensing() {}

Sensing::~Sensing() {}

void Sensing::set_input_param_entity(std::shared_ptr<input_param_t> &_param) {
  param = _param;
}

void Sensing::set_sensing_entity(
    std::shared_ptr<sensing_result_entity_t> &_sensing_result) {
  sensing_result = _sensing_result;
}

// void Sensing::set_planning_task(std::shared_ptr<PlanningTask> &_pt) {
//   pt = _pt;
// }
void Sensing::set_tgt_val(std::shared_ptr<motion_tgt_val_t> &_tgt_val) {
  tgt_val = _tgt_val;
}

void Sensing::init() {
  // timer_init_grp0_timer0();
  if (!GY_MODE) {
    gyro_if.init();
    gyro_if.setup();
    enc_if.init();
  }
  ready = true;

  const auto se = get_sensing_entity();
  // sensing init
  adc2_config_channel_atten(SEN_R90, atten);
  adc2_config_channel_atten(SEN_R45, atten);
  // adc2_config_channel_atten(SEN_R45_2, atten);
  adc2_config_channel_atten(SEN_L45, atten);
  // adc2_config_channel_atten(SEN_L45_2, atten);
  adc2_config_channel_atten(SEN_L90, atten);
  adc2_config_channel_atten(BATTERY, atten);
}
void Sensing::task(bool search_mode, bool mode_select, bool skip_sen) {

  const auto se = get_sensing_entity();
  int64_t start = 0;
  int64_t end = 0;
  int64_t start2 = 0;
  int64_t end2 = 0;
  int64_t start_before = 0;

  int64_t last_gyro_time = 0;
  int64_t now_gyro_time = 0;

  int64_t last_enc_r_time = 0;
  int64_t now_enc_r_time = 0;

  int64_t last_enc_l_time = 0;
  int64_t now_enc_l_time = 0;

  bool r90 = true;
  bool l90 = true;
  bool r45 = true;
  bool l45 = true;

  start_before = start;
  start = esp_timer_get_time();
  se->calc_time = (int16_t)(start - start_before);
  now_gyro_time = esp_timer_get_time();
  gyro_if.req_read2byte_itr(0x26);
  start2 = esp_timer_get_time();
  if (!skip_sen) {
    adc2_get_raw(BATTERY, width, &sensing_result->battery.raw);
  }
  // LED_OFF ADC

  if (search_mode && tgt_val->motion_type == MotionType::STRAIGHT) {
    // 加速中は正面は発光させない
    if (tgt_val->ego_in.state == 0) {
      r90 = l90 = false;
    }
  }
  if (search_mode && tgt_val->nmr.sct == SensorCtrlType::NONE) {
    // 探索中、壁制御しないときはOFF
    r90 = l90 = false;
    r45 = l45 = false;
  }
  if (tgt_val->nmr.sct == SensorCtrlType::Dia) {
    if (tgt_val->ego_in.state == 0) {
      // 斜め壁制御加速中は横は発光させない
      r45 = l45 = false;
    }
  }
  if (tgt_val->nmr.sct == SensorCtrlType::Straight) {
    r90 = l90 = true;
    r45 = l45 = true;
  }
  if (tgt_val->motion_type == MotionType::READY) {
    // motion check用
    r90 = l90 = true;
    r45 = l45 = false;
  }
  if (tgt_val->motion_type == MotionType::FRONT_CTRL) {
    // 前壁制御中は横は発光させない
    r90 = l90 = true;
    r45 = l45 = false;
  }
  if (tgt_val->motion_type == MotionType::SENSING_DUMP) {
    r90 = l90 = true;
    r45 = l45 = true;
  }

  if (r90) {
    adc2_get_raw(SEN_R90, width, &sensing_result->led_sen_before.right90.raw);
  } else {
    sensing_result->led_sen_before.right90.raw = 0;
  }
  if (l90) {
    adc2_get_raw(SEN_L90, width, &sensing_result->led_sen_before.left90.raw);
  } else {
    sensing_result->led_sen_before.left90.raw = 0;
  }
  if (r45) {
    adc2_get_raw(SEN_R45, width, &sensing_result->led_sen_before.right45.raw);
  } else {
    sensing_result->led_sen_before.right45.raw = 0;
  }
  if (l45) {
    adc2_get_raw(SEN_L45, width, &sensing_result->led_sen_before.left45.raw);
  } else {
    sensing_result->led_sen_before.left45.raw = 0;
  }
  end2 = esp_timer_get_time();

  r90 = true;
  l90 = true;
  r45 = true;
  l45 = true;
  // LED_OFF ADC
  // 超信地旋回中は発光をサボる
  bool led_on = true;
  if (tgt_val->motion_type == MotionType::PIVOT ||
      tgt_val->motion_type == MotionType::SLALOM) {
    led_on = false;
    // if (tgt_val->ego_in.sla_param.counter >
    //     (tgt_val->ego_in.sla_param.limit_time_count / 2)) {
    //   led_on = true;
    // }
  };
  if (mode_select) {
    led_on = false;
  }
  if (led_on) {

    if (search_mode && tgt_val->motion_type == MotionType::STRAIGHT) {
      // 加速中は正面は発光させない
      if (tgt_val->ego_in.state == 0) {
        r90 = l90 = false;
      }
    }
    if (search_mode && tgt_val->nmr.sct == SensorCtrlType::NONE) {
      // 探索中、壁制御しないときはOFF
      r90 = l90 = false;
      r45 = l45 = false;
    }
    if (tgt_val->nmr.sct == SensorCtrlType::Dia) {
      if (tgt_val->ego_in.state == 0) {
        // 斜め壁制御加速中は横は発光させない
        r45 = l45 = false;
      }
    }
    if (tgt_val->nmr.sct == SensorCtrlType::Straight) {
      r90 = l90 = true;
      r45 = l45 = true;
    }
    if (tgt_val->motion_type == MotionType::READY) {
      // motion check用
      r90 = l90 = true;
      r45 = l45 = false;
    }
    if (tgt_val->motion_type == MotionType::FRONT_CTRL) {
      // 前壁制御中は横は発光させない
      r90 = l90 = true;
      r45 = l45 = false;
    }
    if (tgt_val->motion_type == MotionType::SENSING_DUMP) {
      r90 = l90 = true;
      r45 = l45 = true;
    }
    if (r90) { // R90
      set_gpio_state(LED_A0, false);
      set_gpio_state(LED_A1, false);
      set_gpio_state(LED_EN, true);
      lec_cnt = 0;
      for (int i = 0; i < param->led_light_delay_cnt; i++) {
        lec_cnt++;
      }
      adc2_get_raw(SEN_R90, width, &se->led_sen_after.right90.raw);
    }
    if (l90) { // L90
      set_gpio_state(LED_A0, true);
      set_gpio_state(LED_A1, false);
      set_gpio_state(LED_EN, true);
      lec_cnt = 0;
      for (int i = 0; i < param->led_light_delay_cnt; i++) {
        lec_cnt++;
      }
      adc2_get_raw(SEN_L90, width, &se->led_sen_after.left90.raw);
    }
    if (r45) { // R45
      set_gpio_state(LED_A0, false);
      set_gpio_state(LED_A1, true);
      set_gpio_state(LED_EN, true);
      lec_cnt = 0;
      for (int i = 0; i < param->led_light_delay_cnt; i++) {
        lec_cnt++;
      }
      adc2_get_raw(SEN_R45, width, &se->led_sen_after.right45.raw);
    }
    if (l45) { // L45
      set_gpio_state(LED_A0, true);
      set_gpio_state(LED_A1, true);
      set_gpio_state(LED_EN, true);
      lec_cnt = 0;
      for (int i = 0; i < param->led_light_delay_cnt; i++) {
        lec_cnt++;
      }
      adc2_get_raw(SEN_L45, width, &se->led_sen_after.left45.raw);
    }
  }

  end2 = esp_timer_get_time();
  set_gpio_state(LED_EN, false);

  // se->battery.data = linearInterpolation(x, y, se->battery.raw);
  // se->battery.data = linearInterpolation(x, y, se->battery.raw);

  se->battery.data = BATTERY_GAIN * 4 * sensing_result->battery.raw / 4096;
  if (led_on) {
    se->led_sen.right90.raw = std::max(
        se->led_sen_after.right90.raw - se->led_sen_before.right90.raw, 0);
    se->led_sen.right45.raw = std::max(
        se->led_sen_after.right45.raw - se->led_sen_before.right45.raw, 0);
    se->led_sen.left45.raw = std::max(
        se->led_sen_after.left45.raw - se->led_sen_before.left45.raw, 0);
    se->led_sen.left90.raw = std::max(
        se->led_sen_after.left90.raw - se->led_sen_before.left90.raw, 0);
    se->led_sen.front.raw =
        (se->led_sen.left90.raw + se->led_sen.right90.raw) / 2;
  } else {
    se->led_sen.right90.raw = 0;
    se->led_sen.right45.raw = 0;
    se->led_sen.left45.raw = 0;
    se->led_sen.left90.raw = 0;
    se->led_sen.front.raw = 0;
  }

  // gyro_if.req_read2byte_itr(0x26);
  se->gyro_list[4] = gyro_if.read_2byte_itr();
  se->gyro.raw = se->gyro_list[4];
  se->gyro.data = (float)(se->gyro_list[4]);
  // int32_t enc_r = (enc_if.read2byte(0x00, 0x00, true) & 0xFFFF) >> 2;

  now_enc_r_time = esp_timer_get_time();
  int32_t enc_r = enc_if.read2byte(0x3F, 0xFF, true) & 0x3FFF;

  se->encoder.right_old = se->encoder.right;
  se->encoder.right = enc_r;

  // int32_t enc_l = (enc_if.read2byte(0x00, 0x00, false) & 0xFFFF) >> 2;

  now_enc_l_time = esp_timer_get_time();
  int32_t enc_l = enc_if.read2byte(0x3F, 0xFF, false) & 0x3FFF;
  se->encoder.left_old = se->encoder.left;
  se->encoder.left = enc_l;
  end = esp_timer_get_time();
  se->calc_time2 = (int16_t)(end - start);
}