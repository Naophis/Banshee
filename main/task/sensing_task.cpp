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
std::vector<int> x = {2150, 2210, 2375, 2335, 2305, 2455, 2515, 2555, 2580};
std::vector<float> y = {7.2, 7.4, 7.6, 7.8, 8.0, 8.2, 8.4, 8.5, 8.6};

float linearInterpolation(const std::vector<int> &x,
                          const std::vector<float> &y, int x1) {
  // xとyの要素数が等しく、少なくとも2つ以上の要素が必要です
  // if (x.size() != y.size() || x.size() < 2) {
  //   throw std::runtime_error("Invalid input: x and y must have the same size
  //   "
  //                            "and at least two elements.");
  // }

  // x1がxの範囲外の場合、端点の値を返します
  if (x1 <= x[0]) {
    return y[0];
  }
  if (x1 >= x[x.size() - 1]) {
    return y[y.size() - 1];
  }

  // x1がxのどの範囲に含まれるかを見つけます
  int index = 0;
  while (x[index] < x1) {
    index++;
  }

  // x1がxの範囲内に含まれる場合、線形補間を行います
  float x0 = x[index - 1];
  float y0 = y[index - 1];
  float x2 = x[index];
  float y2 = y[index];

  return y0 + (y2 - y0) * (x1 - x0) / (x2 - x0);
}

void SensingTask::timer_10us_callback_main() {}

// 壁切れ時に必要ないセンシング処理をやめて、本来ほしいデータにまわす

void SensingTask::timer_200us_callback_main() {}

void SensingTask::timer_250us_callback_main() {}

void SensingTask::create_task(const BaseType_t xCoreID) {
  xTaskCreatePinnedToCore(task_entry_point, "sensing_task", 8192 * 2, this, 2,
                          &handle, xCoreID);
  // const esp_timer_create_args_t timer_200us_args = {
  //     .callback = &SensingTask::timer_200us_callback,
  //     .arg = this,
  //     .name = "timer_100us"};
  // esp_timer_create(&timer_200us_args, &timer_200us);

  // const esp_timer_create_args_t timer_10us_args = {
  //     .callback = &SensingTask::timer_10us_callback,
  //     .arg = this,
  //     .name = "timer_10us"};
  // esp_timer_create(&timer_10us_args, &timer_10us);
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

void SensingTask::set_planning_task(std::shared_ptr<PlanningTask> &_pt) {
  pt = _pt;
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

  const auto se = get_sensing_entity();
  // sensing init
  adc2_config_channel_atten(SEN_R90, atten);
  adc2_config_channel_atten(SEN_R45, atten);
  // adc2_config_channel_atten(SEN_R45_2, atten);
  adc2_config_channel_atten(SEN_L45, atten);
  // adc2_config_channel_atten(SEN_L45_2, atten);
  adc2_config_channel_atten(SEN_L90, atten);
  adc2_config_channel_atten(BATTERY, atten);

  // esp_timer_start_periodic(timer_200us, 200);
  // esp_timer_start_periodic(timer_250us, 250);

  int64_t start = 0;
  int64_t end = 0;
  int64_t start2 = 0;
  int64_t end2 = 0;
  int64_t start_before = 0;
  bool battery_check = true;
  bool skip_sensing = false;

  int64_t last_gyro_time = 0;
  int64_t now_gyro_time = 0;

  int64_t last_enc_r_time = 0;
  int64_t now_enc_r_time = 0;

  int64_t last_enc_l_time = 0;
  int64_t now_enc_l_time = 0;

  while (1) {

    last_gyro_time = now_gyro_time;
    last_enc_r_time = now_enc_r_time;
    last_enc_l_time = now_enc_l_time;

    skip_sensing = !skip_sensing;

    bool r90 = true;
    bool l90 = true;
    bool r45 = true;
    bool l45 = true;

    start_before = start;
    start = esp_timer_get_time();
    se->calc_time = (int16_t)(start - start_before);
    se->sensing_timestamp = start;
    const float tmp_dt = ((float)se->calc_time) / 1000000.0;
    now_gyro_time = esp_timer_get_time();
    const float gyro_dt = ((float)(now_gyro_time - last_gyro_time)) / 1000000.0;
    // gyro_if.req_read2byte_itr(0x26);
    start2 = esp_timer_get_time();

    if (skip_sensing) {
      adc2_get_raw(BATTERY, width, &sensing_result->battery.raw);
    }

    if (pt->search_mode && tgt_val->motion_type == MotionType::STRAIGHT) {
      // 加速中は正面は発光させない
      if (tgt_val->ego_in.state == 0) {
        r90 = l90 = false;
      }
    }
    if (pt->search_mode && tgt_val->nmr.sct == SensorCtrlType::NONE) {
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

    // LED_OFF ADC
    if (skip_sensing) {
      if (r90) {
        adc2_get_raw(SEN_R90, width,
                     &sensing_result->led_sen_before.right90.raw);
      } else {
        sensing_result->led_sen_before.right90.raw = 0;
      }
      adc2_get_raw(BATTERY, width, &sensing_result->battery.raw);
      if (l90) {
        adc2_get_raw(SEN_L90, width,
                     &sensing_result->led_sen_before.left90.raw);
      } else {
        sensing_result->led_sen_before.left90.raw = 0;
      }
    } else {

      if (r45) {
        adc2_get_raw(SEN_R45, width,
                     &sensing_result->led_sen_before.right45.raw);
      } else {
        sensing_result->led_sen_before.right45.raw = 0;
      }
      if (l45) {
        adc2_get_raw(SEN_L45, width,
                     &sensing_result->led_sen_before.left45.raw);
      } else {
        sensing_result->led_sen_before.left45.raw = 0;
      }
    }

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
    if (pt->mode_select) {
      led_on = false;
    }
    if (led_on) {
      if (pt->search_mode && pt->tgt_val->motion_type == MotionType::STRAIGHT) {
        // 加速中は正面は発光させない
        if (pt->tgt_val->ego_in.state == 0) {
          r90 = l90 = false;
        }
      }
      if (pt->search_mode && pt->tgt_val->nmr.sct == SensorCtrlType::NONE) {
        // 探索中、壁制御しないときはOFF
        r90 = l90 = false;
        r45 = l45 = false;
      }
      if (pt->tgt_val->nmr.sct == SensorCtrlType::Dia) {
        if (pt->tgt_val->ego_in.state == 0) {
          // 斜め壁制御加速中は横は発光させない
          r45 = l45 = false;
        }
      }
      if (pt->tgt_val->nmr.sct == SensorCtrlType::Straight) {
        r90 = l90 = true;
        r45 = l45 = true;
      }
      if (pt->tgt_val->motion_type == MotionType::READY) {
        // motion check用
        r90 = l90 = true;
        r45 = l45 = false;
      }
      if (pt->tgt_val->motion_type == MotionType::FRONT_CTRL) {
        // 前壁制御中は横は発光させない
        r90 = l90 = true;
        r45 = l45 = false;
      }
      if (pt->tgt_val->motion_type == MotionType::SENSING_DUMP) {
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
    // se->gyro_list[4] = gyro_if.read_2byte_itr();
    se->gyro_list[4] = gyro_if.read_2byte(0x26);
    se->gyro.raw = se->gyro_list[4];
    se->gyro.data = (float)(se->gyro_list[4]);
    // int32_t enc_r = (enc_if.read2byte(0x00, 0x00, true) & 0xFFFF) >> 2;
    now_enc_r_time = esp_timer_get_time();
    int32_t enc_r = enc_if.read2byte(0x3F, 0xFF, true) & 0x3FFF;
    const auto enc_r_dt =
        ((float)(now_enc_r_time - last_enc_r_time)) / 1000000.0;
    se->encoder.right_old = se->encoder.right;
    se->encoder.right = enc_r;

    // int32_t enc_l = (enc_if.read2byte(0x00, 0x00, false) & 0xFFFF) >> 2;
    now_enc_l_time = esp_timer_get_time();
    int32_t enc_l = enc_if.read2byte(0x3F, 0xFF, false) & 0x3FFF;
    const auto enc_l_dt =
        ((float)(now_enc_l_time - last_enc_l_time)) / 1000000.0;
    se->encoder.left_old = se->encoder.left;
    se->encoder.left = enc_l;

    calc_vel(gyro_dt, enc_l_dt, enc_r_dt);

    // cout << enc_r << ", " << enc_l << endl;
    end = esp_timer_get_time();
    se->calc_time2 = (int16_t)(end - start);
    // printf("sen: %d, %d\n", (int16_t)(end - start), (int16_t)(end2 -
    // start2));
    vTaskDelay(1.0 / portTICK_PERIOD_MS);
  }
}

float SensingTask::calc_sensor(float data, float a, float b) {
  auto res = a / std::log(data) - b;
  if (res < 5 || res > 180)
    return 180;
  return res;
}

void SensingTask::calc_vel(float gyro_dt, float enc_r_dt, float enc_l_dt) {
  // const float dt = param->dt;
  const float tire = pt->suction_en ? param->tire2 : param->tire;
  const auto enc_delta_l =
      sensing_result->encoder.left - sensing_result->encoder.left_old;
  float enc_ang_l = 0.f;
  if (ABS(enc_delta_l) < MIN(ABS(enc_delta_l - ENC_RESOLUTION),
                             ABS(enc_delta_l + ENC_RESOLUTION))) {
    enc_ang_l = 2 * m_PI * (float)enc_delta_l / (float)ENC_RESOLUTION;
  } else {
    if (ABS(enc_delta_l - ENC_RESOLUTION) < ABS(enc_delta_l + ENC_RESOLUTION)) {
      enc_ang_l = 2 * m_PI * (float)(enc_delta_l - ENC_RESOLUTION) /
                  (float)ENC_RESOLUTION;
    } else {
      enc_ang_l = 2 * m_PI * (float)(enc_delta_l + ENC_RESOLUTION) /
                  (float)ENC_RESOLUTION;
    }
  }

  const auto enc_delta_r =
      sensing_result->encoder.right - sensing_result->encoder.right_old;
  float enc_ang_r = 0.f;
  if (ABS(enc_delta_r) < MIN(ABS(enc_delta_r - ENC_RESOLUTION),
                             ABS(enc_delta_r + ENC_RESOLUTION))) {
    enc_ang_r = 2 * m_PI * (float)enc_delta_r / (float)ENC_RESOLUTION;
  } else {
    if (ABS(enc_delta_r - ENC_RESOLUTION) < ABS(enc_delta_r + ENC_RESOLUTION)) {
      enc_ang_r = 2 * m_PI * (float)(enc_delta_r - ENC_RESOLUTION) /
                  (float)ENC_RESOLUTION;
    } else {
      enc_ang_r = 2 * m_PI * (float)(enc_delta_r + ENC_RESOLUTION) /
                  (float)ENC_RESOLUTION;
    }
  }

  sensing_result->ego.v_l_old = sensing_result->ego.v_l;
  sensing_result->ego.v_r_old = sensing_result->ego.v_r;

  sensing_result->ego.v_l = tire * enc_ang_l / enc_l_dt / 2;
  sensing_result->ego.v_r = -tire * enc_ang_r / enc_r_dt / 2;

  sensing_result->ego.v_c =
      (sensing_result->ego.v_l + sensing_result->ego.v_r) / 2;

  sensing_result->ego.rpm.right =
      30.0 * sensing_result->ego.v_r / (m_PI * tire / 2);
  sensing_result->ego.rpm.left =
      30.0 * sensing_result->ego.v_l / (m_PI * tire / 2);

  if (tgt_val->motion_dir == MotionDirection::LEFT) {
    sensing_result->ego.w_raw =
        param->gyro_param.gyro_w_gain_left *
        (sensing_result->gyro.data - tgt_val->gyro_zero_p_offset);
  } else {
    sensing_result->ego.w_raw =
        param->gyro_param.gyro_w_gain_right *
        (sensing_result->gyro.data - tgt_val->gyro_zero_p_offset);
  }

  const auto dt = (enc_l_dt + enc_r_dt) / 2;

  tgt_val->ego_in.dist += sensing_result->ego.v_c * dt;
  tgt_val->global_pos.dist += sensing_result->ego.v_c * dt;
  tgt_val->ego_in.ang += sensing_result->ego.w_lp * gyro_dt;
  tgt_val->global_pos.ang += sensing_result->ego.w_lp * gyro_dt;
}