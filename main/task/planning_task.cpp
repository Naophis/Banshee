
#include "include/planning_task.hpp"

// constexpr int MOTOR_HZ = 250000;
// constexpr int MOTOR_HZ = 125000;
// constexpr int MOTOR_HZ = 100000;
constexpr unsigned long MOTOR_HZ = 75000 / 1;
// constexpr int MOTOR_HZ = 32500;
// constexpr int MOTOR_HZ = 17500;
// constexpr int MOTOR_HZ = 100000 / 1;
constexpr unsigned long SUCTION_MOTOR_HZ = 10000;
PlanningTask::PlanningTask() {}

PlanningTask::~PlanningTask() {}
void PlanningTask::create_task(const BaseType_t xCoreID) {
  xTaskCreatePinnedToCore(task_entry_point, "planning_task", 8192, this, 4,
                          &handle, xCoreID);
  motor_qh_enable = xQueueCreate(4, sizeof(motor_req_t *));
  suction_qh_enable = xQueueCreate(4, sizeof(motor_req_t *));
}

void PlanningTask::set_logging_task(std::shared_ptr<LoggingTask> &_lt) {
  lt = _lt; //
}
void PlanningTask::motor_enable_main() {
  motor_en = true;
  mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
  mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_1);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A,
                      MCPWM_DUTY_MODE_0);
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A,
                      MCPWM_DUTY_MODE_0);
}
void PlanningTask::set_gpio_state(gpio_num_t gpio_num, int state) {
  const int num = (int)gpio_num;
  if (num < 32) {
    if (state) {
      GPIO.out_w1ts = BIT(num);
    } else {
      GPIO.out_w1tc = BIT(num);
    }
  } else {
    if (state) {
      GPIO.out1_w1ts.val = BIT(num - 32);
    } else {
      GPIO.out1_w1tc.val = BIT(num - 32);
    }
  }
}

void PlanningTask::pid_gain_data(pid_error_t &in, pid_error_t &save) {
  // pid_val_data(in, save);
}

void PlanningTask::set_ctrl_val(pid_error2_t &val, float error_p, float error_i,
                                float error_i2, float error_d, float val_p,
                                float val_i, float val_i2, float val_d,
                                float zz, float z) {
  val.p = error_p;
  val.i = error_i;
  val.i2 = error_i2;
  val.d = error_d;
  val.p_val = val_p;
  val.i_val = val_i;
  val.i2_val = val_i2;
  val.d_val = val_d;
  val.zz = zz;
  val.z = z;
}
void PlanningTask::pid_val_data(pid_error_t &in, pid_error_t &save) {
  // error_entity_ptr->error_p = in.error_p;
  // error_entity_ptr->error_i = in.error_i;
  // error_entity_ptr->error_d = in.error_d;
}

float PlanningTask::interp1d(vector<float> &vx, vector<float> &vy, float x,
                             bool extrapolate) {
  int size = vx.size();
  int i = 0;
  if (x >= vx[size - 2]) {
    i = size - 2;
  } else {
    while (x > vx[i + 1])
      i++;
  }
  float xL = vx[i], yL = vy[i], xR = vx[i + 1], yR = vy[i + 1];
  if (!extrapolate) {
    if (x < xL)
      yR = yL;
    if (x > xR)
      yL = yR;
  }

  float dydx = (yR - yL) / (xR - xL);

  return yL + dydx * (x - xL);
}

void PlanningTask::motor_disable_main() {
  motor_en = false;
  gain_cnt = 0;
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0);
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B,
                      MCPWM_DUTY_MODE_0);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 0);
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B,
                      MCPWM_DUTY_MODE_0);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A,
                      MCPWM_DUTY_MODE_0);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A,
                      MCPWM_DUTY_MODE_0);
  mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
  mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);
  mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
  mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B);
  mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
  mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_1);
  gpio_set_level(Motor_L_PWM, 0);
  gpio_set_level(Motor_R_PWM, 0);
}

void PlanningTask::motor_enable() {
  motor_enable_send_msg.enable = true;
  motor_enable_send_msg.timestamp++;
  xQueueReset(motor_qh_enable);
  xQueueSendToFront(motor_qh_enable, &motor_enable_send_msg, 1);
}
void PlanningTask::motor_disable(bool reset_req) {
  motor_enable_send_msg.enable = false;
  motor_enable_send_msg.timestamp++;
  xQueueReset(motor_qh_enable);
  xQueueSendToFront(motor_qh_enable, &motor_enable_send_msg, 1);
}
void PlanningTask::motor_disable() {
  motor_disable(true); //
  vTaskDelay(1.0 / portTICK_PERIOD_MS);
}

void PlanningTask::suction_motor_enable_main() {
  suction_en = true;
  mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_A);
  mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_A,
                 std::abs(tgt_duty.duty_suction));
  mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_A,
                      MCPWM_DUTY_MODE_0);
  mcpwm_start(MCPWM_UNIT_1, MCPWM_TIMER_2);
  // ledc_channel_config(&suction_ch);
  // ledc_timer_config(&suction_timer);
}
void PlanningTask::suction_motor_disable_main() {
  suction_en = false;
  tgt_duty.duty_suction = 0;
  mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_A);
  mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_B);
  // mcpwm_stop(MCPWM_UNIT_1, MCPWM_TIMER_2); //
  mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_B, 0);
  mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_B,
                      MCPWM_DUTY_MODE_0);
  // ledc_set_duty(suction_ch.speed_mode, suction_ch.channel, 0);
  // ledc_update_duty(suction_ch.speed_mode, suction_ch.channel);
}

void PlanningTask::suction_enable(float duty, float duty2) {
  tgt_duty.duty_suction = duty;
  tgt_duty.duty_suction_low = duty2;
  suction_enable_send_msg.enable = true;
  suction_enable_send_msg.timestamp++;
  xQueueReset(suction_qh_enable);
  xQueueSendToFront(suction_qh_enable, &suction_enable_send_msg, 1);
}
void PlanningTask::suction_disable() {
  suction_enable_send_msg.enable = false;
  suction_enable_send_msg.timestamp++;
  xQueueReset(suction_qh_enable);
  xQueueSendToFront(suction_qh_enable, &suction_enable_send_msg, 1);
}
void PlanningTask::task_entry_point(void *task_instance) {
  static_cast<PlanningTask *>(task_instance)->task();
}

void PlanningTask::set_sensing_entity(
    std::shared_ptr<sensing_result_entity_t> &_sensing_result) {
  sensing_result = _sensing_result;
}

void PlanningTask::set_input_param_entity(
    std::shared_ptr<input_param_t> &_param_ro) {
  param_ro = _param_ro;
}

void PlanningTask::set_tgt_val(std::shared_ptr<motion_tgt_val_t> &_tgt_val) {
  tgt_val = _tgt_val;
}
void PlanningTask::set_error_entity(
    std::shared_ptr<pid_error_entity_t> &_error_entity) {
  error_entity_ptr = _error_entity;
}
void PlanningTask::copy_error_entity(pid_error_entity_t &in) {
  error_entity_ptr->v_val.p = in.v_val.p;
  error_entity_ptr->v_val.i = in.v_val.i;
  error_entity_ptr->v_val.i2 = in.v_val.i2;
  error_entity_ptr->v_val.d = in.v_val.d;
  error_entity_ptr->v_val.p_val = in.v_val.p_val;
  error_entity_ptr->v_val.i_val = in.v_val.i_val;
  error_entity_ptr->v_val.i2_val = in.v_val.i2_val;
  error_entity_ptr->v_val.d_val = in.v_val.d_val;
  error_entity_ptr->v_val.zz = in.v_val.zz;
  error_entity_ptr->v_val.z = in.v_val.z;

  error_entity_ptr->w_val.p = in.w_val.p;
  error_entity_ptr->w_val.i = in.w_val.i;
  error_entity_ptr->w_val.i2 = in.w_val.i2;
  error_entity_ptr->w_val.d = in.w_val.d;
  error_entity_ptr->w_val.p_val = in.w_val.p_val;
  error_entity_ptr->w_val.i_val = in.w_val.i_val;
  error_entity_ptr->w_val.i2_val = in.w_val.i2_val;
  error_entity_ptr->w_val.d_val = in.w_val.d_val;
  error_entity_ptr->w_val.zz = in.w_val.zz;
  error_entity_ptr->w_val.z = in.w_val.z;

  error_entity_ptr->s_val.p = in.s_val.p;
  error_entity_ptr->s_val.i = in.s_val.i;
  error_entity_ptr->s_val.i2 = in.s_val.i2;
  error_entity_ptr->s_val.d = in.s_val.d;
  error_entity_ptr->s_val.p_val = in.s_val.p_val;
  error_entity_ptr->s_val.i_val = in.s_val.i_val;
  error_entity_ptr->s_val.i2_val = in.s_val.i2_val;
  error_entity_ptr->s_val.d_val = in.s_val.d_val;
  error_entity_ptr->s_val.zz = in.s_val.zz;
  error_entity_ptr->s_val.z = in.s_val.z;
}
void PlanningTask::active_logging(FILE *_f) {
  log_active = true;
  log_list2_size = 0;
}
void PlanningTask::inactive_logging() { log_active = false; }

void PlanningTask::buzzer(ledc_channel_config_t &buzzer_ch,
                          ledc_timer_config_t &buzzer_timer) {
  int duty = 0;
  bool buzzer = false;
  if (motor_en || suction_en) {
    return;
  }
  if (buzzer_timestamp != tgt_val->buzzer.timstamp) {
    buzzer_time_cnt = 0;
    buzzer_timestamp = tgt_val->buzzer.timstamp;
    buzzer_timer.freq_hz = tgt_val->buzzer.hz;
    ledc_channel_config(&buzzer_ch);
    ledc_timer_config(&buzzer_timer);
  }
  if (buzzer_time_cnt < tgt_val->buzzer.time) {
    duty = 50;
    buzzer_time_cnt++;
    buzzer = true;
  } else if (buzzer_time_cnt >= tgt_val->buzzer.time &&
             buzzer_time_cnt < tgt_val->buzzer.time + 10) {
    duty = 0;
    buzzer_time_cnt++;
    buzzer = true;
  }
  if (buzzer) {
    ledc_set_duty(buzzer_ch.speed_mode, buzzer_ch.channel, duty);
    ledc_update_duty(buzzer_ch.speed_mode, buzzer_ch.channel);
  }
}
void PlanningTask::calc_filter() {
  const auto alpha = param_ro->comp_param.accl_x_hp_gain;
  sensing_result->ego.filter_v = //
      alpha * (sensing_result->ego.filter_v +
               sensing_result->ego.accel_x_raw * dt) + //
      (1 - alpha) * sensing_result->ego.v_c;
  if (!std::isfinite(sensing_result->ego.filter_v)) {
    sensing_result->ego.filter_v = sensing_result->ego.v_c;
  }
}

void PlanningTask::set_motor_hz(unsigned long hz, int res) {
  const unsigned long int resolution = ((unsigned long int)res) * 100'000L;
  mcpwm_group_set_resolution(MCPWM_UNIT_0, resolution);

  memset(&motor_pwm_conf, 0, sizeof(motor_pwm_conf));
  motor_pwm_conf.frequency = hz; // PWM周波数= 10kHz,
  motor_pwm_conf.cmpr_a = 0; // デューティサイクルの初期値（0%）
  motor_pwm_conf.cmpr_b = 0; // デューティサイクルの初期値（0%）
  motor_pwm_conf.counter_mode = MCPWM_UP_COUNTER;
  motor_pwm_conf.duty_mode = MCPWM_DUTY_MODE_0; // アクティブハイ
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &motor_pwm_conf);
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &motor_pwm_conf);

  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, Motor_L_PWM);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, Motor_R_PWM);
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A,
                      MCPWM_DUTY_MODE_0);
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A,
                      MCPWM_DUTY_MODE_0);

  // 2PWM input mode configuration
  if (param_ro->motor_driver_type == MotorDriveType::TWO_PWM) {
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, Motor_L_PWM2);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, Motor_R_PWM2);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B,
                        MCPWM_DUTY_MODE_0);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B,
                        MCPWM_DUTY_MODE_0);
  }
}
void PlanningTask::set_suction_motor_hz(unsigned long hz, int res) {
  const unsigned long int resolution = ((unsigned long int)res) * 100'000L;
  mcpwm_group_set_resolution(MCPWM_UNIT_1, resolution);
  memset(&suction_pwm_conf, 0, sizeof(suction_pwm_conf));
  suction_pwm_conf.frequency = hz; // PWM周波数= 10kHz,
  suction_pwm_conf.cmpr_a = 0; // デューティサイクルの初期値（0%）
  suction_pwm_conf.cmpr_b = 0; // デューティサイクルの初期値（0%）
  suction_pwm_conf.counter_mode = MCPWM_UP_COUNTER;
  suction_pwm_conf.duty_mode = MCPWM_DUTY_MODE_0; // アクティブハイ
  suction_pwm_conf.cmpr_a = 0; // デューティサイクルの初期値（0%）
  mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_2, &suction_pwm_conf);
}

void PlanningTask::task() {
  int64_t start = 0;
  int64_t start2 = 0;
  int64_t start_before = 0;
  int64_t end = 0;
  int64_t start_calc_mpc = 0;
  int64_t end_calc_mpc = 0;
  int64_t start_que_rec = 0;
  int64_t end_que_rec = 0;
  // int64_t start2;
  // int64_t end2;
  // int64_t start2;
  // int64_t end2;
  const TickType_t xDelay = 1.0 / portTICK_PERIOD_MS;
  BaseType_t queue_recieved;
  init_gpio();
  for (int i = 0; i < 4097; i++) {
    log_table.emplace_back(std::log(i));
  }
  memset(&buzzer_ch, 0, sizeof(buzzer_ch));
  memset(&buzzer_timer, 0, sizeof(buzzer_timer));
  buzzer_ch.channel = (ledc_channel_t)LEDC_CHANNEL_0;
  buzzer_ch.duty = 0;
  buzzer_ch.gpio_num = BUZZER;
  buzzer_ch.speed_mode = (ledc_mode_t)LEDC_HIGH_SPEED_MODE;
  buzzer_ch.timer_sel = (ledc_timer_t)LEDC_TIMER_0;

  buzzer_timer.duty_resolution = (ledc_timer_bit_t)LEDC_TIMER_10_BIT;
  buzzer_timer.freq_hz = 440;
  buzzer_timer.speed_mode = (ledc_mode_t)LEDC_HIGH_SPEED_MODE; // timer mode
  buzzer_timer.timer_num = (ledc_timer_t)LEDC_TIMER_0;         // timer index
  ledc_channel_config(&buzzer_ch);
  ledc_timer_config(&buzzer_timer);

  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM2A, SUCTION_PWM);
  mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_A,
                      MCPWM_DUTY_MODE_0);

  set_motor_hz(MOTOR_HZ, 10);
  set_suction_motor_hz(SUCTION_MOTOR_HZ, 10);

  motor_en = false;
  set_next_duty(0, 0, 0);
  gpio_set_level(SUCTION_PWM, 0);
  mpc_tgt_calc.initialize();
  vel_pid.initialize();
  gyro_pid.initialize();
  ang_pid.initialize();

  float initial_state = 0.0;        // 初期姿勢の推定値
  float initial_covariance = 0.950; // 初期姿勢の誤差共分散行列
  float process_noise = 0.05;       // プロセスノイズの共分散
  float measurement_noise = 0.35;   // 観測ノイズの共分散

  kf_w.init(initial_state, initial_covariance, process_noise,
            measurement_noise);
  kf_v.init(initial_state, initial_covariance, process_noise,
            measurement_noise);
  kf_batt.init(initial_state, initial_covariance, process_noise,
               measurement_noise);
  kf_dist.init(initial_state, initial_covariance, process_noise,
               measurement_noise);
  kf_ang.init(initial_state, initial_covariance, process_noise,
              measurement_noise);
  // dist_pid.initialize();
  // sen_pid.initialize();
  // sen_dia_pid.initialize();
  // angle_pid.initialize();
  // vel_pid_2dof.initialize();
  // gyro_pid_2dof.initialize();
  enc_v_q.clear();
  accl_x_q.clear();
  ready = false;

  while (1) {
    start_before = start;
    start = esp_timer_get_time();
    tgt_val->calc_time_diff = start - start_before;

    if (!ready) {
      vTaskDelay(xDelay);
    }

    // sensing
    if (xQueueReceive(motor_qh_enable, &motor_enable_send_msg, 0) == pdTRUE) {
      if (motor_req_timestamp != motor_enable_send_msg.timestamp) {
        if (motor_enable_send_msg.enable) {
          motor_enable_main();
        } else {
          motor_disable_main();
        }
        motor_req_timestamp = motor_enable_send_msg.timestamp;
      }
    }
    if (xQueueReceive(suction_qh_enable, &suction_enable_send_msg, 0) ==
        pdTRUE) {
      if (suction_req_timestamp != suction_enable_send_msg.timestamp) {
        if (suction_enable_send_msg.enable) {
          suction_motor_enable_main();
        } else {
          suction_motor_disable_main();
        }
        suction_req_timestamp = suction_enable_send_msg.timestamp;
      }
    }

    // 自己位置更新
    int64_t start_duty_cal = esp_timer_get_time();
    update_ego_motion(); // 30 usec
    int64_t end_duty_cal = esp_timer_get_time();
    calc_sensor_dist_all(); // 15 ~ 20 usec

    mpc_step = 1;
    tgt_val->tgt_in.time_step2 = param_ro->sakiyomi_time;

    auto res = xQueueReceive(*qh, &receive_req, 0);

    if (res == pdTRUE) {
      cp_request();
      first_req = true;
    }

    // sn->task(search_mode, mode_select, res == pdTRUE);
    start2 = esp_timer_get_time();

    // 物理量ベース計算
    float axel_degenerate_gain = 1;
    diff_old = diff;
    if (!search_mode && tgt_val->motion_type == MotionType::STRAIGHT) {
      // if (tgt_val->motion_type == MotionType::STRAIGHT) {
      if (axel_degenerate_x.size() > 0 &&
          tgt_val->nmr.sct == SensorCtrlType::Straight) {
        // if (tgt_val->nmr.sct == SensorCtrlType::Straight) {
        diff = ABS(check_sen_error());
        // } else if (tgt_val->nmr.sct == SensorCtrlType::Dia) {
        //   diff = ABS(check_sen_error_dia());
        // }
        if (diff == 0) {
          diff = diff_old;
        }
        axel_degenerate_gain =
            interp1d(axel_degenerate_x, axel_degenerate_y, diff, false);
        tgt_val->tgt_in.axel_degenerate_gain =
            (1 - param_ro->sensor_gain.front2.b) *
                tgt_val->tgt_in.axel_degenerate_gain +
            param_ro->sensor_gain.front2.b * axel_degenerate_gain;
      }
    } else {
      diff = diff_old = 0;
      tgt_val->tgt_in.axel_degenerate_gain = axel_degenerate_gain;
    }

    start_calc_mpc = esp_timer_get_time();
    if (first_req) {
      mpc_tgt_calc.step(&tgt_val->tgt_in, &tgt_val->ego_in,
                        tgt_val->motion_mode, mpc_step, &mpc_next_ego,
                        &dynamics);
    }
    end_calc_mpc = esp_timer_get_time();
    // check 3

    if (tgt_val->motion_type == MotionType::STRAIGHT ||
        tgt_val->motion_type == MotionType::SLA_FRONT_STR ||
        tgt_val->motion_type == MotionType::SLA_BACK_STR) {
      if (tgt_val->ego_in.img_dist >= tgt_val->tgt_in.tgt_dist) {
        mpc_next_ego.v = tgt_val->tgt_in.end_v;
      }
      if (tgt_val->ego_in.dist >= tgt_val->tgt_in.tgt_dist) {
        mpc_next_ego.v = tgt_val->tgt_in.end_v;
      }
    }

    // 算出結果をコピー
    cp_tgt_val(); // 1~2usec

    // Duty計算
    calc_tgt_duty(); // 15 ~ 20 usec

    check_fail_safe(); // 7 ~ 9 usec

    // chekc 4
    // 22 ~ 25 usec
    set_next_duty(tgt_duty.duty_l, tgt_duty.duty_r, tgt_duty.duty_suction);

    buzzer(buzzer_ch, buzzer_timer);
    global_msec_timer++;

    end = esp_timer_get_time();
    tgt_val->calc_time = (int16_t)(end - start2);
    vTaskDelay(xDelay);
  }
}

float PlanningTask::calc_sensor_pid() {
  float duty = 0;

  error_entity.sen.error_i += error_entity.sen.error_p;
  error_entity.sen.error_d = error_entity.sen.error_p;
  error_entity.sen.error_p = check_sen_error();
  if (search_mode) {
    if (error_entity.sen.error_p > param_ro->search_sen_ctrl_limitter) {
      error_entity.sen.error_p = param_ro->search_sen_ctrl_limitter;
    } else if (error_entity.sen.error_p < -param_ro->search_sen_ctrl_limitter) {
      error_entity.sen.error_p = -param_ro->search_sen_ctrl_limitter;
    }
  }
  error_entity.sen.error_d =
      error_entity.sen.error_p - error_entity.sen.error_d;

  if (search_mode) {
    if (error_entity.sen.error_p != 0) {
      duty = param_ro->str_ang_pid.p * error_entity.sen.error_p -
             param_ro->str_ang_pid.d * sensing_result->ego.w_kf;

      //  (error_entity.sen_log.gain_z - error_entity.sen_log.gain_zz) * dt;
      set_ctrl_val(error_entity.s_val, error_entity.sen.error_p, 0, 0,
                   sensing_result->ego.w_kf,
                   param_ro->str_ang_pid.p * error_entity.sen.error_p, 0, 0,
                   -param_ro->str_ang_pid.d * sensing_result->ego.w_kf,
                   error_entity.sen_log.gain_zz, error_entity.sen_log.gain_z);
      error_entity.sen_log.gain_zz = error_entity.sen_log.gain_z;
      error_entity.sen_log.gain_z = duty;
    } else {
      duty = 0;
      error_entity.sen_log.gain_zz = error_entity.sen_log.gain_z;
      error_entity.sen_log.gain_z = duty;
      set_ctrl_val(error_entity.s_val, 0, 0, 0, 0, 0, 0, 0, 0,
                   error_entity.sen_log.gain_zz, error_entity.sen_log.gain_z);
    }
  } else {

    if (error_entity.sen.error_p != 0) {
      duty = param_ro->str_ang_pid.b * error_entity.sen.error_p -
             param_ro->str_ang_pid.d * sensing_result->ego.w_kf;

      error_entity.sen_log.gain_zz = error_entity.sen_log.gain_z;
      error_entity.sen_log.gain_z = duty;
      set_ctrl_val(error_entity.s_val, error_entity.sen.error_p, 0, 0,
                   sensing_result->ego.w_kf,
                   param_ro->str_ang_pid.b * error_entity.sen.error_p, 0, 0,
                   -param_ro->str_ang_pid.d * sensing_result->ego.w_kf,
                   error_entity.sen_log.gain_zz, error_entity.sen_log.gain_z);
    } else {
      duty = 0;
      error_entity.sen_log.gain_zz = error_entity.sen_log.gain_z;
      error_entity.sen_log.gain_z = duty;
      set_ctrl_val(error_entity.s_val, 0, 0, 0, 0, 0, 0, 0, 0,
                   error_entity.sen_log.gain_zz, error_entity.sen_log.gain_z);
    }
  }

  // if (search_mode) {
  //   duty = param_ro->sensor_pid.p * error_entity.sen.error_p +
  //          param_ro->sensor_pid.i * error_entity.sen.error_i +
  //          param_ro->sensor_pid.d * error_entity.sen.error_d +
  //          (error_entity.sen_log.gain_z - error_entity.sen_log.gain_zz) * dt;
  //   error_entity.sen_log.gain_zz = error_entity.sen_log.gain_z;
  //   error_entity.sen_log.gain_z = duty;
  // } else {
  //   duty = param_ro->sensor_pid.b * error_entity.sen.error_p +
  //          param_ro->sensor_pid.i * error_entity.sen.error_i +
  //          param_ro->sensor_pid.d * error_entity.sen.error_d +
  //          (error_entity.sen_log.gain_z - error_entity.sen_log.gain_zz) * dt;
  //   error_entity.sen_log.gain_zz = error_entity.sen_log.gain_z;
  //   error_entity.sen_log.gain_z = duty;
  // }

  if (search_mode) {
    if (duty > param_ro->sensor_gain.front.a) {
      duty = param_ro->sensor_gain.front.a;
    } else if (duty < -param_ro->sensor_gain.front.a) {
      duty = -param_ro->sensor_gain.front.a;
    }
  } else {
    if (duty > param_ro->sensor_gain.front2.a) {
      duty = param_ro->sensor_gain.front2.a;
    } else if (duty < -param_ro->sensor_gain.front2.a) {
      duty = -param_ro->sensor_gain.front2.a;
    }
  }

  return duty;
}
float PlanningTask::calc_sensor_pid_dia() {
  float duty = 0;

  error_entity.sen_dia.error_i += error_entity.sen_dia.error_p;
  error_entity.sen_dia.error_d = error_entity.sen_dia.error_p;
  error_entity.sen_dia.error_p = check_sen_error_dia();
  error_entity.sen_dia.error_d =
      error_entity.sen_dia.error_p - error_entity.sen_dia.error_d;
  if (param_ro->sensor_pid_dia.mode == 1) {
    duty =
        param_ro->sensor_pid_dia.p * error_entity.sen_dia.error_p +
        param_ro->sensor_pid_dia.i * error_entity.sen_dia.error_i +
        param_ro->sensor_pid_dia.d * error_entity.sen_dia.error_d +
        (error_entity.sen_log_dia.gain_z - error_entity.sen_log_dia.gain_zz) *
            dt;
    error_entity.sen_log_dia.gain_zz = error_entity.sen_log_dia.gain_z;
    error_entity.sen_log_dia.gain_z = duty;

    set_ctrl_val(error_entity.s_val, error_entity.sen_dia.error_p, 0, 0,
                 error_entity.sen_dia.error_d,
                 param_ro->sensor_pid_dia.p * error_entity.sen_dia.error_p,
                 param_ro->sensor_pid_dia.i * error_entity.sen_dia.error_i, 0,
                 param_ro->sensor_pid_dia.d * error_entity.sen_dia.error_d,
                 error_entity.sen_log_dia.gain_zz,
                 error_entity.sen_log_dia.gain_z);
  } else {
    duty = param_ro->sensor_pid_dia.p * error_entity.sen_dia.error_p +
           param_ro->sensor_pid_dia.i * error_entity.sen_dia.error_i +
           param_ro->sensor_pid_dia.d * error_entity.sen_dia.error_d;

    set_ctrl_val(error_entity.s_val, error_entity.sen_dia.error_p, 0, 0,
                 error_entity.sen_dia.error_d,
                 param_ro->sensor_pid_dia.p * error_entity.sen_dia.error_p,
                 param_ro->sensor_pid_dia.i * error_entity.sen_dia.error_i, 0,
                 param_ro->sensor_pid_dia.d * error_entity.sen_dia.error_d, 0,
                 0);
  }
  // const unsigned char enable = 1;
  // sen_dia_pid.step(&error_entity.sen_dia.error_p,
  // &param_ro->sensor_pid_dia.p,
  //                  &param_ro->sensor_pid_dia.i, &param_ro->sensor_pid_dia.d,
  //                  &enable, &dt, &duty);
  // if (duty > param_ro->sensor_gain.front.b) {
  //   duty = param_ro->sensor_gain.front.b; // 3degまで
  // } else if (duty < -param_ro->sensor_gain.front.b) {
  //   duty = -param_ro->sensor_gain.front.b;
  // }
  duty = 0;
  if (error_entity.sen_dia.error_p > param_ro->sensor_gain.front3.a) {
    duty = param_ro->sensor_gain.front3.b * m_PI / 180; // 3degまで
  } else if (error_entity.sen_dia.error_p < -param_ro->sensor_gain.front3.a) {
    duty = -param_ro->sensor_gain.front3.b * m_PI / 180;
  }

  return duty;
}
float PlanningTask::check_sen_error() {
  float error = 0;
  int check = 0;
  float dist_mod = (int)(tgt_val->ego_in.dist / param_ro->dist_mod_num);
  float tmp_dist = tgt_val->ego_in.dist - param_ro->dist_mod_num * dist_mod;

  if (tmp_dist < 10) {
    sensing_result->ego.exist_r_wall = sensing_result->ego.right45_dist <
                                       param_ro->sen_ref_p.search_exist.right45;
    sensing_result->ego.exist_l_wall = sensing_result->ego.left45_dist <
                                       param_ro->sen_ref_p.search_exist.left45;
  }

  auto exist_right45 = param_ro->sen_ref_p.normal.exist.right45;
  auto exist_left45 = param_ro->sen_ref_p.normal.exist.left45;
  auto exist_right45_2 = param_ro->sen_ref_p.normal2.exist.right45;
  auto exist_left45_2 = param_ro->sen_ref_p.normal2.exist.left45;

  //前壁が近すぎるときはエスケープ
  if (!(10 < sensing_result->ego.left90_mid_dist &&
        sensing_result->ego.left90_mid_dist <
            param_ro->sen_ref_p.normal.exist.front &&
        10 < sensing_result->ego.right90_mid_dist &&
        sensing_result->ego.right90_mid_dist <
            param_ro->sen_ref_p.normal.exist.front)) {
    if (std::abs(sensing_result->ego.right45_dist -
                 sensing_result->ego.right45_dist_old) <
        param_ro->sen_ref_p.normal.ref.kireme_r) {
      if ((1 < sensing_result->ego.right45_dist &&
           sensing_result->ego.right45_dist < exist_right45)) {
        if (search_mode) {
          error += param_ro->sen_ref_p.normal.ref.right45 -
                   sensing_result->ego.right45_dist;
          check++;
        } else {
          if (ABS(tgt_val->global_pos.dist - right_keep.star_dist) >
              param_ro->right_keep_dist_th) {
            error += param_ro->sen_ref_p.normal.ref.right45 -
                     sensing_result->ego.right45_dist;
            check++;
          }
        }
      } else {
        right_keep.star_dist = tgt_val->global_pos.dist;
      }
    } else {
      right_keep.star_dist = tgt_val->global_pos.dist;
    }
    if (std::abs(sensing_result->ego.left45_dist -
                 sensing_result->ego.left45_dist_old) <
        param_ro->sen_ref_p.normal.ref.kireme_l) {
      if ((1 < sensing_result->ego.left45_dist &&
           sensing_result->ego.left45_dist < exist_left45)) {
        if (search_mode) {
          error -= param_ro->sen_ref_p.normal.ref.left45 -
                   sensing_result->ego.left45_dist;
          check++;
        } else {
          if (ABS(tgt_val->global_pos.dist - left_keep.star_dist) >
              param_ro->left_keep_dist_th) {
            error -= param_ro->sen_ref_p.normal.ref.left45 -
                     sensing_result->ego.left45_dist;
            check++;
          }
        }
      } else {
        left_keep.star_dist = tgt_val->global_pos.dist;
      }
    } else {
      left_keep.star_dist = tgt_val->global_pos.dist;
    }
  }
  if (check == 0) {
    error_entity.sen.error_i = 0;
    error_entity.sen_log.gain_zz = 0;
    error_entity.sen_log.gain_z = 0;

    if (!(10 < sensing_result->ego.left90_mid_dist &&
          sensing_result->ego.left90_mid_dist <
              param_ro->sen_ref_p.normal.exist.front &&
          10 < sensing_result->ego.right90_mid_dist &&
          sensing_result->ego.right90_mid_dist <
              param_ro->sen_ref_p.normal.exist.front)) {
      if (sensing_result->ego.right45_dist >
              param_ro->sen_ref_p.normal2.ref.kireme_r &&
          sensing_result->ego.left45_dist >
              param_ro->sen_ref_p.normal2.ref.kireme_l) {
        if ((1 < sensing_result->sen.r45.sensor_dist &&
             sensing_result->sen.r45.sensor_dist < exist_right45_2)) {
          error += param_ro->sen_ref_p.normal2.ref.right45 -
                   sensing_result->sen.r45.sensor_dist;
          check++;
        }
      }
      if (sensing_result->ego.right45_dist >
              param_ro->sen_ref_p.normal2.ref.kireme_r &&
          sensing_result->ego.left45_dist >
              param_ro->sen_ref_p.normal2.ref.kireme_l) {
        if ((1 < sensing_result->sen.l45.sensor_dist &&
             sensing_result->sen.l45.sensor_dist < exist_left45_2)) {
          error -= param_ro->sen_ref_p.normal2.ref.left45 -
                   sensing_result->sen.l45.sensor_dist;
          check++;
        }
      }
      error *= param_ro->sen_ref_p.normal2.exist.front;
    }
  } else {
    // TODO Uターン字は別ロジックに修正
    if (tgt_val->tgt_in.tgt_dist >= param_ro->clear_dist_order) {
      if (!(param_ro->clear_dist_ragne_from <= tmp_dist &&
            tmp_dist <= param_ro->clear_dist_ragne_to)) {
        if ((std::abs(tgt_val->ego_in.ang - tgt_val->ego_in.img_ang) * 180 /
             m_PI) < param_ro->clear_angle) {
          tgt_val->global_pos.ang = tgt_val->global_pos.img_ang;
          error_entity.w.error_i = 0;
          error_entity.w.error_d = 0;
          error_entity.w.error_dd = 0;
          error_entity.ang.error_i = 0;
          error_entity.ang.error_d = 0;
          error_entity.ang.error_dd = 0;
          w_reset = 0;
        }
      } else {
        // error_entity.sen.error_i = 0;
        // error_entity.sen_log.gain_zz = 0;
        // error_entity.sen_log.gain_z = 0;
        // if (check == 2) {
        //   return error;
        // } else if (check == 1) {
        //   return error * 2;
        // }
      }
    }
  }

  if (check == 2) {
    return error;
  } else if (check == 1) {
    return error * 2;
  }
  error_entity.sen.error_i = 0;
  error_entity.sen_log.gain_zz = 0;
  error_entity.sen_log.gain_z = 0;
  return 0;
}
float PlanningTask::check_sen_error_dia() {
  float error = 0;
  int check = 0;

  //前壁が近すぎるときはエスケープ
  if (tgt_val->tgt_in.tgt_dist > param_ro->sen_ctrl_front_th &&
      (tgt_val->tgt_in.tgt_dist - tgt_val->ego_in.dist) >
          param_ro->sen_ctrl_front_diff_th) {
    // if (std::abs(sensing_result->ego.right90_dist -
    //         sensing_result->ego.right90_dist_old) <
    //     param_ro->sen_ref_p.dia.ref.kireme_r) {
    if (1 < sensing_result->ego.right90_mid_dist &&
        sensing_result->ego.right90_mid_dist <
            param_ro->sen_ref_p.dia.exist.right90) {
      error += param_ro->sen_ref_p.dia.ref.right90 -
               sensing_result->ego.right90_mid_dist;

      tgt_val->dia_state.right_old = param_ro->sen_ref_p.dia.ref.right90 -
                                     sensing_result->ego.right90_mid_dist;
      tgt_val->dia_state.right_save = true;

      check++;
    } else {
      if (param_ro->sensor_gain.front4.a != 0) {
        if (tgt_val->dia_state.right_save) {
          error += tgt_val->dia_state.right_old;
          check++;
        }
      }
    }
    // }
    // if (std::abs(sensing_result->ego.left90_dist -
    //         sensing_result->ego.left90_dist_old) <
    //     param_ro->sen_ref_p.dia.ref.kireme_l) {
    if (1 < sensing_result->ego.left90_mid_dist &&
        sensing_result->ego.left90_mid_dist <
            param_ro->sen_ref_p.dia.exist.left90) {
      error -= param_ro->sen_ref_p.dia.ref.left90 -
               sensing_result->ego.left90_mid_dist;
      tgt_val->dia_state.left_old = param_ro->sen_ref_p.dia.ref.left90 -
                                    sensing_result->ego.left90_mid_dist;
      tgt_val->dia_state.left_save = true;
      check++;
    } else {
      if (param_ro->sensor_gain.front4.a != 0) {
        if (tgt_val->dia_state.left_save) {
          error -= tgt_val->dia_state.left_old;
          check++;
        }
      }
    }
  }
  // }
  if (check == 0) {
    error_entity.sen_dia.error_i = 0;
    error_entity.sen_log_dia.gain_zz = 0;
    error_entity.sen_log_dia.gain_z = 0;
  } else {
    // TODO Uターン字は別ロジックに修正
    // error_entity.sen_dia.error_i = 0;
    // error_entity.sen_log_dia.gain_zz = 0;
    // error_entity.sen_log_dia.gain_z = 0;
    if (tgt_val->tgt_in.tgt_dist >= param_ro->clear_dist_order) {
      if ((std::abs(tgt_val->ego_in.ang - tgt_val->ego_in.img_ang) * 180 /
           m_PI) < param_ro->clear_angle) {
        // tgt_val->global_pos.ang = tgt_val->global_pos.img_ang;
        // error_entity.w.error_i = 0;
        // error_entity.w.error_d = 0;
        // error_entity.ang.error_i = 0;
        // error_entity.ang.error_d = 0;
      } else {
        // error_entity.sen.error_i = 0;
        // error_entity.sen_log.gain_zz = 0;
        // error_entity.sen_log.gain_z = 0;
        // if (check == 2) {
        //   return error;
        // } else if (check == 1) {
        //   return error * 2;
        // }
      }
    }
  }

  if (check == 2) {
    return error;
  } else if (check == 1) {
    return error * 2;
  }
  return 0;
}

void PlanningTask::calc_vel() {
  // const float dt = param_ro->dt;
  // const float tire = suction_en ? param_ro->tire2 : param_ro->tire;
  // const auto enc_delta_l =
  //     sensing_result->encoder.left - sensing_result->encoder.left_old;
  // float enc_ang_l = 0.f;
  // if (ABS(enc_delta_l) < MIN(ABS(enc_delta_l - ENC_RESOLUTION),
  //                            ABS(enc_delta_l + ENC_RESOLUTION))) {
  //   enc_ang_l = 2 * m_PI * (float)enc_delta_l / (float)ENC_RESOLUTION;
  // } else {
  //   if (ABS(enc_delta_l - ENC_RESOLUTION) < ABS(enc_delta_l +
  //   ENC_RESOLUTION)) {
  //     enc_ang_l = 2 * m_PI * (float)(enc_delta_l - ENC_RESOLUTION) /
  //                 (float)ENC_RESOLUTION;
  //   } else {
  //     enc_ang_l = 2 * m_PI * (float)(enc_delta_l + ENC_RESOLUTION) /
  //                 (float)ENC_RESOLUTION;
  //   }
  // }

  // const auto enc_delta_r =
  //     sensing_result->encoder.right - sensing_result->encoder.right_old;
  // float enc_ang_r = 0.f;
  // if (ABS(enc_delta_r) < MIN(ABS(enc_delta_r - ENC_RESOLUTION),
  //                            ABS(enc_delta_r + ENC_RESOLUTION))) {
  //   enc_ang_r = 2 * m_PI * (float)enc_delta_r / (float)ENC_RESOLUTION;
  // } else {
  //   if (ABS(enc_delta_r - ENC_RESOLUTION) < ABS(enc_delta_r +
  //   ENC_RESOLUTION)) {
  //     enc_ang_r = 2 * m_PI * (float)(enc_delta_r - ENC_RESOLUTION) /
  //                 (float)ENC_RESOLUTION;
  //   } else {
  //     enc_ang_r = 2 * m_PI * (float)(enc_delta_r + ENC_RESOLUTION) /
  //                 (float)ENC_RESOLUTION;
  //   }
  // }

  // sensing_result->ego.v_l_old = sensing_result->ego.v_l;
  // sensing_result->ego.v_r_old = sensing_result->ego.v_r;

  // sensing_result->ego.v_l = tire * enc_ang_l / dt / 2;
  // sensing_result->ego.v_r = -tire * enc_ang_r / dt / 2;

  // sensing_result->ego.v_c =
  //     (sensing_result->ego.v_l + sensing_result->ego.v_r) / 2;
}

void PlanningTask::update_ego_motion() {
  const float dt = param_ro->dt;
  const float tire = param_ro->tire;
  tgt_val->ego_in.ff_duty_low_th = param_ro->ff_front_dury;
  tgt_val->ego_in.ff_duty_low_v_th = param_ro->ff_v_th;
  if (!motor_en) {
    tgt_val->ego_in.v = 0;
    tgt_val->ego_in.w = 0;
  }
  sensing_result->ego.accel_x_raw =
      param_ro->accel_x_param.gain *
      (sensing_result->accel_x.raw - tgt_val->accel_x_zero_p_offset);

  if (param_ro->comp_param.enable == 1) {
    sensing_result->ego.v_lp =
        (1 - param_ro->comp_param.v_lp_gain) * sensing_result->ego.v_c +
        param_ro->comp_param.v_lp_gain * sensing_result->ego.v_lp;
  } else if (param_ro->comp_param.enable == 2) {
    // sensing_result->ego.v_lp =
    //     (1 - param_ro->comp_param.v_lp_gain) * sum_v / enc_v_q.size() +
    //     param_ro->comp_param.v_lp_gain * sensing_result->ego.v_lp;
    // printf("%f, %f %f %d\n", sensing_result->ego.v_lp,
    // sensing_result->ego.v_c,
    //        sum_v, enc_v_q.size());
  }

  // sensing_result->ego.main_v = sum_v / enc_v_q.size();
  // param_ro->comp_param.gain *
  //     (sensing_result->ego.main_v + sensing_result->ego.accel_x_raw * dt) +
  // (1 - param_ro->comp_param.gain) * sensing_result->ego.v_lp;

  // if (param_ro->comp_param.enable == 0) {
  // } else {
  //   tgt_val->ego_in.dist += sensing_result->ego.main_v * dt;
  //   tgt_val->global_pos.dist += sensing_result->ego.main_v * dt;
  // }

  sensing_result->ego.w_lp =
      sensing_result->ego.w_lp * (1 - param_ro->gyro_param.lp_delay) +
      sensing_result->ego.w_raw * param_ro->gyro_param.lp_delay;

  // kf_w.predict(sensing_result->ego.w_raw);
  // kf_w.update(sensing_result->ego.w_raw);

  if (std::isfinite(tgt_val->ego_in.alpha) &&
      std::isfinite(sensing_result->ego.w_lp)) {
    kf_w.predict(tgt_val->ego_in.alpha);
    kf_w.update(sensing_result->ego.w_lp);
    sensing_result->ego.w_kf = kf_w.get_state();
  }
  if (std::isfinite(tgt_val->ego_in.accl) &&
      std::isfinite(sensing_result->ego.v_c)) {
    kf_v.predict(tgt_val->ego_in.accl);
    kf_v.update(sensing_result->ego.v_c);
    sensing_result->ego.v_kf = kf_v.get_state();
  }

  if (std::isfinite(tgt_val->ego_in.v)) {
    kf_dist.predict(tgt_val->ego_in.v);
    kf_dist.update(tgt_val->ego_in.dist);
    sensing_result->ego.dist_kf = kf_dist.get_state();
  }
  if (std::isfinite(tgt_val->ego_in.w)) {
    kf_ang.predict(tgt_val->ego_in.w);
    kf_ang.update(tgt_val->ego_in.ang);
    sensing_result->ego.ang_kf = kf_ang.get_state();
  }

  sensing_result->ego.battery_raw = sensing_result->battery.data;

  sensing_result->ego.battery_lp =
      sensing_result->ego.battery_lp * (1 - param_ro->battery_param.lp_delay) +
      sensing_result->ego.battery_raw * param_ro->battery_param.lp_delay;

  kf_batt.predict(0);
  kf_batt.update(sensing_result->ego.battery_raw);
  sensing_result->ego.batt_kf = kf_batt.get_state();

  sensing_result->ego.left45_lp_old = sensing_result->ego.left45_lp;
  sensing_result->ego.left90_lp_old = sensing_result->ego.left90_lp;
  sensing_result->ego.front_lp_old = sensing_result->ego.front_lp;
  sensing_result->ego.right45_lp_old = sensing_result->ego.right45_lp;
  sensing_result->ego.right90_lp_old = sensing_result->ego.right90_lp;

  // sensing_result->ego.right45_2_lp_old = sensing_result->ego.right45_2_lp;
  // sensing_result->ego.left45_2_lp_old = sensing_result->ego.left45_2_lp;

  sensing_result->ego.right90_raw = sensing_result->led_sen.right90.raw;
  sensing_result->ego.right90_lp =
      sensing_result->ego.right90_lp * (1 - param_ro->led_param.lp_delay) +
      sensing_result->ego.right90_raw * param_ro->led_param.lp_delay;

  sensing_result->ego.right45_raw = sensing_result->led_sen.right45.raw;
  sensing_result->ego.right45_lp =
      sensing_result->ego.right45_lp * (1 - param_ro->led_param.lp_delay) +
      sensing_result->ego.right45_raw * param_ro->led_param.lp_delay;

  sensing_result->ego.front_raw = sensing_result->led_sen.front.raw;
  sensing_result->ego.front_lp =
      sensing_result->ego.front_lp * (1 - param_ro->led_param.lp_delay) +
      sensing_result->ego.front_raw * param_ro->led_param.lp_delay;

  sensing_result->ego.left45_raw = sensing_result->led_sen.left45.raw;
  sensing_result->ego.left45_lp =
      sensing_result->ego.left45_lp * (1 - param_ro->led_param.lp_delay) +
      sensing_result->ego.left45_raw * param_ro->led_param.lp_delay;

  sensing_result->ego.left90_raw = sensing_result->led_sen.left90.raw;
  sensing_result->ego.left90_lp =
      sensing_result->ego.left90_lp * (1 - param_ro->led_param.lp_delay) +
      sensing_result->ego.left90_raw * param_ro->led_param.lp_delay;

  // sensing_result->ego.right45_2_raw = sensing_result->led_sen.right45_2.raw;
  // sensing_result->ego.right45_2_lp =
  //     sensing_result->ego.right45_2_lp * (1 - param_ro->led_param.lp_delay) +
  //     sensing_result->ego.right45_2_raw * param_ro->led_param.lp_delay;

  // sensing_result->ego.left45_2_raw = sensing_result->led_sen.left45_2.raw;
  // sensing_result->ego.left45_2_lp =
  //     sensing_result->ego.left45_2_lp * (1 - param_ro->led_param.lp_delay) +
  //     sensing_result->ego.left45_2_raw * param_ro->led_param.lp_delay;
  // コピー
  tgt_val->ego_in.slip_point.w = sensing_result->ego.w_lp;
}
bool PlanningTask::judge_motor_pwm(float duty, uint8_t type) {
  if (type == 1) {
    return duty > 0;
  } else {
    return duty < 0;
  }
  return true;
}

void PlanningTask::limitter(float &kp, float &ki, float &kb, float &kd,
                            pid_param_t &limitter) {
  if (limitter.mode == 0) {
    return;
  }
  if (kp > limitter.p) {
    kp = limitter.p;
  } else if (kp < -limitter.p) {
    kp = -limitter.p;
  }
  if (ki > limitter.i) {
    ki = limitter.i;
  } else if (ki < -limitter.i) {
    ki = -limitter.i;
  }
  if (kb > limitter.b) {
    kb = limitter.b;
  } else if (kb < -limitter.b) {
    kb = -limitter.b;
  }
  if (kd > limitter.d) {
    kd = limitter.d;
  } else if (kd < -limitter.d) {
    kd = -limitter.d;
  }
}

void PlanningTask::change_pwm_freq(float duty_l, float duty_r) {
  if (param_ro->motor_debug_mode != 0) {
    duty_r = param_ro->motor_debug_mode_duty_r;
    duty_l = param_ro->motor_debug_mode_duty_l;
  }
  float duty_r_abs = duty_r > 0 ? duty_r : -duty_r;
  float duty_l_abs = duty_l > 0 ? duty_l : -duty_l;
  // if (param_ro->motor_driver_type == MotorDriveType::TWO_PWM) {
  //   if (judge_motor_pwm(duty_l, param_ro->motor_l_cw_ccw_type)) {
  //     mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_l_abs);
  //     mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A,
  //                         MCPWM_DUTY_MODE_0);
  //     mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
  //     mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0);
  //     mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B,
  //                         MCPWM_DUTY_MODE_0);
  //   } else {
  //     mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
  //     mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A,
  //                         MCPWM_DUTY_MODE_0);
  //     mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
  //     mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, duty_l_abs);
  //     mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B,
  //                         MCPWM_DUTY_MODE_0);
  //   }
  //   if (judge_motor_pwm(duty_r, param_ro->motor_r_cw_ccw_type)) {
  //     mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, duty_r_abs);
  //     mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A,
  //                         MCPWM_DUTY_MODE_0);
  //     mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B);
  //     mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 0);
  //     mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B,
  //                         MCPWM_DUTY_MODE_0);
  //   } else {
  //     mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
  //     mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A,
  //                         MCPWM_DUTY_MODE_0);
  //     mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);
  //     mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, duty_r_abs);
  //     mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B,
  //                         MCPWM_DUTY_MODE_0);
  //   }
  // } else if (param_ro->motor_driver_type == MotorDriveType::EN1_PH1) {
  //   if (judge_motor_pwm(duty_l, param_ro->motor_l_cw_ccw_type)) {
  //     set_gpio_state(L_CW_CCW1, true);
  //   } else {
  //     set_gpio_state(L_CW_CCW1, false);
  //   }
  //   if (judge_motor_pwm(duty_r, param_ro->motor_r_cw_ccw_type)) {
  //     set_gpio_state(R_CW_CCW1, true);
  //   } else {
  //     set_gpio_state(R_CW_CCW1, false);
  //   }
  //   mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_l_abs);
  //   mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A,
  //                       MCPWM_DUTY_MODE_0);
  //   mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, duty_r_abs);
  //   mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A,
  //                       MCPWM_DUTY_MODE_0);
  // } else if (param_ro->motor_driver_type == MotorDriveType::EN1_PH2) {
  uint32_t high = 0;
  uint32_t low = 0;

  if (judge_motor_pwm(duty_l, param_ro->motor_l_cw_ccw_type)) {
    high |= L_CW_CCW1_BIT;
    low |= L_CW_CCW2_BIT;
  } else {
    high |= L_CW_CCW2_BIT;
    low |= L_CW_CCW1_BIT;
  }
  if (judge_motor_pwm(duty_r, param_ro->motor_r_cw_ccw_type)) {
    high |= R_CW_CCW1_BIT;
    low |= R_CW_CCW2_BIT;
  } else {
    high |= R_CW_CCW2_BIT;
    low |= R_CW_CCW1_BIT;
  }

  GPIO.out1_w1ts.val = high;
  GPIO.out1_w1tc.val = low;
  // auto start_que_rec = esp_timer_get_time();

  // MCPWM[MCPWM_UNIT_0]->timer[MCPWM_TIMER_0].cmpr_value[MCPWM_OPR_A].cmpr_val
  // = duty_l_abs;
  // MCPWM[MCPWM_UNIT_0]->timer[MCPWM_TIMER_1].cmpr_value[MCPWM_OPR_A].cmpr_val
  // = duty_r_abs;

  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_l_abs);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, duty_r_abs);
  // auto end_que_rec = esp_timer_get_time();
  // tgt_val->calc_time2 = end_que_rec - start_que_rec;
  // }
}

void PlanningTask::set_next_duty(float duty_l, float duty_r,
                                 float duty_suction) {
  if (motor_en) {
    change_pwm_freq(duty_l, duty_r);
  }
  if (suction_en) {
    float duty_suction_in = 0;

    if (tgt_val->tgt_in.tgt_dist > 60 &&
        (tgt_val->ego_in.state == 0 || tgt_val->ego_in.state == 1) &&
        tgt_val->motion_type == MotionType::STRAIGHT) {
      duty_suction_in =
          100.0 * tgt_duty.duty_suction_low / sensing_result->ego.batt_kf;
    } else {
      duty_suction_in =
          100.0 * tgt_duty.duty_suction / sensing_result->ego.batt_kf;
    }
    if (duty_suction_in > 100) {
      duty_suction_in = 100.0;
    }
    gain_cnt += 1.0;
    if (gain_cnt > suction_gain) {
      gain_cnt = suction_gain;
    }
    duty_suction_in = duty_suction_in * gain_cnt / suction_gain;
    if (duty_suction_in > 100) {
      duty_suction_in = 100;
    }
    // mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_A);
    mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_A, duty_suction_in);
    // mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_A,
    //                     MCPWM_DUTY_MODE_0);
  }
}

void PlanningTask::init_gpio() {
  gpio_config_t io_conf;
  // 割り込みをしない
  io_conf.intr_type = GPIO_INTR_DISABLE;
  // 出力モード
  io_conf.mode = GPIO_MODE_OUTPUT;
  // 設定したいピンのビットマスク
  io_conf.pin_bit_mask = 0;

  io_conf.pin_bit_mask |= 1ULL << L_CW_CCW1;
  io_conf.pin_bit_mask |= 1ULL << R_CW_CCW1;
  io_conf.pin_bit_mask |= 1ULL << L_CW_CCW2;
  io_conf.pin_bit_mask |= 1ULL << R_CW_CCW2;

  // 内部プルダウンしない
  io_conf.pull_down_en = (gpio_pulldown_t)0;
  // 内部プルアップしない
  io_conf.pull_up_en = (gpio_pullup_t)0;
  // 設定をセットする
  gpio_config(&io_conf);
  gpio_set_level(SUCTION_PWM, 0);
  // gpio_set_direction((gpio_num_t)GPIO_OUTPUT_IO_8,
}

void PlanningTask::pl_req_activate() {
  if (receive_req->pl_req.time_stamp != pid_req_timestamp) {
    if (receive_req->pl_req.error_gyro_reset == 1) {
      error_entity.v.error_i = 0;
    }
    if (receive_req->pl_req.error_vel_reset == 1) {
      error_entity.dist.error_i = 0;
    }
    if (receive_req->pl_req.error_dist_reset == 1) {
      error_entity.w.error_i = 0;
    }
    if (receive_req->pl_req.error_ang_reset == 1) {
      error_entity.ang.error_i = 0;
    }
    if (receive_req->pl_req.error_led_reset == 1) {
      // error_entity.led.error_i = 0;
    }
    // if (tgt_val->pl_req.log_start == 1) {
    //   log_active = true;
    // }
    // if (tgt_val->pl_req.log_end == 1) {
    //   log_active = false;
    // }
    pid_req_timestamp = receive_req->pl_req.time_stamp;
  }
}

float PlanningTask::get_feadforward_front(TurnDirection td) { return 0; }
float PlanningTask::get_feadforward_front() { return 0; }
float PlanningTask::get_feadforward_roll() { return 0; }
float PlanningTask::get_rpm_ff_val(TurnDirection td) { return 0; }
float PlanningTask::satuate_sen_duty(float duty_sen) { return duty_sen; }
void PlanningTask::calc_tgt_duty() {

  float duty_sen = 0;
  float sen_ang = 0;

  error_entity.s_val.p = error_entity.s_val.i = error_entity.s_val.d = 0;
  error_entity.s_val.p_val = error_entity.s_val.i_val =
      error_entity.s_val.d_val = 0;
  error_entity.s_val.z = error_entity.s_val.zz = 0;
  if (tgt_val->nmr.sct == SensorCtrlType::Straight) {
    duty_sen = calc_sensor_pid();
    // if (search_mode) {
    // } else {
    //   sen_ang = calc_sensor_pid();
    // }
    error_entity.sen_dia.error_i = 0;
    error_entity.sen_log_dia.gain_zz = 0;
    error_entity.sen_log_dia.gain_z = 0;
  } else if (tgt_val->nmr.sct == SensorCtrlType::Dia) {
    // if (param_ro->angle_pid.c == 0) {
    //   duty_sen = calc_sensor_pid_dia();
    // } else {
    // }
    sen_ang = calc_sensor_pid_dia();
    error_entity.sen.error_i = 0;
    error_entity.sen_log.gain_zz = 0;
    error_entity.sen_log.gain_z = 0;
  } else if (tgt_val->nmr.sct == SensorCtrlType::NONE) {
    sen_ang = 0;
    error_entity.sen.error_i = 0;
    error_entity.sen_log.gain_zz = 0;
    error_entity.sen_log.gain_z = 0;
    error_entity.sen_dia.error_i = 0;
    error_entity.sen_log_dia.gain_zz = 0;
    error_entity.sen_log_dia.gain_z = 0;
  }
  sensing_result->ego.duty.sen = duty_sen;
  sensing_result->ego.duty.sen_ang = sen_ang;

  error_entity.v.error_dd = error_entity.v.error_d;
  error_entity.v.error_dd = error_entity.v.error_d;
  error_entity.v_kf.error_dd = error_entity.v_kf.error_d;
  error_entity.dist.error_dd = error_entity.dist.error_d;
  error_entity.w.error_dd = error_entity.w.error_d;
  error_entity.w_kf.error_dd = error_entity.w_kf.error_d;
  error_entity.ang.error_dd = error_entity.ang.error_d;

  error_entity.v.error_d = error_entity.v.error_p;
  error_entity.v_r.error_d = error_entity.v_r.error_p;
  error_entity.v_l.error_d = error_entity.v_l.error_p;
  error_entity.v_kf.error_d = error_entity.v_kf.error_p;
  error_entity.dist.error_d = error_entity.dist.error_p;
  error_entity.w.error_d = error_entity.w.error_p;
  error_entity.w_kf.error_d = error_entity.w_kf.error_p;
  error_entity.ang.error_d = error_entity.ang.error_p;

  if (param_ro->comp_param.enable == 0) {
    error_entity.v.error_p = tgt_val->ego_in.v - sensing_result->ego.v_c;
    error_entity.w.error_p = tgt_val->ego_in.w - sensing_result->ego.w_lp;
    error_entity.v_r.error_p = ideal_v_r - sensing_result->ego.v_r;
    error_entity.v_l.error_p = ideal_v_l - sensing_result->ego.v_l;
    error_entity.v_kf.error_p = tgt_val->ego_in.v - sensing_result->ego.v_kf;
    error_entity.w_kf.error_p = tgt_val->ego_in.w - sensing_result->ego.w_kf;
  } else {
    error_entity.v.error_p = tgt_val->ego_in.v - sensing_result->ego.v_kf;
    error_entity.w.error_p = tgt_val->ego_in.w - sensing_result->ego.w_kf;
  }

  // tgt_val->global_pos.img_ang += mpc_next_ego.w;
  // tgt_val->global_pos.img_dist += mpc_next_ego.v;

  // error_entity.dist.error_p = tgt_val->ego_in.img_dist -
  // tgt_val->ego_in.dist; error_entity.ang.error_p = tgt_val->ego_in.img_ang
  // - tgt_val->ego_in.ang;

  error_entity.dist.error_p =
      tgt_val->global_pos.img_dist - tgt_val->global_pos.dist;
  if (error_entity.dist.error_p > param_ro->front_ctrl_error_th) {
    error_entity.dist.error_p = param_ro->front_ctrl_error_th;
  } else if (error_entity.dist.error_p < -param_ro->front_ctrl_error_th) {
    error_entity.dist.error_p = -param_ro->front_ctrl_error_th;
  }

  error_entity.ang.error_p =
      (tgt_val->global_pos.img_ang + sen_ang) - tgt_val->global_pos.ang;

  if (tgt_val->motion_type == MotionType::FRONT_CTRL) {
    error_entity.v.error_i = error_entity.v.error_d = 0;
    error_entity.w.error_i = error_entity.w.error_d = 0;
    error_entity.v_r.error_i = error_entity.v_r.error_d = 0;
    error_entity.v_l.error_i = error_entity.v_l.error_d = 0;

    // error_entity.v.error_i = 0;
    // error_entity.w.error_i = 0;
    if (sensing_result->ego.front_dist < param_ro->cell) {
      error_entity.dist.error_p = sensing_result->ego.front_dist -
                                  param_ro->sen_ref_p.search_exist.front_ctrl;
      error_entity.ang.error_p =
          (sensing_result->ego.right90_dist - sensing_result->ego.left90_dist) /
              2 -
          param_ro->sen_ref_p.search_exist.kireme_r;
      // error_entity.v.error_p = sensing_result->ego.front_dist - 45;
      // error_entity.w.error_p =
      //     (sensing_result->ego.right90_dist -
      //     sensing_result->ego.left90_dist) / 2;
    } else {
      error_entity.dist.error_p = error_entity.dist.error_i =
          error_entity.dist.error_d = 0;
      error_entity.ang.error_p = error_entity.ang.error_i =
          error_entity.ang.error_d = 0;
      // error_entity.v.error_p = error_entity.v.error_i =
      // error_entity.v.error_d =
      //     0;
      // error_entity.w.error_p = error_entity.w.error_i =
      // error_entity.w.error_d =
      //     0;
    }
  }

  error_entity.v_kf.error_d =
      error_entity.v_kf.error_p - error_entity.v_kf.error_d;
  error_entity.v.error_d = error_entity.v.error_p - error_entity.v.error_d;
  error_entity.dist.error_d =
      error_entity.dist.error_p - error_entity.dist.error_d;
  error_entity.w.error_d = error_entity.w.error_p - error_entity.w.error_d;
  error_entity.w_kf.error_d =
      error_entity.w_kf.error_p - error_entity.w_kf.error_d;
  error_entity.ang.error_d =
      error_entity.ang.error_p - error_entity.ang.error_d;

  error_entity.v_l.error_d =
      error_entity.v_l.error_p - error_entity.v_l.error_d;
  error_entity.v_r.error_d =
      error_entity.v_r.error_p - error_entity.v_r.error_d;

  error_entity.v_kf.error_dd =
      error_entity.v_kf.error_d - error_entity.v_kf.error_dd;
  error_entity.v.error_dd = error_entity.v.error_d - error_entity.v.error_dd;
  error_entity.dist.error_dd =
      error_entity.dist.error_d - error_entity.dist.error_dd;
  error_entity.w.error_dd = error_entity.w.error_d - error_entity.w.error_dd;
  error_entity.w_kf.error_dd =
      error_entity.w_kf.error_d - error_entity.w_kf.error_dd;
  error_entity.ang.error_dd =
      error_entity.ang.error_d - error_entity.ang.error_dd;

  error_entity.v.error_i += error_entity.v.error_p;
  error_entity.dist.error_i += error_entity.dist.error_p;
  error_entity.w.error_i += error_entity.w.error_p;
  error_entity.ang.error_i += error_entity.ang.error_p;

  error_entity.v_l.error_i += error_entity.v_l.error_p;
  error_entity.v_r.error_i += error_entity.v_r.error_p;

  tgt_val->v_error = error_entity.v.error_i;
  tgt_val->w_error = error_entity.w.error_i;

  // float duty_rpm_r = get_rpm_ff_val(TurnDirection::Right);
  // float duty_rpm_l = get_rpm_ff_val(TurnDirection::Left);
  // float duty_rpm_r = get_feadforward_front(TurnDirection::Right);
  // float duty_rpm_l = get_feadforward_front(TurnDirection::Left);
  float duty_ff_front = 0;
  float duty_ff_roll = 0;
  // duty_ff_front = get_feadforward_front();
  // duty_ff_roll = get_feadforward_roll();

  duty_c = 0;
  duty_c2 = 0;
  duty_roll = 0;
  duty_roll2 = 0;

  if (tgt_val->motion_type == MotionType::FRONT_CTRL || !motor_en ||
      tgt_val->motion_type == MotionType::NONE) {
    error_entity.v.error_i = error_entity.v.error_d = 0;
    error_entity.v_kf.error_i = error_entity.v_kf.error_d = 0;
    error_entity.w_kf.error_i = error_entity.w_kf.error_d = 0;
    error_entity.v_log.gain_z = error_entity.v_log.gain_zz = 0;
    error_entity.v_l.error_i = error_entity.v_l.error_d = 0;
    error_entity.v_r.error_i = error_entity.v_r.error_d = 0;
    error_entity.v_l_log.gain_z = error_entity.v_l_log.gain_zz = 0;
    error_entity.v_r_log.gain_z = error_entity.v_r_log.gain_zz = 0;
    error_entity.sen.error_i = error_entity.sen.error_d = 0;
    error_entity.sen_log.gain_zz = error_entity.sen_log.gain_z = 0;
    error_entity.sen_dia.error_i = error_entity.sen_dia.error_d = 0;
    error_entity.sen_log_dia.gain_zz = error_entity.sen_log_dia.gain_z = 0;
  }

  // if (param_ro->motor_pid.mode == 1) {
  //   duty_c = param_ro->motor_pid.p * error_entity.v.error_p +
  //            param_ro->motor_pid.i * error_entity.v.error_i +
  //            param_ro->motor_pid.d * error_entity.v.error_d +
  //            (error_entity.v_log.gain_z - error_entity.v_log.gain_zz) * dt;
  //   error_entity.v_log.gain_zz = error_entity.v_log.gain_z;
  //   error_entity.v_log.gain_z = duty_c;
  // } else {
  //   duty_c = param_ro->motor_pid.p * error_entity.v.error_p +
  //            param_ro->motor_pid.i * error_entity.v.error_i +
  //            param_ro->motor_pid.d * error_entity.v.error_d;
  //   error_entity.v_log.gain_zz = 0;
  //   error_entity.v_log.gain_z = 0;
  // }
  const unsigned char reset_req = motor_en ? 1 : 0;
  const unsigned char reset = 0;
  const unsigned char enable = 1;

  // reset
  error_entity.v_val.p = error_entity.v_val.i = error_entity.v_val.d = 0;
  error_entity.w_val.p = error_entity.w_val.i = error_entity.w_val.d = 0;
  error_entity.v_val.p_val = error_entity.v_val.i_val =
      error_entity.v_val.d_val = 0;
  error_entity.w_val.p_val = error_entity.w_val.i_val =
      error_entity.w_val.d_val = 0;
  error_entity.v_val.z = error_entity.v_val.zz = 0;
  error_entity.w_val.z = error_entity.w_val.zz = 0;

  if (tgt_val->motion_type == MotionType::FRONT_CTRL || !motor_en) {
    vel_pid.step(&error_entity.v.error_p, &param_ro->motor_pid.p,
                 &param_ro->motor_pid.i, &param_ro->motor_pid.d, &reset, &dt,
                 &duty_c);
    set_ctrl_val(
        error_entity.v_val, error_entity.v.error_p, error_entity.v.error_i, 0,
        error_entity.v.error_d, param_ro->motor_pid.p * error_entity.v.error_p,
        vel_pid.simple_pid_controller_DW.Integrator_DSTATE, 0, 0, 0, 0);
  } else {
    // if (tgt_val->motion_type == MotionType::PIVOT) {
    if (tgt_val->motion_type == MotionType::NONE) {
      vel_pid.step(&error_entity.v.error_p, &param_ro->motor_pid.p,
                   &param_ro->motor_pid.i, &param_ro->motor_pid.d, &reset_req,
                   &dt, &duty_c);

      set_ctrl_val(error_entity.v_val, error_entity.v.error_p,
                   error_entity.v.error_i, 0, error_entity.v.error_d, 0, 0, 0,
                   0, 0, 0);
    } else {
      if (param_ro->motor_pid2.mode == 3) {
        const auto diff_dist =
            tgt_val->ego_in.img_dist - sensing_result->ego.dist_kf;
        duty_c = param_ro->motor_pid2.p * error_entity.v.error_p +
                 param_ro->motor_pid2.i * diff_dist +
                 param_ro->motor_pid2.d * error_entity.v.error_d;

        set_ctrl_val(error_entity.v_val, error_entity.v.error_p, 0, diff_dist,
                     error_entity.v.error_d,
                     param_ro->motor_pid2.p * error_entity.v.error_p,
                     param_ro->motor_pid2.i * diff_dist, 0,
                     param_ro->motor_pid2.d * error_entity.v.error_d, 0, 0);
      } else if (param_ro->motor_pid2.mode == 2) {
        const auto diff_dist =
            tgt_val->ego_in.img_dist - sensing_result->ego.dist_kf;
        duty_c = param_ro->motor_pid2.p * error_entity.v.error_p +
                 param_ro->motor_pid2.i * diff_dist +
                 param_ro->motor_pid2.d * error_entity.v.error_d +
                 (error_entity.v_log.gain_z - error_entity.v_log.gain_zz) * dt;
        error_entity.v_log.gain_zz = error_entity.v_log.gain_z;
        error_entity.v_log.gain_z = duty_c;

        set_ctrl_val(error_entity.v_val, error_entity.v.error_p, 0, diff_dist,
                     error_entity.v.error_d,
                     param_ro->motor_pid2.p * error_entity.v.error_p, 0,
                     param_ro->motor_pid2.i * diff_dist,
                     param_ro->motor_pid2.d * error_entity.v.error_d,
                     error_entity.v_log.gain_zz, error_entity.v_log.gain_z);

      } else if (param_ro->motor_pid2.mode == 4) {
        const auto diff_dist =
            tgt_val->ego_in.img_dist - sensing_result->ego.dist_kf;
        duty_c = param_ro->motor_pid2.p * error_entity.v.error_p +
                 param_ro->motor_pid2.i * error_entity.v.error_i +
                 param_ro->motor_pid2.b * diff_dist +
                 param_ro->motor_pid2.d * error_entity.v.error_d +
                 (error_entity.v_log.gain_z - error_entity.v_log.gain_zz) * dt;
        error_entity.v_log.gain_zz = error_entity.v_log.gain_z;
        error_entity.v_log.gain_z = duty_c;

        set_ctrl_val(error_entity.v_val, error_entity.v.error_p,
                     error_entity.v.error_i, diff_dist, error_entity.v.error_d,
                     param_ro->motor_pid2.p * error_entity.v.error_p,
                     param_ro->motor_pid2.i * error_entity.v.error_i,
                     param_ro->motor_pid2.b * diff_dist,
                     param_ro->motor_pid2.d * error_entity.v.error_d,
                     error_entity.v_log.gain_zz, error_entity.v_log.gain_z);

      } else if (param_ro->motor_pid2.mode == 7) {
        const auto diff_dist =
            tgt_val->ego_in.img_dist - sensing_result->ego.dist_kf;
        auto kp_gain = param_ro->motor_pid2.p * error_entity.v.error_p;
        auto ki_gain = param_ro->motor_pid2.i * error_entity.v.error_i;
        auto kb_gain = param_ro->motor_pid2.b * diff_dist;
        auto kd_gain = param_ro->motor_pid2.d * error_entity.v_kf.error_d;
        limitter(kp_gain, ki_gain, kb_gain, kd_gain,
                 param_ro->motor2_pid_gain_limitter);

        duty_c = kp_gain + ki_gain + kb_gain + kd_gain +
                 (error_entity.v_log.gain_z - error_entity.v_log.gain_zz) * dt;
        error_entity.v_log.gain_zz = error_entity.v_log.gain_z;
        error_entity.v_log.gain_z = duty_c;

        set_ctrl_val(error_entity.v_val, error_entity.v.error_p,
                     error_entity.v.error_i, diff_dist, error_entity.v.error_d,
                     kp_gain, ki_gain, kb_gain, kd_gain,
                     error_entity.v_log.gain_zz, error_entity.v_log.gain_z);

      } else if (param_ro->motor_pid2.mode == 9) {
        auto diff_dist = tgt_val->ego_in.img_dist - sensing_result->ego.dist_kf;
        auto kp_gain = param_ro->motor_pid2.p * error_entity.v.error_d;
        auto ki_gain = param_ro->motor_pid2.i * error_entity.v.error_p;
        auto kb_gain = 0.0f;
        auto kd_gain = param_ro->motor_pid2.d * error_entity.v_kf.error_dd;

        limitter(kp_gain, ki_gain, kb_gain, kd_gain,
                 param_ro->motor2_pid_gain_limitter);

        duty_c =
            error_entity.v_log.gain_z + kp_gain + ki_gain + kb_gain + kd_gain;
        error_entity.v_log.gain_zz = error_entity.v_log.gain_z;
        error_entity.v_log.gain_z = duty_c;

        set_ctrl_val(error_entity.v_val, error_entity.v.error_d,
                     error_entity.v.error_p, 0, error_entity.v.error_dd,
                     kp_gain, ki_gain, kb_gain, kd_gain,
                     error_entity.v_log.gain_zz, error_entity.v_log.gain_z);

      } else if (param_ro->motor_pid2.mode == 10) {
        auto kp_gain = param_ro->motor_pid2.p * error_entity.v_kf.error_d;
        auto ki_gain = param_ro->motor_pid2.i * error_entity.v_kf.error_p;
        auto kb_gain = 0.0f;
        auto kd_gain = param_ro->motor_pid2.d * error_entity.v_kf.error_dd;

        limitter(kp_gain, ki_gain, kb_gain, kd_gain,
                 param_ro->motor2_pid_gain_limitter);

        duty_c =
            error_entity.v_log.gain_z + kp_gain + ki_gain + kb_gain + kd_gain;
        error_entity.v_log.gain_zz = error_entity.v_log.gain_z;
        error_entity.v_log.gain_z = duty_c;

        set_ctrl_val(error_entity.v_val, error_entity.v.error_d,
                     error_entity.v.error_p, 0, error_entity.v.error_dd,
                     kp_gain, ki_gain, kb_gain, kd_gain,
                     error_entity.v_log.gain_zz, error_entity.v_log.gain_z);

      } else if (param_ro->motor_pid2.mode == 8) {
        const auto diff_dist =
            tgt_val->ego_in.img_dist - sensing_result->ego.dist_kf;
        duty_c = param_ro->motor_pid2.p * error_entity.v.error_p +
                 param_ro->motor_pid2.i * error_entity.v.error_i +
                 param_ro->motor_pid2.b * diff_dist +
                 param_ro->motor_pid2.d * error_entity.v_kf.error_d;
        error_entity.v_log.gain_zz = 0;
        error_entity.v_log.gain_z = 0;

        set_ctrl_val(error_entity.v_val, error_entity.v.error_p,
                     error_entity.v.error_i, diff_dist, error_entity.v.error_d,
                     param_ro->motor_pid2.p * error_entity.v.error_p,
                     param_ro->motor_pid2.i * error_entity.v.error_i,
                     param_ro->motor_pid2.b * diff_dist,
                     param_ro->motor_pid2.d * error_entity.v.error_d, 0, 0);

      } else if (param_ro->motor_pid2.mode == 6) {
        const auto diff_v = tgt_val->ego_in.v - sensing_result->ego.v_kf;
        duty_c = param_ro->motor_pid2.p * error_entity.v.error_p +
                 param_ro->motor_pid2.i * error_entity.v.error_i +
                 param_ro->motor_pid2.b * diff_v +
                 param_ro->motor_pid2.d * error_entity.v.error_d +
                 (error_entity.v_log.gain_z - error_entity.v_log.gain_zz) * dt;
        error_entity.v_log.gain_zz = error_entity.v_log.gain_z;
        error_entity.v_log.gain_z = duty_c;

        set_ctrl_val(error_entity.v_val, error_entity.v.error_p,
                     error_entity.v.error_i, diff_v, error_entity.v.error_d,
                     param_ro->motor_pid2.p * error_entity.v.error_p,
                     param_ro->motor_pid2.i * error_entity.v.error_i,
                     param_ro->motor_pid2.b * diff_v,
                     param_ro->motor_pid2.d * error_entity.v.error_d,
                     error_entity.v_log.gain_zz, error_entity.v_log.gain_z);

      } else if (param_ro->motor_pid2.mode == 1) {
        duty_c = param_ro->motor_pid2.p * error_entity.v.error_p +
                 param_ro->motor_pid2.i * error_entity.v.error_i +
                 param_ro->motor_pid2.d * error_entity.v.error_d +
                 (error_entity.v_log.gain_z - error_entity.v_log.gain_zz) * dt;
        error_entity.v_log.gain_zz = error_entity.v_log.gain_z;
        error_entity.v_log.gain_z = duty_c;
        set_ctrl_val(error_entity.v_val, error_entity.v.error_p,
                     error_entity.v.error_i, 0, error_entity.v.error_d,
                     param_ro->motor_pid2.p * error_entity.v.error_p,
                     param_ro->motor_pid2.i * error_entity.v.error_i,
                     param_ro->motor_pid2.b * 0,
                     param_ro->motor_pid2.d * error_entity.v.error_d,
                     error_entity.v_log.gain_zz, error_entity.v_log.gain_z);
      } else if (param_ro->motor_pid2.mode == 5) {
        duty_c = param_ro->motor_pid2.p * error_entity.v.error_p +
                 param_ro->motor_pid2.i * error_entity.v.error_i +
                 param_ro->motor_pid2.d * error_entity.v.error_d;

        set_ctrl_val(error_entity.v_val, error_entity.v.error_p,
                     error_entity.v.error_i, 0, error_entity.v.error_d,
                     param_ro->motor_pid2.p * error_entity.v.error_p,
                     param_ro->motor_pid2.i * error_entity.v.error_i,
                     param_ro->motor_pid2.b * 0,
                     param_ro->motor_pid2.d * error_entity.v.error_d,
                     error_entity.v_log.gain_zz, error_entity.v_log.gain_z);
      } else {
        vel_pid.step(&error_entity.v.error_p, &param_ro->motor_pid2.p,
                     &param_ro->motor_pid2.i, &param_ro->motor_pid2.d,
                     &reset_req, &dt, &duty_c);
        set_ctrl_val(error_entity.v_val, error_entity.v.error_p,
                     error_entity.v.error_i, 0, error_entity.v.error_d, 0, 0, 0,
                     0, 0, 0);
      }
    }
  }

  if (w_reset == 0 || tgt_val->motion_type == MotionType::FRONT_CTRL ||
      !motor_en) {
    error_entity.w.error_i = error_entity.w.error_d = 0;
    error_entity.w_log.gain_z = error_entity.w_log.gain_zz = 0;
  }

  if (param_ro->gyro_pid.mode == 1) {
    if (w_reset == 0 || tgt_val->motion_type == MotionType::FRONT_CTRL ||
        !motor_en) {
      gyro_pid.step(&error_entity.w.error_p, &param_ro->gyro_pid.p,
                    &param_ro->gyro_pid.i, &param_ro->gyro_pid.d, &reset, &dt,
                    &duty_roll);

      set_ctrl_val(error_entity.w_val, error_entity.w.error_p,
                   error_entity.w.error_i, 0, error_entity.w.error_d, 0, 0, 0,
                   0, 0, 0);
    } else {
      gyro_pid.step(&error_entity.w.error_p, &param_ro->gyro_pid.p,
                    &param_ro->gyro_pid.i, &param_ro->gyro_pid.d, &enable, &dt,
                    &duty_roll);
      set_ctrl_val(error_entity.w_val, error_entity.w.error_p,
                   error_entity.w.error_i, 0, error_entity.w.error_d, 0, 0, 0,
                   0, 0, 0);
    }
  } else if (param_ro->gyro_pid.mode == 0) {
    if (tgt_val->nmr.sct == SensorCtrlType::Dia) {
      if (param_ro->str_ang_dia_pid.mode == 1) {
        duty_roll =
            param_ro->str_ang_dia_pid.p * error_entity.ang.error_p -
            param_ro->str_ang_dia_pid.d * sensing_result->ego.w_lp +
            (error_entity.ang_log.gain_z - error_entity.ang_log.gain_zz) * dt;
        error_entity.ang_log.gain_zz = error_entity.ang_log.gain_z;
        error_entity.ang_log.gain_z = duty_roll;
        // set_ctrl_val(error_entity.w_val, error_entity.w.error_p,
        //              error_entity.w.error_i, 0, error_entity.w.error_d,
        //              param_ro->gyro_pid.p * error_entity.w.error_p,
        //              param_ro->gyro_pid.i * error_entity.w.error_i,
        //              param_ro->gyro_pid.b * 0,
        //              param_ro->gyro_pid.d * error_entity.w.error_d,
        //              error_entity.ang_log.gain_zz,
        //              error_entity.ang_log.gain_z);
      } else {
        duty_roll = param_ro->str_ang_dia_pid.p * error_entity.ang.error_p -
                    param_ro->str_ang_dia_pid.d * sensing_result->ego.w_lp;
        error_entity.ang_log.gain_zz = 0;
        error_entity.ang_log.gain_z = 0;

        // set_ctrl_val(error_entity.w_val, error_entity.w.error_p,
        //              error_entity.w.error_i, 0, error_entity.w.error_d,
        //              param_ro->gyro_pid.p * error_entity.w.error_p,
        //              param_ro->gyro_pid.i * error_entity.w.error_i,
        //              param_ro->gyro_pid.b * 0,
        //              param_ro->gyro_pid.d * error_entity.w.error_d,
        //              error_entity.ang_log.gain_zz,
        //              error_entity.ang_log.gain_z);
      }
    } else if (tgt_val->motion_type == MotionType::PIVOT ||
               tgt_val->motion_type == MotionType::SLALOM || sen_ang == 0 ||
               search_mode) {
      if (
          // tgt_val->motion_type == MotionType::SLALOM ||
          // tgt_val->motion_type == MotionType::SLA_FRONT_STR ||
          tgt_val->motion_type == MotionType::SLA_BACK_STR) {
        duty_roll = param_ro->gyro_pid.p * error_entity.w.error_p +
                    param_ro->gyro_pid.b * error_entity.w.error_i +
                    param_ro->gyro_pid.c * error_entity.w.error_d;

        set_ctrl_val(error_entity.w_val, error_entity.w.error_p,
                     error_entity.w.error_i, 0, error_entity.w.error_d,
                     param_ro->gyro_pid.p * error_entity.w.error_p,
                     param_ro->gyro_pid.i * error_entity.w.error_i,
                     param_ro->gyro_pid.b * 0,
                     param_ro->gyro_pid.d * error_entity.w.error_d, 0, 0);
      } else {
        duty_roll = param_ro->gyro_pid.p * error_entity.w.error_p +
                    param_ro->gyro_pid.i * error_entity.w.error_i +
                    param_ro->gyro_pid.d * error_entity.w.error_d;
        set_ctrl_val(error_entity.w_val, error_entity.w.error_p,
                     error_entity.w.error_i, 0, error_entity.w.error_d,
                     param_ro->gyro_pid.p * error_entity.w.error_p,
                     param_ro->gyro_pid.i * error_entity.w.error_i,
                     param_ro->gyro_pid.b * 0,
                     param_ro->gyro_pid.d * error_entity.w.error_d, 0, 0);
      }
      error_entity.ang_log.gain_zz = 0;
      error_entity.ang_log.gain_z = 0;
    } else {
      error_entity.ang_log.gain_zz = 0;
      error_entity.ang_log.gain_z = 0;
    }
  } else if (param_ro->gyro_pid.mode == 2) {
    if (tgt_val->nmr.sct == SensorCtrlType::Dia) {
      if (param_ro->str_ang_dia_pid.mode == 1) {
        duty_roll =
            param_ro->str_ang_dia_pid.p * error_entity.ang.error_p -
            param_ro->str_ang_dia_pid.d * sensing_result->ego.w_lp +
            (error_entity.ang_log.gain_z - error_entity.ang_log.gain_zz) * dt;
        error_entity.ang_log.gain_zz = error_entity.ang_log.gain_z;
        error_entity.ang_log.gain_z = duty_roll;

        set_ctrl_val(error_entity.w_val, error_entity.ang.error_p, 0, 0,
                     sensing_result->ego.w_lp,
                     param_ro->str_ang_dia_pid.p * error_entity.ang.error_p, 0,
                     0, param_ro->str_ang_dia_pid.d * sensing_result->ego.w_lp,
                     error_entity.ang_log.gain_zz, error_entity.ang_log.gain_z);
      } else {
        duty_roll = param_ro->str_ang_dia_pid.p * error_entity.ang.error_p -
                    param_ro->str_ang_dia_pid.d * sensing_result->ego.w_lp;
        error_entity.ang_log.gain_zz = 0;
        error_entity.ang_log.gain_z = 0;
        set_ctrl_val(error_entity.w_val, error_entity.ang.error_p, 0, 0,
                     sensing_result->ego.w_lp,
                     param_ro->str_ang_dia_pid.p * error_entity.ang.error_p, 0,
                     0, param_ro->str_ang_dia_pid.d * sensing_result->ego.w_lp,
                     0, 0);
      }
    } else {
      if (tgt_val->motion_type == MotionType::PIVOT) {
        duty_roll =
            param_ro->gyro_pid.p * error_entity.w.error_p +
            param_ro->gyro_pid.b * error_entity.w.error_i +
            param_ro->gyro_pid.c * error_entity.w.error_d +
            (error_entity.ang_log.gain_z - error_entity.ang_log.gain_zz) * dt;
        error_entity.ang_log.gain_zz = error_entity.ang_log.gain_z;
        error_entity.ang_log.gain_z = duty_roll;

        set_ctrl_val(error_entity.w_val, error_entity.w.error_p,
                     error_entity.w.error_i, 0, error_entity.w.error_d,
                     param_ro->gyro_pid.p * error_entity.w.error_p,
                     param_ro->gyro_pid.i * error_entity.w.error_i, 0,
                     param_ro->gyro_pid.c * error_entity.w.error_d,
                     error_entity.ang_log.gain_zz, error_entity.ang_log.gain_z);
      } else {
        duty_roll =
            param_ro->gyro_pid.p * error_entity.w.error_p +
            param_ro->gyro_pid.i *
                (tgt_val->ego_in.img_ang - sensing_result->ego.ang_kf) +
            param_ro->gyro_pid.c * error_entity.w.error_d +
            (error_entity.ang_log.gain_z - error_entity.ang_log.gain_zz) * dt;
        error_entity.ang_log.gain_zz = error_entity.ang_log.gain_z;
        error_entity.ang_log.gain_z = duty_roll;

        set_ctrl_val(error_entity.w_val, error_entity.w.error_p,
                     (tgt_val->ego_in.img_ang - sensing_result->ego.ang_kf), 0,
                     error_entity.w.error_d,
                     param_ro->gyro_pid.p * error_entity.w.error_p,
                     param_ro->gyro_pid.i *
                         (tgt_val->ego_in.img_ang - sensing_result->ego.ang_kf),
                     0, param_ro->gyro_pid.c * error_entity.w.error_d,
                     error_entity.ang_log.gain_zz, error_entity.ang_log.gain_z);
      }
    }
  } else if (param_ro->gyro_pid.mode == 3) {
    if (tgt_val->nmr.sct == SensorCtrlType::Dia) {
      if (param_ro->str_ang_dia_pid.mode == 1) {
        duty_roll =
            param_ro->str_ang_dia_pid.p * error_entity.ang.error_p -
            param_ro->str_ang_dia_pid.d * sensing_result->ego.w_lp +
            (error_entity.ang_log.gain_z - error_entity.ang_log.gain_zz) * dt;
        error_entity.ang_log.gain_zz = error_entity.ang_log.gain_z;
        error_entity.ang_log.gain_z = duty_roll;

        set_ctrl_val(error_entity.w_val, error_entity.ang.error_p, 0, 0,
                     sensing_result->ego.w_lp,
                     param_ro->gyro_pid.p * error_entity.w.error_p, 0, 0,
                     param_ro->str_ang_dia_pid.d * sensing_result->ego.w_lp, 0,
                     0);

      } else {
        duty_roll = param_ro->str_ang_dia_pid.p * error_entity.ang.error_p -
                    param_ro->str_ang_dia_pid.d * sensing_result->ego.w_lp;
        error_entity.ang_log.gain_zz = 0;
        error_entity.ang_log.gain_z = 0;
      }
    } else {
      if (tgt_val->motion_type == MotionType::NONE) {
        duty_roll = param_ro->gyro_pid.p * error_entity.w.error_p +
                    param_ro->gyro_pid.b * error_entity.w.error_i +
                    param_ro->gyro_pid.c * error_entity.w.error_d;
        // (error_entity.ang_log.gain_z - error_entity.ang_log.gain_zz) * dt;
        error_entity.ang_log.gain_zz = error_entity.ang_log.gain_z;
        error_entity.ang_log.gain_z = duty_roll;

        set_ctrl_val(error_entity.w_val, error_entity.w.error_p,
                     error_entity.w.error_i, 0, error_entity.w.error_d,
                     param_ro->gyro_pid.p * error_entity.w.error_p,
                     param_ro->gyro_pid.b * error_entity.w.error_i,
                     param_ro->gyro_pid.b * 0,
                     param_ro->gyro_pid.c * error_entity.w.error_d,
                     error_entity.ang_log.gain_zz, error_entity.ang_log.gain_z);

      } else {
        // mode3 main
        auto diff_ang = (tgt_val->ego_in.img_ang - sensing_result->ego.ang_kf);
        if (tgt_val->motion_type == MotionType::SLALOM) {
          diff_ang = 0;
        }
        auto kp_gain = param_ro->gyro_pid.p * error_entity.w.error_p;
        auto ki_gain = param_ro->gyro_pid.i * diff_ang;
        auto kb_gain = param_ro->gyro_pid.b * error_entity.w.error_i;
        auto kc_gain = param_ro->gyro_pid.c * error_entity.w.error_d;
        auto kd_gain = param_ro->gyro_pid.d * error_entity.w_kf.error_d;
        limitter(kp_gain, ki_gain, kb_gain, kd_gain,
                 param_ro->gyro_pid_gain_limitter);
        duty_roll =
            kp_gain + ki_gain + kb_gain + kc_gain + kd_gain +
            (error_entity.ang_log.gain_z - error_entity.ang_log.gain_zz) * dt;

        error_entity.ang_log.gain_zz = error_entity.ang_log.gain_z;
        error_entity.ang_log.gain_z = duty_roll;

        set_ctrl_val(error_entity.w_val,
                     error_entity.w.error_p,                           // p
                     diff_ang,                                         // i
                     error_entity.w.error_i,                           // i2
                     error_entity.w_kf.error_d,                        // d
                     param_ro->gyro_pid.p * error_entity.w.error_p,    // kp*p
                     param_ro->gyro_pid.i * diff_ang,                  // ki*i
                     param_ro->gyro_pid.b * error_entity.w.error_i,    // kb*i2
                     param_ro->gyro_pid.d * error_entity.w_kf.error_d, // kd*d
                     error_entity.ang_log.gain_zz, error_entity.ang_log.gain_z);
        // const auto diff_dist =
        //     tgt_val->ego_in.img_dist - sensing_result->ego.dist_kf;
        // duty_c = param_ro->motor_pid2.p * error_entity.v.error_p +
        //          param_ro->motor_pid2.i * error_entity.v.error_i +
        //          param_ro->motor_pid2.b * diff_dist +
        //          param_ro->motor_pid2.d * error_entity.v_kf.error_d +
        //          (error_entity.v_log.gain_z - error_entity.v_log.gain_zz) *
        //          dt;
        // error_entity.v_log.gain_zz = error_entity.v_log.gain_z;
        // error_entity.v_log.gain_z = duty_c;
      }
    }
  } else if (param_ro->gyro_pid.mode == 4) {
    if (tgt_val->nmr.sct == SensorCtrlType::Dia) {
      if (param_ro->str_ang_dia_pid.mode == 1) {
        duty_roll =
            param_ro->str_ang_dia_pid.p * error_entity.ang.error_p -
            param_ro->str_ang_dia_pid.d * sensing_result->ego.w_lp +
            (error_entity.ang_log.gain_z - error_entity.ang_log.gain_zz) * dt;
        error_entity.ang_log.gain_zz = error_entity.ang_log.gain_z;
        error_entity.ang_log.gain_z = duty_roll;

        set_ctrl_val(error_entity.w_val, error_entity.ang.error_p, 0, 0,
                     sensing_result->ego.w_lp,
                     param_ro->gyro_pid.p * error_entity.w.error_p, 0, 0,
                     param_ro->str_ang_dia_pid.d * sensing_result->ego.w_lp, 0,
                     0);

      } else {
        duty_roll = param_ro->str_ang_dia_pid.p * error_entity.ang.error_p -
                    param_ro->str_ang_dia_pid.d * sensing_result->ego.w_lp;
        error_entity.ang_log.gain_zz = 0;
        error_entity.ang_log.gain_z = 0;
      }
    } else {
      if (tgt_val->motion_type == MotionType::NONE) {
        duty_roll = param_ro->gyro_pid.p * error_entity.w.error_p +
                    param_ro->gyro_pid.b * error_entity.w.error_i +
                    param_ro->gyro_pid.c * error_entity.w.error_d;
        // (error_entity.ang_log.gain_z - error_entity.ang_log.gain_zz) * dt;
        error_entity.ang_log.gain_zz = error_entity.ang_log.gain_z;
        error_entity.ang_log.gain_z = duty_roll;

        set_ctrl_val(error_entity.w_val, error_entity.w.error_p,
                     error_entity.w.error_i, 0, error_entity.w.error_d,
                     param_ro->gyro_pid.p * error_entity.w.error_p,
                     param_ro->gyro_pid.b * error_entity.w.error_i,
                     param_ro->gyro_pid.b * 0,
                     param_ro->gyro_pid.c * error_entity.w.error_d,
                     error_entity.ang_log.gain_zz, error_entity.ang_log.gain_z);

      } else {
        // mode4 main
        const auto diff_ang =
            (tgt_val->ego_in.img_ang - sensing_result->ego.ang_kf);
        auto kp_gain = param_ro->gyro_pid.p * error_entity.w_kf.error_d;
        auto ki_gain = param_ro->gyro_pid.i * error_entity.w.error_p;
        auto kb_gain = 0.0f;
        auto kc_gain = 0.0f;
        auto kd_gain = param_ro->gyro_pid.d * error_entity.w_kf.error_dd;
        limitter(kp_gain, ki_gain, kb_gain, kd_gain,
                 param_ro->gyro_pid_gain_limitter);
        duty_roll =
            error_entity.ang_log.gain_z + kp_gain + ki_gain + kb_gain + kd_gain;
        error_entity.ang_log.gain_zz = error_entity.ang_log.gain_z;
        error_entity.ang_log.gain_z = duty_roll;

        set_ctrl_val(error_entity.w_val,
                     error_entity.w_kf.error_d,          // p
                     error_entity.w.error_p,             // i
                     0,                                  // i2
                     error_entity.w_kf.error_dd,         // d
                     kp_gain, ki_gain, kb_gain, kd_gain, // kd*d
                     error_entity.ang_log.gain_zz, error_entity.ang_log.gain_z);
        // const auto diff_dist =
        //     tgt_val->ego_in.img_dist - sensing_result->ego.dist_kf;
        // duty_c = param_ro->motor_pid2.p * error_entity.v.error_p +
        //          param_ro->motor_pid2.i * error_entity.v.error_i +
        //          param_ro->motor_pid2.b * diff_dist +
        //          param_ro->motor_pid2.d * error_entity.v_kf.error_d +
        //          (error_entity.v_log.gain_z - error_entity.v_log.gain_zz) *
        //          dt;
        // error_entity.v_log.gain_zz = error_entity.v_log.gain_z;
        // error_entity.v_log.gain_z = duty_c;
      }
    }
  }
  sensing_result->ego.duty.sen = duty_roll;
  sensing_result->ego.duty.sen = duty_sen;
  if (tgt_val->motion_type == MotionType::SLALOM) {
    const float max_duty_roll = 13.5;
    if (duty_roll > max_duty_roll) {
      duty_roll = max_duty_roll;
    } else if (duty_roll < -max_duty_roll) {
      duty_roll = -max_duty_roll;
    }
  } else {
    const float max_duty_roll = 3.5;
    if (duty_roll > max_duty_roll) {
      duty_roll = max_duty_roll;
    } else if (duty_roll < -max_duty_roll) {
      duty_roll = -max_duty_roll;
    }
  }
  if (tgt_val->motion_type == MotionType::FRONT_CTRL) {
    // duty_roll = 0;
    // duty_c = 0;
    if (param_ro->dist_pid.mode == 1) {
      duty_c2 = param_ro->dist_pid.p * error_entity.dist.error_p +
                param_ro->dist_pid.i * error_entity.dist.error_i +
                // param_ro->dist_pid.d * error_entity.dist.error_d +
                param_ro->dist_pid.d * sensing_result->ego.v_c;

      // (error_entity.dist_log.gain_z - error_entity.dist_log.gain_zz) * dt;
      error_entity.dist_log.gain_zz = error_entity.dist_log.gain_z;
      error_entity.dist_log.gain_z = duty_c;
    } else {
      duty_c2 = param_ro->dist_pid.p * error_entity.dist.error_p +
                param_ro->dist_pid.i * error_entity.dist.error_i +
                // param_ro->dist_pid.d * error_entity.dist.error_d;
                param_ro->dist_pid.d * sensing_result->ego.v_c;
      error_entity.dist_log.gain_zz = 0;
      error_entity.dist_log.gain_z = 0;
    }
    if (param_ro->angle_pid.mode == 2) {
      duty_roll2 = param_ro->angle_pid.p * error_entity.ang.error_p +
                   param_ro->angle_pid.i * error_entity.ang.error_i +
                   param_ro->angle_pid.d * error_entity.w_kf.error_p;
      error_entity.ang_log.gain_zz = error_entity.ang_log.gain_z;
      error_entity.ang_log.gain_z = duty_roll2;
    } else if (param_ro->angle_pid.mode == 1) {
      duty_roll2 = param_ro->angle_pid.p * error_entity.ang.error_p +
                   param_ro->angle_pid.i * error_entity.ang.error_i +
                   param_ro->angle_pid.d * error_entity.ang.error_d;
      error_entity.ang_log.gain_zz = error_entity.ang_log.gain_z;
      error_entity.ang_log.gain_z = duty_roll2;
    } else {
      duty_roll2 = param_ro->angle_pid.p * error_entity.ang.error_p +
                   param_ro->angle_pid.i * error_entity.ang.error_i +
                   param_ro->angle_pid.d * error_entity.ang.error_d;
      error_entity.ang_log.gain_zz = 0;
      error_entity.ang_log.gain_z = 0;
    }
  }

  // if (tgt_val->motion_type == MotionType::PIVOT) {
  //   duty_roll = 0;
  //   if (param_ro->angle_pid.mode == 1) {
  //     duty_roll2 =
  //         param_ro->angle_pid.p * error_entity.w.error_p +
  //         param_ro->angle_pid.i * error_entity.w.error_i +
  //         param_ro->angle_pid.d * error_entity.w.error_d +
  //         (error_entity.ang_log.gain_z - error_entity.ang_log.gain_zz) * dt;
  //     error_entity.ang_log.gain_zz = error_entity.ang_log.gain_z;
  //     error_entity.ang_log.gain_z = duty_roll2;
  //     set_ctrl_val(error_entity.w_val, error_entity.w.error_p,
  //                  error_entity.w.error_i, 0, error_entity.w.error_d,
  //                  param_ro->angle_pid.p * error_entity.w.error_p,
  //                  param_ro->angle_pid.i * error_entity.w.error_i, 0,
  //                  param_ro->angle_pid.d * error_entity.w.error_d,
  //                  error_entity.ang_log.gain_zz,
  //                  error_entity.ang_log.gain_z);

  //   } else {
  //     duty_roll2 = param_ro->angle_pid.p * error_entity.w.error_p +
  //                  param_ro->angle_pid.i * error_entity.w.error_i +
  //                  param_ro->angle_pid.d * error_entity.w.error_d;
  //     error_entity.ang_log.gain_zz = 0;
  //     error_entity.ang_log.gain_z = 0;
  //     set_ctrl_val(error_entity.w_val, error_entity.w.error_p,
  //                  error_entity.w.error_i, 0, error_entity.w.error_d,
  //                  param_ro->angle_pid.p * error_entity.w.error_p,
  //                  param_ro->angle_pid.i * error_entity.w.error_i, 0,
  //                  param_ro->angle_pid.d * error_entity.w.error_d, 0, 0);
  //   }
  // }
  duty_enc_v_r = duty_enc_v_l = 0;
  if (tgt_val->motion_type != MotionType::NONE) {
    if (param_ro->motor_pid3.mode == 1) {
      duty_enc_v_r = param_ro->motor_pid3.p * error_entity.v_r.error_p +
                     param_ro->motor_pid3.i * error_entity.v_r.error_i +
                     param_ro->motor_pid3.d * error_entity.v_r.error_d;
      duty_enc_v_l = param_ro->motor_pid3.p * error_entity.v_l.error_p +
                     param_ro->motor_pid3.i * error_entity.v_l.error_i +
                     param_ro->motor_pid3.d * error_entity.v_l.error_d;
      error_entity.v_l_log.gain_zz = error_entity.v_l_log.gain_z;
      error_entity.v_l_log.gain_z = 0;
      error_entity.v_r_log.gain_zz = error_entity.v_r_log.gain_z;
      error_entity.v_r_log.gain_z = 0;
    } else if (param_ro->motor_pid3.mode == 2) {
      auto kp_r = param_ro->motor_pid3.p * error_entity.v_r.error_p;
      auto ki_r = param_ro->motor_pid3.i * error_entity.v_r.error_i;
      auto kd_r = param_ro->motor_pid3.d * error_entity.v_r.error_d;
      auto kp_l = param_ro->motor_pid3.p * error_entity.v_l.error_p;
      auto ki_l = param_ro->motor_pid3.i * error_entity.v_l.error_i;
      auto kd_l = param_ro->motor_pid3.d * error_entity.v_l.error_d;
      duty_enc_v_r =
          kp_r + ki_r + kd_r +
          (error_entity.v_r_log.gain_z - error_entity.v_r_log.gain_zz) * dt;
      duty_enc_v_l =
          kp_l + ki_l + kd_l +
          (error_entity.v_l_log.gain_z - error_entity.v_l_log.gain_zz) * dt;
      error_entity.v_l_log.gain_zz = error_entity.v_l_log.gain_z;
      error_entity.v_l_log.gain_z = duty_enc_v_l;
      error_entity.v_r_log.gain_zz = error_entity.v_r_log.gain_z;
      error_entity.v_r_log.gain_z = duty_enc_v_r;

      // set_ctrl_val(error_entity.s_val, error_entity.v_r.error_p,
      //              error_entity.v_r.error_i, 0, error_entity.v_r.error_d,
      //              kp_r, ki_r, 0, kd_r, error_entity.v_r_log.gain_zz,
      //              error_entity.v_r_log.gain_z);
    }
  }

  // tgt_duty.duty_r = (duty_c + duty_c2 + duty_roll + duty_roll2 + duty_rpm_r
  // +
  //                    duty_ff_front + duty_ff_roll + duty_sen) /
  //                   sensing_result->ego.battery_lp * 100;

  // tgt_duty.duty_l = (duty_c + duty_c2 - duty_roll - duty_roll2 + duty_rpm_l
  // +
  //                    duty_ff_front - duty_ff_roll - duty_sen) /
  //                   sensing_result->ego.battery_lp * 100;

  // printf("%0.3f, %0.3f, %0.3f, %0.3f,%0.3f,%0.3f \n", duty_c, duty_c2,
  //        duty_roll, duty_roll2, mpc_next_ego.ff_duty_r, duty_sen);

  auto ff_front = mpc_next_ego.ff_duty_front;
  auto ff_roll = mpc_next_ego.ff_duty_roll;

  if (tgt_val->motion_type == MotionType::WALL_OFF ||
      tgt_val->motion_type == MotionType::WALL_OFF_DIA) {
    ff_front = param_ro->ff_roll_gain_before * ff_front;
    mpc_next_ego.ff_duty_front = ff_front;
  }

  if (tgt_val->motion_type == MotionType::SLALOM) {
    if (tgt_val->ego_in.sla_param.base_alpha > 0) {
      if (tgt_val->ego_in.alpha < 0) {
        ff_roll = param_ro->ff_roll_gain_after * ff_roll;
        sensing_result->ego.duty.ff_duty_roll = ff_roll;
        mpc_next_ego.ff_duty_roll = ff_roll;
      }
    } else if (tgt_val->ego_in.sla_param.base_alpha < 0) {
      if (tgt_val->ego_in.alpha > 0) {
        ff_roll = param_ro->ff_roll_gain_after * ff_roll;
        sensing_result->ego.duty.ff_duty_roll = ff_roll;
        mpc_next_ego.ff_duty_roll = ff_roll;
      }
    }
  }
  auto ff_duty_r = ff_front + ff_roll + mpc_next_ego.ff_duty_rpm_r;
  auto ff_duty_l = ff_front - ff_roll + mpc_next_ego.ff_duty_rpm_l;

  if (param_ro->FF_keV == 0) {
    ff_duty_l = ff_duty_r = 0;
  }

  tgt_duty.duty_r = (duty_c + duty_c2 + duty_enc_v_r + duty_roll + duty_roll2 +
                     ff_duty_r + duty_sen) /
                    sensing_result->ego.battery_lp * 100;

  tgt_duty.duty_l = (duty_c + duty_c2 + duty_enc_v_l - duty_roll - duty_roll2 +
                     ff_duty_l - duty_sen) /
                    sensing_result->ego.battery_lp * 100;

  if (tgt_val->motion_type == MotionType::STRAIGHT ||
      tgt_val->motion_type == MotionType::SLALOM ||
      tgt_val->motion_type == MotionType::SLA_BACK_STR ||
      tgt_val->motion_type == MotionType::SLA_FRONT_STR ||
      tgt_val->motion_type == MotionType::PIVOT) {
    const auto min_duty = param_ro->min_duty;

    if (0 <= tgt_duty.duty_r && tgt_duty.duty_r < min_duty) {
      tgt_duty.duty_r = min_duty;
    } else if (-min_duty < tgt_duty.duty_r && tgt_duty.duty_r <= 0) {
      tgt_duty.duty_r = -min_duty;
    }
    if (0 <= tgt_duty.duty_l && tgt_duty.duty_l < min_duty) {
      tgt_duty.duty_l = min_duty;
    } else if (-min_duty < tgt_duty.duty_l && tgt_duty.duty_l <= 0) {
      tgt_duty.duty_l = -min_duty;
    }
  } else if (tgt_val->motion_type == MotionType::FRONT_CTRL) {
    const auto max_duty = param_ro->sen_ref_p.search_exist.offset_l;
    if (tgt_duty.duty_r > max_duty) {
      tgt_duty.duty_r = max_duty;
    } else if (tgt_duty.duty_r < -max_duty) {
      tgt_duty.duty_r = -max_duty;
    }
    if (tgt_duty.duty_l > max_duty) {
      tgt_duty.duty_l = max_duty;
    } else if (tgt_duty.duty_l < -max_duty) {
      tgt_duty.duty_l = -max_duty;
    }
  } else {
    const auto max_duty = param_ro->max_duty;
    if (tgt_duty.duty_r > max_duty) {
      tgt_duty.duty_r = max_duty;
    } else if (tgt_duty.duty_r < -max_duty) {
      tgt_duty.duty_r = -max_duty;
    }
    if (tgt_duty.duty_l > max_duty) {
      tgt_duty.duty_l = max_duty;
    } else if (tgt_duty.duty_l < -max_duty) {
      tgt_duty.duty_l = -max_duty;
    }
  }

  if (tgt_val->motion_type == MotionType::NONE) {
    tgt_duty.duty_l = tgt_duty.duty_r = 0;
  }
  // if (motor_en) {
  //   printf("%2.2f %2.2f %2.2f %2.2f\n", duty_c, duty_c2, ff_duty_l,
  //   ff_duty_r);
  // }
  if (!motor_en) {
    duty_c = 0;
    duty_c2 = 0;
    duty_roll = 0;
    duty_roll2 = 0;
    duty_sen = 0;
    error_entity.v.error_i = 0;
    error_entity.v.error_d = 0;
    error_entity.v.error_dd = 0;
    error_entity.dist.error_i = 0;
    error_entity.dist.error_d = 0;
    error_entity.dist.error_dd = 0;
    error_entity.w.error_i = 0;
    error_entity.w.error_d = 0;
    error_entity.w.error_dd = 0;
    error_entity.ang.error_i = 0;
    error_entity.ang.error_d = 0;
    error_entity.ang.error_dd = 0;
    error_entity.sen.error_i = 0;
    error_entity.sen.error_d = 0;
    error_entity.sen.error_dd = 0;
    error_entity.sen_dia.error_i = 0;
    error_entity.sen_dia.error_d = 0;
    error_entity.sen_dia.error_dd = 0;
    tgt_duty.duty_r = tgt_duty.duty_l = 0;
    duty_ff_front = duty_ff_roll = 0;
    error_entity.v_log.gain_zz = 0;
    error_entity.v_log.gain_z = 0;
    error_entity.dist_log.gain_zz = 0;
    error_entity.dist_log.gain_z = 0;
    error_entity.w_log.gain_zz = 0;
    error_entity.w_log.gain_z = 0;
    error_entity.ang_log.gain_zz = 0;
    error_entity.ang_log.gain_z = 0;
    error_entity.sen_log.gain_z = 0;
    error_entity.sen_log.gain_zz = 0;

    error_entity.v_l_log.gain_zz = 0;
    error_entity.v_l_log.gain_z = 0;
    error_entity.v_r_log.gain_zz = 0;
    error_entity.v_r_log.gain_z = 0;

    tgt_val->global_pos.ang = 0;
    tgt_val->global_pos.img_ang = 0;
    tgt_val->global_pos.dist = 0;
    tgt_val->global_pos.img_dist = 0;
    kf_dist.reset(0);
    kf_ang.reset(0);
  }
  sensing_result->ego.duty.duty_r = tgt_duty.duty_r;
  sensing_result->ego.duty.duty_l = tgt_duty.duty_l;

  sensing_result->ego.duty.ff_duty_front = mpc_next_ego.ff_duty_front;
  sensing_result->ego.duty.ff_duty_roll = mpc_next_ego.ff_duty_roll;
  sensing_result->ego.duty.ff_duty_rpm_r = mpc_next_ego.ff_duty_rpm_r;
  sensing_result->ego.duty.ff_duty_rpm_l = mpc_next_ego.ff_duty_rpm_l;

  sensing_result->ego.ff_duty.front = duty_ff_front;
  sensing_result->ego.ff_duty.roll = duty_ff_roll;
  w_reset = 1;
  copy_error_entity(error_entity);
}

void PlanningTask::cp_tgt_val() {
  tgt_val->ego_in.accl = mpc_next_ego.accl;
  tgt_val->ego_in.alpha = mpc_next_ego.alpha;
  tgt_val->ego_in.pivot_state = mpc_next_ego.pivot_state;
  tgt_val->ego_in.sla_param = mpc_next_ego.sla_param;
  tgt_val->ego_in.state = mpc_next_ego.state;

  const auto tmp_v = tgt_val->ego_in.v;
  tgt_val->ego_in.v = mpc_next_ego.v;
  if (tgt_val->motion_type == MotionType::SLALOM) {
    if (tgt_val->ego_in.v < 10) {
      tgt_val->ego_in.v = tmp_v;
    }
    sensing_result->sen.r45.sensor_dist = 0;
    sensing_result->sen.l45.sensor_dist = 0;
  }

  tgt_val->ego_in.w = mpc_next_ego.w;
  tgt_val->ego_in.sla_param.state = mpc_next_ego.sla_param.state;
  tgt_val->ego_in.sla_param.counter = mpc_next_ego.sla_param.counter;
  tgt_val->ego_in.sla_param.state = mpc_next_ego.sla_param.state;

  tgt_val->ego_in.img_ang = mpc_next_ego.img_ang;
  tgt_val->ego_in.img_dist = mpc_next_ego.img_dist;

  tgt_val->global_pos.img_ang += mpc_next_ego.w * dt;
  tgt_val->global_pos.img_dist += mpc_next_ego.v * dt;

  tgt_val->ego_in.slip_point.slip_angle = mpc_next_ego.slip_point.slip_angle;

  tgt_val->ego_in.cnt_delay_accl_ratio = mpc_next_ego.cnt_delay_accl_ratio;
  tgt_val->ego_in.cnt_delay_decel_ratio = mpc_next_ego.cnt_delay_decel_ratio;

  // const auto theta = tgt_val->ego_in.img_ang + slip_param.beta;
  // const auto x = tgt_val->ego_in.v * std::cos(theta);
  // const auto y = tgt_val->ego_in.v * std::sin(theta);
  // tgt_val->p.x += x;
  // tgt_val->p.y += y;

  tgt_val->ego_in.slip.beta = mpc_next_ego.slip.beta;
  tgt_val->ego_in.slip.accl = mpc_next_ego.slip.accl;
  tgt_val->ego_in.slip.v = mpc_next_ego.slip.v;
  tgt_val->ego_in.slip.vx = mpc_next_ego.slip.vx;
  tgt_val->ego_in.slip.vy = mpc_next_ego.slip.vy;

  ideal_v_r = tgt_val->ego_in.v - tgt_val->ego_in.w * param_ro->tire_tread / 2;
  ideal_v_l = tgt_val->ego_in.v + tgt_val->ego_in.w * param_ro->tire_tread / 2;
}

void PlanningTask::check_fail_safe() {
  bool no_problem = true;

  if (!motor_en) {
    tgt_val->fss.error = 0;
    return;
  }
  if (ABS(error_entity.ang.error_p) > param_ro->fail_check_ang_th) {
    fail_check_ang++;
  } else {
    fail_check_ang = 0;
  }
  if (tgt_val->motion_type == MotionType::WALL_OFF ||
      tgt_val->motion_type == MotionType::WALL_OFF_DIA) {
    keep_wall_off_cnt++;
  } else {
    keep_wall_off_cnt = 0;
  }

  if (ABS(error_entity.v.error_i) > param_ro->fail_check.v) {
    tgt_val->fss.error = 1;
  }
  if (ABS(error_entity.w.error_i) > param_ro->fail_check.w) {
    tgt_val->fss.error = 1;
  }
  if (fail_check_ang > param_ro->fail_check.ang) {
    tgt_val->fss.error = 1;
  }
  if (keep_wall_off_cnt > param_ro->fail_check.wall_off) {
    tgt_val->fss.error = 1;
  }

  // if (!std::isfinite(tgt_duty.duty_l) || !std::isfinite(tgt_duty.duty_r))
  // {
  //   tgt_val->fss.error = 1;
  // }
}

void PlanningTask::cp_request() {
  // tgt_val->tgt_in.mass = param_ro->Mass;

  pl_req_activate();
  if (motion_req_timestamp == receive_req->nmr.timstamp) {
    return;
  }
  const float dt = param_ro->dt;
  slip_param.K = param_ro->slip_param_K;
  slip_param.k = param_ro->slip_param_k2;
  motion_req_timestamp = receive_req->nmr.timstamp;

  tgt_val->tgt_in.v_max = receive_req->nmr.v_max;

  if (receive_req->nmr.motion_type == MotionType::STRAIGHT ||
      receive_req->nmr.motion_type == MotionType::SLA_FRONT_STR) {
    if (tgt_val->ego_in.v > receive_req->nmr.v_max) {
      tgt_val->tgt_in.v_max = tgt_val->ego_in.v;
    }
  }

  tgt_val->tgt_in.end_v = receive_req->nmr.v_end;
  tgt_val->tgt_in.accl = receive_req->nmr.accl;
  tgt_val->tgt_in.decel = receive_req->nmr.decel;
  tgt_val->tgt_in.w_max = receive_req->nmr.w_max;
  tgt_val->tgt_in.end_w = receive_req->nmr.w_end;
  tgt_val->tgt_in.alpha = receive_req->nmr.alpha;

  tgt_val->tgt_in.tgt_dist = receive_req->nmr.dist;
  tgt_val->tgt_in.tgt_angle = receive_req->nmr.ang;

  tgt_val->motion_mode = (int)(receive_req->nmr.motion_mode);
  tgt_val->motion_type = receive_req->nmr.motion_type;

  tgt_val->ego_in.sla_param.base_alpha = receive_req->nmr.sla_alpha;
  tgt_val->ego_in.sla_param.base_time = receive_req->nmr.sla_time;
  tgt_val->ego_in.sla_param.limit_time_count =
      receive_req->nmr.sla_time * 2 / dt;
  tgt_val->ego_in.sla_param.pow_n = receive_req->nmr.sla_pow_n;

  tgt_val->ego_in.state = 0;
  tgt_val->ego_in.pivot_state = 0;

  tgt_val->dia_state.left_save = receive_req->dia_state.right_save = false;
  tgt_val->dia_state.left_old = receive_req->dia_state.right_old = 0;

  tgt_val->dia_state.left_save = false;
  tgt_val->dia_state.right_save = false;
  tgt_val->dia_state.left_old = tgt_val->dia_state.right_old = 0;
  if (!(tgt_val->motion_type == MotionType::NONE ||
        tgt_val->motion_type == MotionType::STRAIGHT ||
        tgt_val->motion_type == MotionType::PIVOT_PRE ||
        tgt_val->motion_type == MotionType::PIVOT_AFTER ||
        tgt_val->motion_type == MotionType::READY ||
        tgt_val->motion_type == MotionType::SENSING_DUMP ||
        tgt_val->motion_type == MotionType::WALL_OFF)) {
    const auto tmp_ang = tgt_val->ego_in.img_ang;
    tgt_val->ego_in.ang -= tmp_ang;
    tgt_val->ego_in.img_ang = 0;
    kf_ang.offset(-tmp_ang);
  }

  if (tgt_val->motion_type == MotionType::NONE ||
      tgt_val->motion_type == MotionType::READY) {
    tgt_val->global_pos.ang -= tgt_val->global_pos.img_ang;
    tgt_val->global_pos.img_ang = 0;

    tgt_val->global_pos.dist -= tgt_val->global_pos.img_dist;
    tgt_val->global_pos.img_dist = 0;
  }

  right_keep.star_dist = tgt_val->global_pos.dist;
  left_keep.star_dist = tgt_val->global_pos.dist;

  if (tgt_val->tgt_in.tgt_angle != 0) {
    const auto tmp_ang = tgt_val->ego_in.ang;
    tgt_val->ego_in.img_ang -= tmp_ang;
    kf_ang.offset(-tmp_ang);
    tgt_val->ego_in.ang = 0;
    // } else {
    //   kf_ang.reset(0);
  }
  if (tgt_val->tgt_in.tgt_dist != 0) {
    const auto tmp_dist = tgt_val->ego_in.dist;
    tgt_val->ego_in.img_dist -= tmp_dist;
    kf_dist.offset(-tmp_dist);
    tgt_val->ego_in.dist = 0;
    // } else {
    //   kf_dist.reset(0);
  }

  sensing_result->ego.dist_kf = kf_dist.get_state();
  sensing_result->ego.ang_kf = kf_ang.get_state();

  tgt_val->ego_in.sla_param.counter = 1;
  tgt_val->ego_in.sla_param.state = 0;

  tgt_val->motion_dir = receive_req->nmr.motion_dir;
  tgt_val->dia_mode = receive_req->nmr.dia_mode;

  tgt_val->tgt_in.accl_param.limit = 2500;
  tgt_val->tgt_in.accl_param.n = 4;

  tgt_val->tgt_in.slip_gain_K1 = param_ro->slip_param_K;
  tgt_val->tgt_in.slip_gain_K2 = param_ro->slip_param_k2;
  if (receive_req->nmr.motion_type == MotionType::SLALOM) {
    tgt_val->ego_in.v = receive_req->nmr.v_max;
  }
}
float PlanningTask::calc_sensor(float data, float a, float b) {
  int idx = (int)data;
  if (idx <= param_ro->sensor_range_min || idx >= log_table.size()) {
    return param_ro->sensor_range_max;
  }
  auto res = a / log_table.at(idx) - b;
  if (res < param_ro->sensor_range_min || res > param_ro->sensor_range_max) {
    return param_ro->sensor_range_max;
  }
  if (!isfinite(res)) {
    return param_ro->sensor_range_max;
  }
  return res;
}

void PlanningTask::calc_sensor_dist_all() {
  if (!(tgt_val->motion_type == MotionType::NONE ||
        tgt_val->motion_type == MotionType::PIVOT)) {
    sensing_result->ego.left90_dist_old = sensing_result->ego.left90_dist;
    sensing_result->ego.left45_dist_old = sensing_result->ego.left45_dist;
    sensing_result->ego.front_dist_old = sensing_result->ego.front_dist;
    sensing_result->ego.right45_dist_old = sensing_result->ego.right45_dist;
    sensing_result->ego.right90_dist_old = sensing_result->ego.right90_dist;

    sensing_result->ego.left90_dist =
        calc_sensor(sensing_result->ego.left90_lp, param_ro->sensor_gain.l90.a,
                    param_ro->sensor_gain.l90.b);
    sensing_result->ego.left45_dist =
        calc_sensor(sensing_result->ego.left45_lp, param_ro->sensor_gain.l45.a,
                    param_ro->sensor_gain.l45.b);
    sensing_result->ego.right45_dist =
        calc_sensor(sensing_result->ego.right45_lp, param_ro->sensor_gain.r45.a,
                    param_ro->sensor_gain.r45.b);
    sensing_result->ego.right90_dist =
        calc_sensor(sensing_result->ego.right90_lp, param_ro->sensor_gain.r90.a,
                    param_ro->sensor_gain.r90.b);

    sensing_result->ego.left90_far_dist = calc_sensor(
        sensing_result->ego.left90_lp, param_ro->sensor_gain.l90_far.a,
        param_ro->sensor_gain.l90_far.b);
    sensing_result->ego.right90_far_dist = calc_sensor(
        sensing_result->ego.right90_lp, param_ro->sensor_gain.r90_far.a,
        param_ro->sensor_gain.r90_far.b);

    sensing_result->ego.left90_mid_dist = calc_sensor(
        sensing_result->ego.left90_lp, param_ro->sensor_gain.l90_mid.a,
        param_ro->sensor_gain.l90_mid.b);
    sensing_result->ego.right90_mid_dist = calc_sensor(
        sensing_result->ego.right90_lp, param_ro->sensor_gain.r90_mid.a,
        param_ro->sensor_gain.r90_mid.b);

    if (sensing_result->ego.left90_dist < param_ro->sensor_range_far_max &&
        sensing_result->ego.right90_dist < param_ro->sensor_range_far_max) {
      sensing_result->ego.front_dist =
          (sensing_result->ego.left90_dist + sensing_result->ego.right90_dist) /
          2;
    } else if (sensing_result->ego.left90_dist >
                   param_ro->sensor_range_far_max &&
               sensing_result->ego.right90_dist <
                   param_ro->sensor_range_far_max) {
      sensing_result->ego.front_dist = sensing_result->ego.right90_dist;
    } else if (sensing_result->ego.left90_dist <
                   param_ro->sensor_range_far_max &&
               sensing_result->ego.right90_dist >
                   param_ro->sensor_range_far_max) {
      sensing_result->ego.front_dist = sensing_result->ego.left90_dist;
    } else {
      sensing_result->ego.front_dist = param_ro->sensor_range_max;
    }

    if (sensing_result->ego.left90_far_dist < param_ro->sensor_range_far_max &&
        sensing_result->ego.right90_far_dist < param_ro->sensor_range_far_max) {
      sensing_result->ego.front_far_dist =
          (sensing_result->ego.left90_far_dist +
           sensing_result->ego.right90_far_dist) /
          2;
    } else if (sensing_result->ego.left90_far_dist >
                   param_ro->sensor_range_far_max &&
               sensing_result->ego.right90_far_dist <
                   param_ro->sensor_range_far_max) {
      sensing_result->ego.front_far_dist = sensing_result->ego.right90_far_dist;
    } else if (sensing_result->ego.left90_far_dist <
                   param_ro->sensor_range_far_max &&
               sensing_result->ego.right90_far_dist >
                   param_ro->sensor_range_far_max) {
      sensing_result->ego.front_far_dist = sensing_result->ego.left90_far_dist;
    } else {
      sensing_result->ego.front_far_dist = param_ro->sensor_range_max;
    }
    if (sensing_result->ego.left90_mid_dist < param_ro->sensor_range_far_max &&
        sensing_result->ego.right90_mid_dist < param_ro->sensor_range_far_max) {
      sensing_result->ego.front_mid_dist =
          (sensing_result->ego.left90_mid_dist +
           sensing_result->ego.right90_mid_dist) /
          2;
    } else if (sensing_result->ego.left90_mid_dist >
                   param_ro->sensor_range_far_max &&
               sensing_result->ego.right90_mid_dist <
                   param_ro->sensor_range_far_max) {
      sensing_result->ego.front_mid_dist = sensing_result->ego.right90_mid_dist;
    } else if (sensing_result->ego.left90_mid_dist <
                   param_ro->sensor_range_far_max &&
               sensing_result->ego.right90_mid_dist >
                   param_ro->sensor_range_far_max) {
      sensing_result->ego.front_mid_dist = sensing_result->ego.left90_mid_dist;
    } else {
      sensing_result->ego.front_mid_dist = param_ro->sensor_range_max;
    }
  } else {
    sensing_result->ego.left90_dist        //
        = sensing_result->ego.left45_dist  //
        = sensing_result->ego.front_dist   //
        = sensing_result->ego.right45_dist //
        = sensing_result->ego.right90_dist = param_ro->sensor_range_max;
    sensing_result->ego.left45_dist_diff = 0;
    sensing_result->ego.right45_dist_diff = 0;
  }

  sensing_result->ego.left45_dist_diff = ABS(
      sensing_result->ego.left45_dist - sensing_result->ego.left45_dist_old);

  sensing_result->ego.right45_dist_diff = ABS(
      sensing_result->ego.right45_dist - sensing_result->ego.right45_dist_old);

  // 壁からの距離に変換。あとで斜め用に変更
  calc_sensor_dist_diff();
}

void PlanningTask::calc_sensor_dist_diff() {
  if (sensing_result->sen.l45.sensor_dist > sensing_result->ego.left45_dist) {
    sensing_result->sen.l45.sensor_dist = sensing_result->ego.left45_dist;
    sensing_result->sen.l45.global_run_dist = tgt_val->global_pos.dist;
  } else {
    if (((tgt_val->global_pos.dist - sensing_result->sen.l45.global_run_dist) >
         param_ro->wall_off_hold_dist) &&
        sensing_result->ego.left45_dist <
            param_ro->sen_ref_p.normal2.exist.left90) {
      sensing_result->sen.l45.sensor_dist = sensing_result->ego.left45_dist;
    }
  }

  if (sensing_result->sen.r45.sensor_dist > sensing_result->ego.right45_dist) {
    sensing_result->sen.r45.sensor_dist = sensing_result->ego.right45_dist;
    sensing_result->sen.r45.global_run_dist = tgt_val->global_pos.dist;
  } else {
    if (((tgt_val->global_pos.dist - sensing_result->sen.r45.global_run_dist) >
         param_ro->wall_off_hold_dist) &&
        sensing_result->ego.right45_dist <
            param_ro->sen_ref_p.normal2.exist.right90) {
      sensing_result->sen.r45.sensor_dist = sensing_result->ego.right45_dist;
    }
  }
  sen_log.r45_dist = sensing_result->sen.r45.sensor_dist;
  sen_log.l45_dist = sensing_result->sen.r45.sensor_dist;
  sen_log.global_run_dist = tgt_val->global_pos.dist;
  // sensing_result->sen_dist_log.list.push_back(sen_log);
  // if (sensing_result->sen_dist_log.list.size() > param_ro->sen_log_size)
  // {
  //   sensing_result->sen_dist_log.list.pop_front();
  // }
}