#include "include/main_task.hpp"

#define getItem cJSON_GetObjectItem
#define getArray cJSON_GetArrayItem
MainTask::MainTask() {
  ui = std::make_shared<UserInterface>();
  mp = std::make_shared<MotionPlanning>();
  lgc = std::make_shared<MazeSolverBaseLgc>();
  search_ctrl = std::make_shared<SearchController>();
  pc = std::make_shared<PathCreator>();
  mp->set_path_creator(pc);
}

MainTask::~MainTask() {}

void MainTask::create_task(const BaseType_t xCoreID) {
  xTaskCreatePinnedToCore(task_entry_point, "main_task", 8192, this, 2, &handle,
                          xCoreID);
}
void MainTask::task_entry_point(void *task_instance) {
  static_cast<MainTask *>(task_instance)->task();
}
void MainTask::set_sensing_entity(
    std::shared_ptr<sensing_result_entity_t> &_sensing_result) {
  sensing_result = _sensing_result;
  ui->set_sensing_entity(_sensing_result);
  ui->init_i2c_master();
  mp->set_sensing_entity(_sensing_result);
  search_ctrl->set_sensing_entity(_sensing_result);
}
void MainTask::set_input_param_entity(std::shared_ptr<input_param_t> &_param) {
  param = _param;
  mp->set_input_param_entity(_param);
  search_ctrl->set_input_param_entity(_param);
}
void MainTask::set_tgt_val(std::shared_ptr<motion_tgt_val_t> &_tgt_val) {
  tgt_val = _tgt_val;
  ui->set_tgt_val(_tgt_val);
  mp->set_tgt_val(_tgt_val);
}
void MainTask::set_planning_task(std::shared_ptr<PlanningTask> &_pt) {
  pt = _pt;
  search_ctrl->set_planning_task(_pt);
  mp->set_planning_task(_pt);
}
void MainTask::set_logging_task(std::shared_ptr<LoggingTask> &_lt) {
  lt = _lt; //
  search_ctrl->set_logging_task(lt);
  mp->set_logging_task(lt);
}
void MainTask::check_battery() {
  vTaskDelay(1500.0 / portTICK_PERIOD_MS); //他モジュールの起動待ち

  printf("battery= %f\n", sensing_result->ego.battery_raw);
  if (sensing_result->ego.battery_raw > LOW_BATTERY_TH ||
      sensing_result->ego.battery_raw < 6.5)
    return;
  while (1) {
    ui->music_sync(MUSIC::G5_, 250);
    vTaskDelay(tgt_val->buzzer.time / portTICK_PERIOD_MS);
    bool break_btn = ui->button_state_hold();
    if (break_btn) {
      ui->coin(100);
      break;
    }
  }
}

TurnType MainTask::cast_turn_type(std::string str) {
  if (str == "normal")
    return TurnType::Normal;
  if (str == "large")
    return TurnType::Large;
  if (str == "orval")
    return TurnType::Orval;
  if (str == "dia45")
    return TurnType::Dia45;
  if (str == "dia45_2")
    return TurnType::Dia45_2;
  if (str == "dia135")
    return TurnType::Dia135;
  if (str == "dia135_2")
    return TurnType::Dia135_2;
  if (str == "dia90")
    return TurnType::Dia90;
  return TurnType::None;
}
void MainTask::dump1() {
  notify_handle = xTaskGetCurrentTaskHandle();
  mp->reset_gyro_ref_with_check();
  tgt_val->nmr.motion_type = MotionType::READY;
  tgt_val->nmr.timstamp++;
  xQueueReset(*qh);
  xQueueSendToFront(*qh, &tgt_val, 1);
  while (1) {

    printf("%c[2J", ESC);   /* 画面消去 */
    printf("%c[0;0H", ESC); /* 戦闘戻す*/

    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1))) {
      printf("notification recieve\n");
    }

    printf("SW1 %d \n", gpio_get_level(SW1));

    printf("gyro: %d\t(%0.3f)\n", sensing_result->gyro.raw,
           tgt_val->gyro_zero_p_offset);
    printf("accel_x: %f\t(%f)\n", sensing_result->ego.accel_x_raw,
           sensing_result->ego.accel_x_raw / 9806.65 *
               param->accel_x_param.gain);
    printf("accel_y: %d\n", sensing_result->accel_y.raw);
    printf("battery: %0.3f (%d)\n", sensing_result->ego.battery_lp,
           sensing_result->battery.raw);
    printf("encoder: %d, %d\n", sensing_result->encoder.left,
           sensing_result->encoder.right);
    printf(
        "sensor: %4d, %4d, %4d, %4d, %4d, %4d, %4d\n",
        sensing_result->led_sen.left90.raw, sensing_result->led_sen.left45.raw,
        sensing_result->led_sen.left45_2.raw, sensing_result->led_sen.front.raw,
        sensing_result->led_sen.right45_2.raw,
        sensing_result->led_sen.right45.raw,
        sensing_result->led_sen.right90.raw);
    printf("sensor_before: %4d, %4d, %4d, %4d, %4d\n",
           sensing_result->led_sen_before.left90.raw,
           sensing_result->led_sen_before.left45.raw,
           sensing_result->led_sen_before.front.raw,
           sensing_result->led_sen_before.right45.raw,
           sensing_result->led_sen_before.right90.raw);
    printf("sensor_after: %4d, %4d, %4d, %4d, %4d\n",
           sensing_result->led_sen_after.left90.raw,
           sensing_result->led_sen_after.left45.raw,
           sensing_result->led_sen_after.front.raw,
           sensing_result->led_sen_after.right45.raw,
           sensing_result->led_sen_after.right90.raw);
    printf("sensor_dist(near): %3.2f, %3.2f, %3.2f, %3.2f, %3.2f\n",
           sensing_result->ego.left90_dist,  //
           sensing_result->ego.left45_dist,  //
           sensing_result->ego.front_dist,   //
           sensing_result->ego.right45_dist, //
           sensing_result->ego.right90_dist);
    printf("sensor_dist(mid): %3.2f, %3.2f, %3.2f, %3.2f, %3.2f\n",
           sensing_result->ego.left90_mid_dist, //
           sensing_result->ego.left45_dist,     //
           sensing_result->ego.front_mid_dist,  //
           sensing_result->ego.right45_dist,    //
           sensing_result->ego.right90_mid_dist);
    printf("sensor_dist(far): %3.2f, %3.2f, %3.2f, %3.2f, %3.2f\n",
           sensing_result->ego.left90_far_dist, //
           sensing_result->ego.left45_dist,     //
           sensing_result->ego.front_far_dist,  //
           sensing_result->ego.right45_dist,    //
           sensing_result->ego.right90_far_dist);

    printf("sensor: %0.1f, %0.1f, %0.1f, %0.1f, %0.1f\n",
           param->sen_ref_p.search_exist.left90,
           param->sen_ref_p.search_exist.left45,
           param->sen_ref_p.search_exist.front,
           param->sen_ref_p.search_exist.right45,
           param->sen_ref_p.search_exist.right90);

    printf("ego_v: %4.3f, %4.3f, %4.3f, %4.3f, (%4d, %4d)\n",
           sensing_result->ego.v_l, sensing_result->ego.v_c,
           sensing_result->ego.v_r, tgt_val->ego_in.dist,
           sensing_result->encoder.left, sensing_result->encoder.right);

    printf("calc_v: %4.3f, %3.3f\n", tgt_val->ego_in.v, tgt_val->ego_in.w);

    printf("ego_w: %2.3f, %2.3f, %2.3f, %3.3f deg\n", sensing_result->ego.w_raw,
           sensing_result->ego.w_lp, tgt_val->ego_in.ang,
           tgt_val->ego_in.ang * 180 / m_PI);

    printf("gyro_raw[]: %4d, %4d, %4d, %4d, %4d\n",
           sensing_result->gyro_list[0], sensing_result->gyro_list[1],
           sensing_result->gyro_list[2], sensing_result->gyro_list[3],
           sensing_result->gyro_list[4]);

    const float tgt_gain =
        1000.0 /
        (sensing_result->accel_x.raw - tgt_val->accel_x_zero_p_offset) * 9.8;
    printf("accel: %3.3f, %6.6f\n", sensing_result->ego.accel_x_raw, tgt_gain);

    printf("duty: %3.3f, %3.3f\n", sensing_result->ego.duty.duty_l,
           sensing_result->ego.duty.duty_r);

    if (ui->button_state()) {
      tgt_val->ego_in.ang = tgt_val->ego_in.dist = 0;
    }

    vTaskDelay(xDelay100);
  }
}

void MainTask::dump2() {

  tgt_val->nmr.motion_type = MotionType::READY;
  tgt_val->nmr.timstamp++;
  xQueueReset(*qh);
  xQueueSendToFront(*qh, &tgt_val, 1);
  while (1) {
    printf("%d, %d, %d, %d, %d\n", sensing_result->led_sen.left90.raw,
           sensing_result->led_sen.left45.raw,
           sensing_result->led_sen.front.raw,
           sensing_result->led_sen.right45.raw,
           sensing_result->led_sen.right90.raw);

    if (ui->button_state()) {
      tgt_val->ego_in.ang = tgt_val->ego_in.dist = 0;
    }

    vTaskDelay(xDelay100);
  }
}

int MainTask::select_mode() {
  int mode_num = 0;
  lbit.byte = 0;
  while (1) {
    int res = ui->encoder_operation();
    mode_num += res;
    if (mode_num == -1) {
      mode_num = 23;
    } else if (mode_num == 24) {
      mode_num = (int)(MODE::SEARCH);
    }
    lbit.byte = mode_num + 1;
    ui->LED_bit(lbit.b0, lbit.b1, lbit.b2, lbit.b3, lbit.b4, lbit.b5);
    if (ui->button_state_hold()) {
      ui->coin(100);
      break;
    }
    vTaskDelay(xDelay1);
  }
  return mode_num;
}

void MainTask::reset_tgt_data() { mp->reset_tgt_data(); }

void MainTask::reset_ego_data() { mp->reset_ego_data(); }

void MainTask::keep_pivot() {
  mp->reset_gyro_ref_with_check();
  reset_tgt_data();
  reset_ego_data();
  req_error_reset();
  pt->motor_enable();
  req_error_reset();
  ps.v_max = 0.000000001;
  ps.v_end = 0.000000001;
  ps.dist = 1000;
  ps.accl = 0.1;
  ps.decel = -1;
  ps.sct = SensorCtrlType::NONE;
  ps.motion_type = MotionType::STRAIGHT;
  ps.dia_mode = false;
  mp->go_straight(ps);
  while (1) {
    vTaskDelay(1.0 / portTICK_RATE_MS);
    if (ui->button_state_hold()) {
      break;
    }
  }
  pt->motor_disable();
}
void MainTask::echo_sensing_result_with_json() {
  mp->reset_gyro_ref_with_check();
  while (1) {
    // entity_to_json(json_instance);
    // printf("%s\n", json_instance.dump().c_str());
    vTaskDelay(xDelay50);
    bool break_btn = ui->button_state_hold();
    if (break_btn) {
      ui->coin(100);
      break;
    }
  }
}
vector<string> MainTask::split(const string &s, char delim) {
  vector<string> elems;
  stringstream ss(s);
  string item;
  while (getline(ss, item, delim)) {
    if (!item.empty()) {
      elems.push_back(item);
    }
  }
  return elems;
}

void MainTask::save_json_data(std::string &str) {
  mount();
  //「ファイル名@JSON文字列」の体裁でクエリーが来ること前提
  auto res = split(str, '@'); // セパレータ「＠」で分割
  if (res.size() != 2)        // invalid check
    return;

  // mount領域+ファイル名を指定しファイルを開く
  auto tgt_file = "/spiflash/" + res[0];
  auto *f = fopen(tgt_file.c_str(), "wb");
  if (f == NULL)
    return;
  // 書き込み&ファイルclose
  fprintf(f, res[1].c_str());
  fclose(f);
  // printf("%s\n", str.c_str());
  ui->coin(25);
}

void MainTask::load_hw_param() {
  string fileName = "/spiflash/hardware.txt";

  std::ifstream ifs(fileName);
  if (!ifs) {
    return;
  }
  std::string str;
  std::string buf;
  while (!ifs.eof()) {
    std::getline(ifs, buf);
    str += buf;
  }

  // printf("%s\n", str.c_str());

  cJSON *root = cJSON_CreateObject(), *motor_pid, *motor_pid2, *gyro_pid,
        *str_agl_pid, *str_agl_dia_pid, *gyro_param, *kalman_config,
        *battery_param, *led_param, *angle_pid, *dist_pid, *sen_pid,
        *sen_pid_dia, *accel_x, *comp_v_param, *axel_degenerate_x,
        *axel_degenerate_y;
  root = cJSON_Parse(str.c_str());

  param->dt = getItem(root, "dt")->valuedouble;
  param->tire = getItem(root, "tire")->valuedouble;
  param->tire2 = getItem(root, "tire2")->valuedouble;
  param->log_size = getItem(root, "log_size")->valueint;
  param->cell = getItem(root, "cell")->valuedouble;
  param->cell2 = getItem(root, "cell2")->valuedouble;
  param->gear_a = getItem(root, "gear_a")->valuedouble;
  param->gear_b = getItem(root, "gear_b")->valuedouble;
  param->max_duty = getItem(root, "max_duty")->valuedouble;
  param->min_duty = getItem(root, "min_duty")->valuedouble;
  param->Ke = getItem(root, "Ke")->valuedouble;
  param->Km = getItem(root, "Km")->valuedouble;
  param->Resist = getItem(root, "Resist")->valuedouble;
  param->Mass = getItem(root, "Mass")->valuedouble;
  param->Lm = getItem(root, "Lm")->valuedouble;
  param->slip_param_K = getItem(root, "slip_param_K")->valuedouble;
  param->slip_param_k2 = getItem(root, "slip_param_k2")->valuedouble;
  param->sen_log_size = getItem(root, "sen_log_size")->valueint;
  param->offset_start_dist = getItem(root, "offset_start_dist")->valuedouble;
  param->offset_start_dist_search =
      getItem(root, "offset_start_dist_search")->valuedouble;
  param->sakiyomi_time = getItem(root, "sakiyomi_time")->valuedouble;
  param->clear_angle = getItem(root, "clear_angle")->valuedouble;
  param->clear_dist_order = getItem(root, "clear_dist_order")->valuedouble;
  param->front_dist_offset = getItem(root, "front_dist_offset")->valuedouble;
  param->front_dist_offset2 = getItem(root, "front_dist_offset2")->valuedouble;
  param->front_dist_offset3 = getItem(root, "front_dist_offset3")->valuedouble;
  param->front_dist_offset4 = getItem(root, "front_dist_offset4")->valuedouble;
  param->clear_dist_ragne_from =
      getItem(root, "clear_dist_ragne_from")->valuedouble;
  param->clear_dist_ragne_to =
      getItem(root, "clear_dist_ragne_to")->valuedouble;
  param->led_light_delay_cnt =
      getItem(root, "led_light_delay_cnt")->valuedouble;
  param->led_light_delay_cnt2 =
      getItem(root, "led_light_delay_cnt2")->valuedouble;
  param->front_diff_th = getItem(root, "front_diff_th")->valuedouble;

  axel_degenerate_x = getItem(root, "axel_degenerate_x");
  axel_degenerate_y = getItem(root, "axel_degenerate_y");
  int list_size = cJSON_GetArraySize(axel_degenerate_x);
  for (int i = 0; i < list_size; i++) {
    const auto x = cJSON_GetArrayItem(axel_degenerate_x, i)->valuedouble;
    const auto y = cJSON_GetArrayItem(axel_degenerate_y, i)->valuedouble;
    pt->axel_degenerate_x.emplace_back(x);
    pt->axel_degenerate_y.emplace_back(y);
  }

  param->ff_v_th = getItem(root, "ff_v_th")->valuedouble;
  param->ff_front_dury = getItem(root, "ff_front_dury")->valuedouble;

  param->fail_check.duty = getItem(root, "fail_duty_cnt")->valueint;
  param->fail_check.v = getItem(root, "fail_v_cnt")->valueint;
  param->fail_check.w = getItem(root, "fail_w_cnt")->valueint;
  param->seach_timer = getItem(root, "seach_timer")->valueint;
  tgt_val->ego_in.ff_duty_low_th = param->ff_front_dury;
  tgt_val->ego_in.ff_duty_low_v_th = param->ff_v_th;
  param->front_ctrl_error_th =
      getItem(root, "front_ctrl_error_th")->valuedouble;

  param->offset_after_turn_l2 =
      getItem(root, "offset_after_turn_l2")->valuedouble;
  param->offset_after_turn_r2 =
      getItem(root, "offset_after_turn_r2")->valuedouble;
  param->offset_after_turn_l =
      getItem(root, "offset_after_turn_l")->valuedouble;
  param->offset_after_turn_r =
      getItem(root, "offset_after_turn_r")->valuedouble;

  param->offset_after_turn_dia_l =
      getItem(root, "offset_after_turn_dia_l")->valuedouble;
  param->offset_after_turn_dia_r =
      getItem(root, "offset_after_turn_dia_r")->valuedouble;

  param->dia_turn_th_l = getItem(root, "dia_turn_th_l")->valuedouble;
  param->dia_turn_th_r = getItem(root, "dia_turn_th_r")->valuedouble;

  param->logging_time =
      getItem(root, "logging_time")->valuedouble / portTICK_PERIOD_MS;
  param->set_param = true;

  param->front_dist_offset_pivot_th =
      getItem(root, "front_dist_offset_pivot_th")->valuedouble;
  param->front_dist_offset_pivot =
      getItem(root, "front_dist_offset_pivot")->valuedouble;

  param->search_log_enable = getItem(root, "search_log_enable")->valueint;
  param->test_log_enable = getItem(root, "test_log_enable")->valueint;
  param->fast_log_enable = getItem(root, "fast_log_enable")->valueint;

  param->wall_off_hold_dist = getItem(root, "wall_off_hold_dist")->valuedouble;
  param->wall_off_dist.left_str =
      getItem(root, "wall_off_hold_dist_str_l")->valuedouble;
  param->wall_off_dist.right_str =
      getItem(root, "wall_off_hold_dist_str_r")->valuedouble;
  param->wall_off_dist.left_str_exist =
      getItem(root, "wall_off_hold_dist_str_l_exist")->valuedouble;
  param->wall_off_dist.right_str_exist =
      getItem(root, "wall_off_hold_dist_str_r_exist")->valuedouble;

  param->wall_off_dist.left_dia =
      getItem(root, "wall_off_hold_dist_dia_l")->valuedouble;
  param->wall_off_dist.right_dia =
      getItem(root, "wall_off_hold_dist_dia_r")->valuedouble;
  param->wall_off_dist.left_dia2 =
      getItem(root, "wall_off_hold_dist_dia_l2")->valuedouble;
  param->wall_off_dist.right_dia2 =
      getItem(root, "wall_off_hold_dist_dia_r2")->valuedouble;

  param->wall_off_dist.exist_dist_l =
      getItem(root, "wall_off_hold_exist_dist_l")->valuedouble;
  param->wall_off_dist.exist_dist_r =
      getItem(root, "wall_off_hold_exist_dist_r")->valuedouble;
  param->wall_off_dist.wall_off_exist_wall_th_l =
      getItem(root, "wall_off_exist_wall_th_l")->valuedouble;
  param->wall_off_dist.wall_off_exist_wall_th_r =
      getItem(root, "wall_off_exist_wall_th_r")->valuedouble;

  param->wall_off_dist.ctrl_exist_wall_th_l =
      getItem(root, "ctrl_exist_wall_th_l")->valuedouble;
  param->wall_off_dist.ctrl_exist_wall_th_r =
      getItem(root, "ctrl_exist_wall_th_r")->valuedouble;

  param->wall_off_dist.noexist_th_l =
      getItem(root, "wall_off_hold_noexist_th_l")->valuedouble;
  param->wall_off_dist.noexist_th_r =
      getItem(root, "wall_off_hold_noexist_th_r")->valuedouble;

  param->wall_off_dist.noexist_th_l2 =
      getItem(root, "wall_off_hold_noexist_th_l2")->valuedouble;
  param->wall_off_dist.noexist_th_r2 =
      getItem(root, "wall_off_hold_noexist_th_r2")->valuedouble;

  param->wall_off_dist.exist_dia_th_l =
      getItem(root, "wall_off_hold_exist_dist_dia_l")->valuedouble;
  param->wall_off_dist.exist_dia_th_r =
      getItem(root, "wall_off_hold_exist_dist_dia_r")->valuedouble;

  param->wall_off_dist.noexist_dia_th_l =
      getItem(root, "wall_off_hold_noexist_dia_th_l")->valuedouble;
  param->wall_off_dist.noexist_dia_th_r =
      getItem(root, "wall_off_hold_noexist_dia_th_r")->valuedouble;

  param->dia_wall_off_ref_l = getItem(root, "dia_wall_off_ref_l")->valuedouble;
  param->dia_wall_off_ref_r = getItem(root, "dia_wall_off_ref_r")->valuedouble;
  param->dia_offset_max_dist =
      getItem(root, "dia_offset_max_dist")->valuedouble;

  param->sla_wall_ref_l = getItem(root, "sla_wall_ref_l")->valuedouble;
  param->sla_wall_ref_r = getItem(root, "sla_wall_ref_r")->valuedouble;
  param->sla_max_offset_dist =
      getItem(root, "sla_max_offset_dist")->valuedouble;

  param->sla_wall_ref_l_orval =
      getItem(root, "sla_wall_ref_l_orval")->valuedouble;
  param->sla_wall_ref_r_orval =
      getItem(root, "sla_wall_ref_r_orval")->valuedouble;
  param->orval_enable = getItem(root, "orval_offset_enable")->valueint;
  param->dia45_offset_enable = getItem(root, "dia45_offset_enable")->valueint;

  param->front_dist_offset_dia_front =
      getItem(root, "front_dist_offset_dia_front")->valuedouble;
  param->front_dist_offset_dia_45_th =
      getItem(root, "front_dist_offset_dia_45_th")->valuedouble;
  param->front_dist_offset_dia_right45 =
      getItem(root, "front_dist_offset_dia_right45")->valuedouble;
  param->front_dist_offset_dia_left45 =
      getItem(root, "front_dist_offset_dia_left45")->valuedouble;

  param->FF_front = getItem(root, "FF_front")->valueint;
  param->FF_roll = getItem(root, "FF_roll")->valueint;
  param->FF_keV = getItem(root, "FF_keV")->valueint;

  motor_pid = getItem(root, "motor_pid");
  param->motor_pid.p = getItem(motor_pid, "p")->valuedouble;
  param->motor_pid.i = getItem(motor_pid, "i")->valuedouble;
  param->motor_pid.d = getItem(motor_pid, "d")->valuedouble;
  param->motor_pid.b = getItem(motor_pid, "b")->valuedouble;
  param->motor_pid.c = getItem(motor_pid, "c")->valuedouble;
  param->motor_pid.mode = getItem(motor_pid, "mode")->valueint;

  str_agl_pid = getItem(root, "str_agl_pid");
  param->str_ang_pid.p = getItem(str_agl_pid, "p")->valuedouble;
  param->str_ang_pid.i = getItem(str_agl_pid, "i")->valuedouble;
  param->str_ang_pid.d = getItem(str_agl_pid, "d")->valuedouble;
  param->str_ang_pid.b = getItem(str_agl_pid, "b")->valuedouble;
  param->str_ang_pid.c = getItem(str_agl_pid, "c")->valuedouble;
  param->str_ang_pid.mode = getItem(str_agl_pid, "mode")->valueint;

  str_agl_pid = getItem(root, "str_agl_dia_pid");
  param->str_ang_dia_pid.p = getItem(str_agl_pid, "p")->valuedouble;
  param->str_ang_dia_pid.i = getItem(str_agl_pid, "i")->valuedouble;
  param->str_ang_dia_pid.d = getItem(str_agl_pid, "d")->valuedouble;
  param->str_ang_dia_pid.b = getItem(str_agl_pid, "b")->valuedouble;
  param->str_ang_dia_pid.c = getItem(str_agl_pid, "c")->valuedouble;
  param->str_ang_dia_pid.mode = getItem(str_agl_pid, "mode")->valueint;

  motor_pid2 = getItem(root, "motor_pid2");
  param->motor_pid2.p = getItem(motor_pid2, "p")->valuedouble;
  param->motor_pid2.i = getItem(motor_pid2, "i")->valuedouble;
  param->motor_pid2.d = getItem(motor_pid2, "d")->valuedouble;
  param->motor_pid2.b = getItem(motor_pid2, "b")->valuedouble;
  param->motor_pid2.c = getItem(motor_pid2, "c")->valuedouble;
  param->motor_pid2.mode = getItem(motor_pid2, "mode")->valueint;

  sen_pid = getItem(root, "sensor_pid");
  param->sensor_pid.p = getItem(sen_pid, "p")->valuedouble;
  param->sensor_pid.i = getItem(sen_pid, "i")->valuedouble;
  param->sensor_pid.d = getItem(sen_pid, "d")->valuedouble;
  param->sensor_pid.b = getItem(sen_pid, "b")->valuedouble;
  param->sensor_pid.mode = getItem(sen_pid, "mode")->valueint;

  sen_pid_dia = getItem(root, "sensor_pid_dia");
  param->sensor_pid_dia.p = getItem(sen_pid_dia, "p")->valuedouble;
  param->sensor_pid_dia.i = getItem(sen_pid_dia, "i")->valuedouble;
  param->sensor_pid_dia.d = getItem(sen_pid_dia, "d")->valuedouble;
  param->sensor_pid_dia.mode = getItem(sen_pid_dia, "mode")->valueint;

  dist_pid = getItem(root, "dist_pid");
  param->dist_pid.p = getItem(dist_pid, "p")->valuedouble;
  param->dist_pid.i = getItem(dist_pid, "i")->valuedouble;
  param->dist_pid.d = getItem(dist_pid, "d")->valuedouble;
  param->dist_pid.mode = getItem(dist_pid, "mode")->valueint;

  gyro_pid = getItem(root, "gyro_pid");
  param->gyro_pid.p = getItem(gyro_pid, "p")->valuedouble;
  param->gyro_pid.i = getItem(gyro_pid, "i")->valuedouble;
  param->gyro_pid.d = getItem(gyro_pid, "d")->valuedouble;
  param->gyro_pid.b = getItem(gyro_pid, "b")->valuedouble;
  param->gyro_pid.c = getItem(gyro_pid, "c")->valuedouble;
  param->gyro_pid.mode = getItem(gyro_pid, "mode")->valueint;

  angle_pid = getItem(root, "angle_pid");
  param->angle_pid.p = getItem(angle_pid, "p")->valuedouble;
  param->angle_pid.i = getItem(angle_pid, "i")->valuedouble;
  param->angle_pid.d = getItem(angle_pid, "d")->valuedouble;
  param->angle_pid.mode = getItem(angle_pid, "mode")->valueint;

  gyro_param = getItem(root, "gyro_param");
  param->gyro_param.gyro_w_gain_right =
      getItem(gyro_param, "gyro_w_gain_right")->valuedouble;
  param->gyro_param.gyro_w_gain_left =
      getItem(gyro_param, "gyro_w_gain_left")->valuedouble;
  param->gyro_param.lp_delay = getItem(gyro_param, "lp_delay")->valuedouble;

  accel_x = getItem(root, "accel_x_param");
  param->accel_x_param.gain = getItem(accel_x, "gain")->valuedouble;

  battery_param = getItem(root, "battery_param");
  param->battery_param.lp_delay =
      getItem(battery_param, "lp_delay")->valuedouble;

  led_param = getItem(root, "led_param");
  param->led_param.lp_delay = getItem(led_param, "lp_delay")->valuedouble;

  kalman_config = getItem(root, "kalman_config");
  param->Kalman_ang = getItem(kalman_config, "q_ang")->valuedouble;
  param->Kalman_bias = getItem(kalman_config, "q_bias")->valuedouble;
  param->Kalman_measure = getItem(kalman_config, "r_measure")->valuedouble;

  comp_v_param = getItem(root, "comp_v_param");
  param->comp_param.v_lp_gain = getItem(comp_v_param, "enc_v_lp")->valuedouble;
  param->comp_param.accl_x_hp_gain =
      getItem(comp_v_param, "acc_x_hp")->valuedouble;
  param->comp_param.gain = getItem(comp_v_param, "gain_v")->valuedouble;
  param->comp_param.enable = getItem(comp_v_param, "enable")->valueint;

  pt->dynamics.mass = param->Mass;
  pt->dynamics.lm = param->Lm;
  pt->dynamics.km = param->Km;
  pt->dynamics.resist = param->Resist;
  pt->dynamics.tread = param->tread;
  pt->dynamics.ke = param->Ke;
  pt->dynamics.tire = param->tire;
  pt->dynamics.gear_ratio = param->gear_a / param->gear_b;

  // cJSON_free(motor_pid);
  // cJSON_free(gyro_pid);
  // cJSON_free(sen_pid);
  // cJSON_free(sen_pid_dia);
  // cJSON_free(gyro_param);
  // cJSON_free(battery_param);
  // cJSON_free(led_param);
  // cJSON_free(dist_pid);
  // cJSON_free(angle_pid);
  // cJSON_free(kalman_config);
  // cJSON_free(accel_x);
  // cJSON_free(comp_v_param);
  cJSON_Delete(root);
}

void MainTask::load_sensor_param() {
  string fileName = "/spiflash/sensor.txt";
  std::ifstream ifs(fileName);
  if (!ifs) {
    return;
  }
  std::string str;
  std::string buf;
  while (!ifs.eof()) {
    std::getline(ifs, buf);
    str += buf;
  }
  cJSON *root = cJSON_CreateObject(), *normal, *normal_ref, *normal_exist, *dia,
        *normal2, *normal2_ref, *normal2_exist, *dia_ref, *dia_exist, *search,
        *search_ref, *search_exist, *gain;
  root = cJSON_Parse(str.c_str());

  normal = getItem(root, "normal");
  normal_ref = getItem(normal, "ref");
  normal_exist = getItem(normal, "exist");
  normal2 = getItem(root, "normal2");
  normal2_ref = getItem(normal2, "ref");
  normal2_exist = getItem(normal2, "exist");
  param->sen_ref_p.normal.ref.right45 =
      getItem(normal_ref, "right45")->valuedouble;
  param->sen_ref_p.normal.ref.left45 =
      getItem(normal_ref, "left45")->valuedouble;
  param->sen_ref_p.normal.ref.kireme_r =
      getItem(normal_ref, "kireme_r")->valuedouble;
  param->sen_ref_p.normal.ref.kireme_l =
      getItem(normal_ref, "kireme_l")->valuedouble;

  param->sen_ref_p.normal.exist.right45 =
      getItem(normal_exist, "right45")->valuedouble;
  param->sen_ref_p.normal.exist.left45 =
      getItem(normal_exist, "left45")->valuedouble;
  param->sen_ref_p.normal.exist.front =
      getItem(normal_exist, "front")->valuedouble;
  param->sen_ref_p.normal.exist.right90 =
      getItem(normal_exist, "right90")->valuedouble;
  param->sen_ref_p.normal.exist.left90 =
      getItem(normal_exist, "left90")->valuedouble;

  param->sen_ref_p.normal2.ref.right45 =
      getItem(normal2_ref, "right45")->valuedouble;
  param->sen_ref_p.normal2.ref.left45 =
      getItem(normal2_ref, "left45")->valuedouble;
  param->sen_ref_p.normal2.ref.kireme_r =
      getItem(normal2_ref, "kireme_r")->valuedouble;
  param->sen_ref_p.normal2.ref.kireme_l =
      getItem(normal2_ref, "kireme_l")->valuedouble;

  param->sen_ref_p.normal2.exist.right45 =
      getItem(normal2_exist, "right45")->valuedouble;
  param->sen_ref_p.normal2.exist.left45 =
      getItem(normal2_exist, "left45")->valuedouble;
  param->sen_ref_p.normal2.exist.front =
      getItem(normal2_exist, "front")->valuedouble;
  param->sen_ref_p.normal2.exist.right90 =
      getItem(normal2_exist, "right90")->valuedouble;
  param->sen_ref_p.normal2.exist.left90 =
      getItem(normal2_exist, "left90")->valuedouble;

  printf("normal2.exist.left90=%f\n", param->sen_ref_p.normal2.exist.left90);
  printf("normal2.exist.right90=%f\n", param->sen_ref_p.normal2.exist.right90);

  dia = getItem(root, "dia");
  dia_ref = getItem(dia, "ref");
  dia_exist = getItem(dia, "exist");
  param->sen_ref_p.dia.ref.right90 = getItem(dia_ref, "right90")->valuedouble;
  param->sen_ref_p.dia.ref.left90 = getItem(dia_ref, "left90")->valuedouble;
  param->sen_ref_p.dia.ref.kireme_r = getItem(dia_ref, "kireme_r")->valuedouble;
  param->sen_ref_p.dia.ref.kireme_l = getItem(dia_ref, "kireme_l")->valuedouble;

  param->sen_ref_p.dia.exist.right90 =
      getItem(dia_exist, "right90")->valuedouble;
  param->sen_ref_p.dia.exist.left90 = getItem(dia_exist, "left90")->valuedouble;

  search = getItem(root, "search");
  search_exist = getItem(search, "exist");
  param->sen_ref_p.search_exist.front =
      getItem(search_exist, "front")->valuedouble;
  param->sen_ref_p.search_exist.right45 =
      getItem(search_exist, "right45")->valuedouble;
  param->sen_ref_p.search_exist.right90 =
      getItem(search_exist, "right90")->valuedouble;
  param->sen_ref_p.search_exist.left45 =
      getItem(search_exist, "left45")->valuedouble;
  param->sen_ref_p.search_exist.left90 =
      getItem(search_exist, "left90")->valuedouble;
  param->sen_ref_p.search_exist.kireme_r =
      getItem(search_exist, "kireme_r")->valuedouble;
  param->sen_ref_p.search_exist.kireme_l =
      getItem(search_exist, "kireme_l")->valuedouble;
  param->sen_ref_p.search_exist.offset_r =
      getItem(search_exist, "offset_r")->valuedouble;
  param->sen_ref_p.search_exist.offset_l =
      getItem(search_exist, "offset_l")->valuedouble;
  param->sen_ref_p.search_exist.front_ctrl_th =
      getItem(search_exist, "front_ctrl_th")->valuedouble;
  param->sen_ref_p.search_exist.front_ctrl =
      getItem(search_exist, "front_ctrl")->valuedouble;

  search_ref = getItem(search, "ref");
  param->sen_ref_p.search_ref.right45 =
      getItem(search_ref, "right45")->valuedouble;
  param->sen_ref_p.search_ref.left45 =
      getItem(search_ref, "left45")->valuedouble;
  param->sen_ref_p.search_ref.right90 =
      getItem(search_ref, "right90")->valuedouble;
  param->sen_ref_p.search_ref.left90 =
      getItem(search_ref, "left90")->valuedouble;

  gain = getItem(root, "gain");
  param->sensor_gain.l90.a =
      cJSON_GetArrayItem(getItem(gain, "L90_near"), 0)->valuedouble;
  param->sensor_gain.l90.b =
      cJSON_GetArrayItem(getItem(gain, "L90_near"), 1)->valuedouble;
  param->sensor_gain.l90_far.a =
      cJSON_GetArrayItem(getItem(gain, "L90_far"), 0)->valuedouble;
  param->sensor_gain.l90_far.b =
      cJSON_GetArrayItem(getItem(gain, "L90_far"), 1)->valuedouble;
  param->sensor_gain.l90_mid.a =
      cJSON_GetArrayItem(getItem(gain, "L90_mid"), 0)->valuedouble;
  param->sensor_gain.l90_mid.b =
      cJSON_GetArrayItem(getItem(gain, "L90_mid"), 1)->valuedouble;
  param->sensor_gain.l45.a =
      cJSON_GetArrayItem(getItem(gain, "L45"), 0)->valuedouble;
  param->sensor_gain.l45.b =
      cJSON_GetArrayItem(getItem(gain, "L45"), 1)->valuedouble;
  param->sensor_gain.front.a =
      cJSON_GetArrayItem(getItem(gain, "F"), 0)->valuedouble;
  param->sensor_gain.front.b =
      cJSON_GetArrayItem(getItem(gain, "F"), 1)->valuedouble;
  param->sensor_gain.front2.a =
      cJSON_GetArrayItem(getItem(gain, "F2"), 0)->valuedouble;
  param->sensor_gain.front2.b =
      cJSON_GetArrayItem(getItem(gain, "F2"), 1)->valuedouble;
  param->sensor_gain.front3.a =
      cJSON_GetArrayItem(getItem(gain, "F3"), 0)->valuedouble;
  param->sensor_gain.front3.b =
      cJSON_GetArrayItem(getItem(gain, "F3"), 1)->valuedouble;
  param->sensor_gain.front4.a =
      cJSON_GetArrayItem(getItem(gain, "F4"), 0)->valuedouble;
  param->sensor_gain.front4.b =
      cJSON_GetArrayItem(getItem(gain, "F4"), 1)->valuedouble;
  param->sensor_gain.r45.a =
      cJSON_GetArrayItem(getItem(gain, "R45"), 0)->valuedouble;
  param->sensor_gain.r45.b =
      cJSON_GetArrayItem(getItem(gain, "R45"), 1)->valuedouble;
  param->sensor_gain.r90.a =
      cJSON_GetArrayItem(getItem(gain, "R90_near"), 0)->valuedouble;
  param->sensor_gain.r90.b =
      cJSON_GetArrayItem(getItem(gain, "R90_near"), 1)->valuedouble;
  param->sensor_gain.r90_far.a =
      cJSON_GetArrayItem(getItem(gain, "R90_far"), 0)->valuedouble;
  param->sensor_gain.r90_far.b =
      cJSON_GetArrayItem(getItem(gain, "R90_far"), 1)->valuedouble;
  param->sensor_gain.r90_mid.a =
      cJSON_GetArrayItem(getItem(gain, "R90_mid"), 0)->valuedouble;
  param->sensor_gain.r90_mid.b =
      cJSON_GetArrayItem(getItem(gain, "R90_mid"), 1)->valuedouble;

  // cJSON_free(normal);
  // cJSON_free(normal_ref);
  // cJSON_free(normal_exist);
  // cJSON_free(normal2);
  // cJSON_free(normal2_ref);
  // cJSON_free(normal2_exist);
  // cJSON_free(dia);
  // cJSON_free(dia_ref);
  // cJSON_free(dia_exist);
  // cJSON_free(search);
  // cJSON_free(search_ref);
  // cJSON_free(search_exist);
  // cJSON_free(gain);
  cJSON_Delete(root);
}

void MainTask::load_sys_param() {
  string fileName = "/spiflash/system.txt";
  std::ifstream ifs(fileName);
  if (!ifs) {
    return;
  }
  std::string str;
  std::string buf;
  while (!ifs.eof()) {
    std::getline(ifs, buf);
    str += buf;
  }

  cJSON *root = cJSON_CreateObject(), *test, *goals;
  root = cJSON_Parse(str.c_str());

  sys.goals.clear();
  goals = getItem(root, "goals");
  int goal_size = cJSON_GetArraySize(goals);
  for (int i = 0; i < goal_size; i++) {
    point_t pt;
    pt.x = cJSON_GetArrayItem(cJSON_GetArrayItem(goals, i), 0)->valueint;
    pt.y = cJSON_GetArrayItem(cJSON_GetArrayItem(goals, i), 1)->valueint;
    sys.goals.emplace_back(pt);
    printf("%u %u\n", pt.x, pt.y);
  }
  sys.maze_size = getItem(root, "maze_size")->valueint;
  sys.user_mode = getItem(root, "mode")->valueint;
  test = getItem(root, "test");

  sys.test.v_max = getItem(test, "v_max")->valuedouble;
  sys.test.dia = getItem(test, "dia")->valuedouble;
  sys.test.end_v = getItem(test, "end_v")->valuedouble;
  sys.test.accl = getItem(test, "accl")->valuedouble;
  sys.test.decel = getItem(test, "decel")->valuedouble;
  sys.test.dist = getItem(test, "dist")->valuedouble;
  sys.test.w_max = getItem(test, "w_max")->valuedouble;
  // sys.test.w_end = getItem(test, "w_end")->valuedouble;
  sys.test.alpha = getItem(test, "alpha")->valuedouble;
  sys.test.ang = getItem(test, "ang")->valuedouble;
  sys.test.suction_active = getItem(test, "suction_active")->valueint;
  sys.test.suction_duty = getItem(test, "suction_duty")->valuedouble;
  sys.test.suction_duty_low = getItem(test, "suction_duty_low")->valuedouble;
  sys.test.suction_gain = getItem(test, "suction_gain")->valuedouble;
  pt->suction_gain = sys.test.suction_gain;

  sys.test.file_idx = getItem(test, "file_idx")->valueint;
  printf("sys.test.file_idx = %d\n", sys.test.file_idx);
  file_idx = sys.test.file_idx;
  sys.test.sla_type = getItem(test, "sla_type")->valueint;
  sys.test.sla_return = getItem(test, "sla_return")->valueint;
  sys.test.sla_type2 = getItem(test, "sla_type2")->valueint;
  sys.test.sla_dist = getItem(test, "sla_dist")->valuedouble;
  sys.test.turn_times = getItem(test, "turn_times")->valueint;
  sys.test.ignore_opp_sen = getItem(test, "ignore_opp_sen")->valueint;

  sys.test.sysid_test_mode = getItem(test, "sysid_test_mode")->valueint;
  sys.test.sysid_duty = getItem(test, "sysid_duty")->valuedouble;
  sys.test.sysid_time = getItem(test, "sysid_time")->valuedouble;
  sys.test.start_turn = getItem(test, "start_turn")->valueint;

  // cJSON_free(goals);
  // cJSON_free(test);
  cJSON_Delete(root);
}

void MainTask::load_turn_param_profiles(bool const_mode) {
  string fileName = "/spiflash/profiles.txt";
  std::ifstream ifs(fileName);
  if (!ifs) {
    return;
  }
  std::string str;
  std::string buf;
  while (!ifs.eof()) {
    std::getline(ifs, buf);
    str += buf;
  }

  cJSON *root = cJSON_CreateObject(), *profile_list, *profile_idx;
  root = cJSON_Parse(str.c_str());

  tpp.file_list.clear();
  profile_list = getItem(root, "list");
  int profile_list_size = cJSON_GetArraySize(profile_list);
  printf("profile_list\n");
  tpp.file_list_size = 0;
  for (int i = 0; i < profile_list_size; i++) {
    tpp.file_list.emplace_back(
        cJSON_GetArrayItem(profile_list, i)->valuestring);
    tpp.file_list_size++;
  }
  printf("tpp.file_list.size() = %d\n", tpp.file_list.size());

  tpp.profile_idx_size = getItem(root, "profile_idx_size")->valueint;
  printf("tpp.profile_idx_size= %d\n", tpp.profile_idx_size);

  tpp.profile_list.clear();
  profile_idx = getItem(root, "profile_idx");
  int profile_idx_size = cJSON_GetArraySize(profile_idx);

  for (int i = 0; i < profile_idx_size; i++) {
    p_idx[TurnType::None] =
        getItem(getArray(profile_idx, i), "run_param")->valueint;
    p_idx[TurnType::Finish] =
        getItem(getArray(profile_idx, i), "suction")->valueint;
    p_idx[TurnType::Normal] =
        getItem(getArray(profile_idx, i), "normal")->valueint;
    p_idx[TurnType::Large] =
        getItem(getArray(profile_idx, i), "large")->valueint;
    p_idx[TurnType::Orval] =
        getItem(getArray(profile_idx, i), "orval")->valueint;
    p_idx[TurnType::Dia45] =
        getItem(getArray(profile_idx, i), "dia45")->valueint;
    p_idx[TurnType::Dia45_2] =
        getItem(getArray(profile_idx, i), "dia45_2")->valueint;
    p_idx[TurnType::Dia135] =
        getItem(getArray(profile_idx, i), "dia135")->valueint;
    p_idx[TurnType::Dia135_2] =
        getItem(getArray(profile_idx, i), "dia135_2")->valueint;
    p_idx[TurnType::Dia90] =
        getItem(getArray(profile_idx, i), "dia90")->valueint;
    if (const_mode) {
      p_idx[TurnType::None] = i;
      p_idx[TurnType::Finish] = i;
      p_idx[TurnType::Normal] = i;
      p_idx[TurnType::Large] = i;
      p_idx[TurnType::Orval] = i;
      p_idx[TurnType::Dia45] = i;
      p_idx[TurnType::Dia45_2] = i;
      p_idx[TurnType::Dia135] = i;
      p_idx[TurnType::Dia135_2] = i;
      p_idx[TurnType::Dia90] = i;
    }
    tpp.profile_list.emplace_back(p_idx);
  }
  // cJSON_free(profile_list);
  // cJSON_free(profile_idx);
  cJSON_Delete(root);
}

void MainTask::load_sla(int idx, string turn_name, slalom_param2_t &sla_p) {}
void MainTask::load_straight(
    int idx, std::unordered_map<StraightType, straight_param_t> &str_map) {
  mount();
  if ((int)(tpp.file_list.size()) < (idx - 1)) {
    return;
  }
  const auto file_name = tpp.file_list[idx];
  const auto path = std::string("/spiflash/" + file_name);

  std::ifstream ifs(path);
  std::string str = "";
  std::string buf;
  while (!ifs.eof()) {
    std::getline(ifs, buf);
    str += buf;
  }
  ifs.close();
  cJSON *root = cJSON_CreateObject();
  root = cJSON_Parse(str.c_str());
  str.shrink_to_fit();
  const auto key = "straight";
  for (const auto &p2 : straight_name_list) {
    str_p.v_max =
        getItem(getItem(getItem(root, key), p2.second.c_str()), "v_max")
            ->valuedouble;
    str_p.accl = getItem(getItem(getItem(root, key), p2.second.c_str()), "accl")
                     ->valuedouble;
    str_p.decel =
        getItem(getItem(getItem(root, key), p2.second.c_str()), "decel")
            ->valuedouble;
    str_p.w_max =
        getItem(getItem(getItem(root, key), p2.second.c_str()), "w_max")
            ->valuedouble;
    str_p.w_end =
        getItem(getItem(getItem(root, key), p2.second.c_str()), "w_end")
            ->valuedouble;
    str_p.alpha =
        getItem(getItem(getItem(root, key), p2.second.c_str()), "alpha")
            ->valuedouble;
    str_map[p2.first] = str_p;
  }
  cJSON_Delete(root);
  umount();
}

void MainTask::load_slas(
    int idx, vector<pair<TurnType, string>> &turn_list,
    std::unordered_map<TurnType, slalom_param2_t> &turn_map) {
  mount();
  if ((int)(tpp.file_list.size()) < (idx - 1)) {
    umount();
    return;
  }
  const auto file_name = tpp.file_list[idx];
  const auto path = std::string("/spiflash/" + file_name);

  std::ifstream ifs(path);
  std::string str = "";
  std::string buf;
  while (!ifs.eof()) {
    std::getline(ifs, buf);
    str += buf;
  }
  ifs.close();
  cJSON *root = cJSON_CreateObject();
  root = cJSON_Parse(str.c_str());
  str.shrink_to_fit();
  printf("%s\n", file_name.c_str());
  for (const auto &p : turn_list) {
    printf(" - %s\n", p.second.c_str());
    turn_map[p.first].v =
        getItem(getItem(root, p.second.c_str()), "v")->valuedouble;
    printf(" - v: %f\n", turn_map[p.first].v);
    turn_map[p.first].ang =
        getItem(getItem(root, p.second.c_str()), "ang")->valuedouble;
    turn_map[p.first].ang = m_PI * turn_map[p.first].ang / 180;

    turn_map[p.first].rad =
        getItem(getItem(root, p.second.c_str()), "rad")->valuedouble;
    turn_map[p.first].pow_n =
        getItem(getItem(root, p.second.c_str()), "pow_n")->valueint;
    turn_map[p.first].time =
        getItem(getItem(root, p.second.c_str()), "time")->valuedouble;
    turn_map[p.first].front.right =
        getItem(getItem(getItem(root, p.second.c_str()), "front"), "right")
            ->valuedouble;
    turn_map[p.first].front.left =
        getItem(getItem(getItem(root, p.second.c_str()), "front"), "left")
            ->valuedouble;
    turn_map[p.first].back.right =
        getItem(getItem(getItem(root, p.second.c_str()), "back"), "right")
            ->valuedouble;
    turn_map[p.first].back.left =
        getItem(getItem(getItem(root, p.second.c_str()), "back"), "left")
            ->valuedouble;
    turn_map[p.first].type = cast_turn_type(p.second);
  }
  cJSON_Delete(root);
  umount();
}
void MainTask::load_slalom_param(int idx, int idx2) {
  mount();
  param_set.suction = tpp.profile_list[idx][TurnType::Finish] > 0;
  param_set.suction_duty = sys.test.suction_duty;
  param_set.suction_duty_low = sys.test.suction_duty_low;
  param_set.map.clear();
  param_set.map_slow.clear();
  param_set.str_map.clear();

  turn_map.clear();
  for (const auto &p : turn_name_list) {
    if (p.first == TurnType::None) {
      continue;
    }
    const unsigned char sla_idx = tpp.profile_list[idx][p.first];
    turn_map[sla_idx].emplace_back(p);
    // load_sla(sla_idx, p.second, param_set.map[p.first]);
  }
  for (auto itr = turn_map.begin(); itr != turn_map.end(); ++itr) {
    load_slas(itr->first, itr->second, param_set.map);
  }
  turn_map.clear();
  for (const auto &p : turn_name_list) {
    if (p.first == TurnType::None) {
      continue;
    }
    const unsigned char sla_idx = tpp.profile_list[idx2][p.first];
    turn_map[sla_idx].emplace_back(p);
    // load_sla(sla_idx, p.second, param_set.map_slow[p.first]);
  }
  for (auto itr = turn_map.begin(); itr != turn_map.end(); ++itr) {
    load_slas(itr->first, itr->second, param_set.map_slow);
  }
  load_straight(idx, param_set.str_map);
  umount();
}
void MainTask::load_slalom_param() {}

void MainTask::load_param() {
  if (!ui->button_state_hold()) {
    load_hw_param();
    load_sensor_param();
    load_sys_param();
    load_turn_param_profiles(false);
    // load_slalom_param();
  }
}
void MainTask::rx_uart_json() {

  mount();
  load_param();

  uint8_t *data = (uint8_t *)malloc(BUF_SIZE);
  ui->coin(40);
  bool update = false;
  while (1) {
    int len = uart_read_bytes(UART_NUM_0, data, (BUF_SIZE - 1),
                              250.0 / portTICK_RATE_MS);
    uart_write_bytes(UART_NUM_0, (const char *)data, len);
    if (len) {
      data[len] = '\0';
      std::string str = std::string((const char *)data);
      save_json_data(str);
      update = true;
    }
    if (ui->button_state_hold()) {
      break;
    }
  }
  free(data);
  if (update) {
    load_param();
    ui->coin(40);
    ui->coin(40);
  }
  umount();
  ui->coin(100);
  vTaskDelay(100.0 / portTICK_PERIOD_MS);
}
void MainTask::task() {
  mp->set_userinterface(ui);
  search_ctrl->set_userinterface(ui);
  pt->motor_disable();
  pt->suction_disable();
  check_battery();
  // ui->init();

  ui->coin(80);

  rx_uart_json(); // uartでファイルを受け取る
  reset_tgt_data();
  reset_ego_data();

  if (sys.user_mode != 0) {
    mount();
    if (sys.user_mode == 1) {
      printf("test_sla\n");
      test_sla();
    } else if (sys.user_mode == 2) {
      printf("test_run\n");
      test_run();
    } else if (sys.user_mode == 3) {
      printf("test_turn\n");
      test_turn();
    } else if (sys.user_mode == 4) {
      printf("test_run_sla\n");
      test_run_sla();
    } else if (sys.user_mode == 5) {
      printf("test_search_sla\n");
      test_search_sla();
    } else if (sys.user_mode == 6) {
      printf("test_front_wall_offset\n");
      test_front_wall_offset();
    } else if (sys.user_mode == 7) {
      printf("test_front_ctrl hold\n");
      test_front_ctrl(true);
    } else if (sys.user_mode == 8) {
      printf("test_front_ctrl \n");
      test_front_ctrl(false);
    } else if (sys.user_mode == 9) {
      printf("test_sla_walloff\n");
      test_sla_walloff();
    } else if (sys.user_mode == 10) {
      printf("back\n");
      test_back();
    } else if (sys.user_mode == 11) {
      printf("suction\n");
      mp->reset_gyro_ref_with_check();
      pt->suction_enable(sys.test.suction_duty, sys.test.suction_duty_low);
      vTaskDelay(1000.0 * 10 / portTICK_PERIOD_MS);
      pt->suction_disable();
    } else if (sys.user_mode == 12) {
      printf("hold\n");
      mp->reset_gyro_ref_with_check();
      // pt->suction_enable(sys.test.suction_duty, sys.test.suction_duty_low);
      lt->start_slalom_log();
      vTaskDelay(1000.0 * 10 / portTICK_PERIOD_MS);
      lt->stop_slalom_log();
      lt->save(slalom_log_file);
      ui->coin(120);
      while (1) {
        if (ui->button_state_hold())
          break;
        vTaskDelay(10.0 / portTICK_RATE_MS);
      }
      lt->dump_log(slalom_log_file);
      while (1) {
        if (ui->button_state_hold())
          break;
        vTaskDelay(10.0 / portTICK_RATE_MS);
      }
    } else if (sys.user_mode == 13) {
      printf("keep_pivot\n");
      keep_pivot();
    } else if (sys.user_mode == 14) {
      printf("echo_sensor_csv\n");
      dump1();
    } else if (sys.user_mode == 15) {
      printf("echo_printf\n");
      dump2();
    } else if (sys.user_mode == 16) {
      printf("sys id para\n");
      test_system_identification(false);
    } else if (sys.user_mode == 17) {
      printf("sys id roll\n");
      test_system_identification(true);
    } else if (sys.user_mode == 18) {
      lt->dump_log(slalom_log_file);
      while (1) {
        if (ui->button_state_hold())
          break;
        vTaskDelay(10.0 / portTICK_RATE_MS);
      }
    } else if (sys.user_mode == 19) {
      printf("test_dia_walloff\n");
      test_dia_walloff();
    } else if (sys.user_mode == 20) {
      printf("test_pivot_n\n");
      test_pivot_n();
    } else if (sys.user_mode == 21) {
      printf("test_pivot_n2\n");
      test_pivot_n2();
    }
    umount();
  } else {
    // ui->hello_exia();
    ui->coin(200);
    lgc->init(sys.maze_size, sys.maze_size * sys.maze_size - 1);
    lgc->set_goal_pos(sys.goals);
    search_ctrl->set_lgc(lgc);
    search_ctrl->set_motion_plannning(mp);
    pc->set_logic(lgc);
    pc->set_userinterface(ui);
    read_maze_data();
    search_ctrl->print_maze();
    while (1) {
      mode_num = select_mode();
      printf("%d\n", mode_num);
      if (mode_num == 0) {
        lgc->set_goal_pos(sys.goals);
        rorl2 = ui->select_direction2();
        int idx = 0;
        if (rorl2 == TurnDirection::Right) {
          idx = 0;
        } else {
          idx = 1;
        }
        load_slalom_param(idx, idx);
        sr = search_ctrl->exec(param_set, SearchMode::ALL);
        if (sr == SearchResult::SUCCESS)
          save_maze_data(true);
        while (1) {
          if (ui->button_state_hold())
            break;
          vTaskDelay(10.0 / portTICK_RATE_MS);
        }
        search_ctrl->print_maze();
      } else if (mode_num == 1) {
        lgc->set_goal_pos(sys.goals);
        rorl = ui->select_direction();
        rorl2 = ui->select_direction2();
        int idx = 0;
        if (rorl2 == TurnDirection::Right) {
          idx = 0;
        } else {
          idx = 1;
        }
        sr = SearchResult::SUCCESS;
        load_slalom_param(idx, idx);
        if (rorl == TurnDirection::Right)
          sr = search_ctrl->exec(param_set, SearchMode::Kata);
        else
          sr = search_ctrl->exec(param_set, SearchMode::Return);
        if (sr == SearchResult::SUCCESS)
          save_maze_data(true);
        while (1) {
          if (ui->button_state_hold())
            break;
          vTaskDelay(10.0 / portTICK_RATE_MS);
        }
        search_ctrl->print_maze();
      } else if (mode_num == 2) {
        path_run(0, 0);
      } else if (mode_num == 3) {
        path_run(1, 1);
      } else if (mode_num == 4) {
        path_run(2, 2);
      } else if (mode_num == 5) {
        path_run(3, 3);
      } else if (mode_num == 6) {
        path_run(4, 4);
      } else if (mode_num == 7) {
        path_run(5, 5);
      } else if (mode_num == 8) {
        path_run(6, 6);
      } else if (mode_num == 9) {
        path_run(7, 7);
      } else if (mode_num == 10) {
        path_run(8, 8);
      } else if (mode_num == 11) {
        path_run(9, 8);
      } else if (mode_num == 12) {
        path_run(10, 8);
      } else if (mode_num == 13) {
        path_run(11, 8);
      } else if (mode_num == 14) {
        path_run(12, 8);
      } else if (mode_num == 15) {
        path_run(13, 8);
      } else if (mode_num == 16) {
        path_run(14, 8);
      } else if (mode_num == 17) {
        path_run(15, 8);
      } else if (mode_num == 18) {
        path_run(16, 8);
      } else if (mode_num == 19) {
        path_run(17, 8);
      } else if (mode_num == 20) {
        path_run(18, 8);
      } else if (mode_num == 21) {
        printf("keep_pivot\n");
        keep_pivot();
      } else if (mode_num == 22) {
        // dump1(); // taskの最終行に配置すること
        printf("suction\n");
        mp->reset_gyro_ref_with_check();
        pt->suction_enable(sys.test.suction_duty, sys.test.suction_duty_low);
        vTaskDelay(1000 * 10 / portTICK_PERIOD_MS);
        pt->suction_disable();
      } else if (mode_num == 23) {
        save_maze_data(false);
        save_maze_kata_data(false);
        save_maze_return_data(false);
        lgc->init(sys.maze_size, sys.maze_size * sys.maze_size - 1);
        lgc->set_goal_pos(sys.goals);
      }
      // param->fast_log_enable = 0; //１回きり
      vTaskDelay(10.0 / portTICK_RATE_MS);
    }
  }

  // echo_sensing_result_with_json();
  while (1) {
    if (ui->button_state_hold())
      break;
    vTaskDelay(xDelay10 / portTICK_RATE_MS);
  }
  dump1(); // taskの最終行に配置すること
  while (1) {
    vTaskDelay(xDelay100);
  }
}

void MainTask::recieve_data() {}

void MainTask::req_error_reset() {
  tgt_val->pl_req.error_vel_reset = 1;
  tgt_val->pl_req.error_gyro_reset = 1;
  tgt_val->pl_req.error_ang_reset = 1;
  tgt_val->pl_req.error_dist_reset = 1;
  tgt_val->pl_req.time_stamp++;
  xQueueReset(*qh);
  xQueueSendToFront(*qh, &tgt_val, 1);
}

void MainTask::test_system_identification(bool para) {
  if (para) {
    rorl = ui->select_direction();
  }

  if (para) {
    lt->change_sysid_mode(sys.test.sysid_duty, sys.test.sysid_duty,
                          sys.test.sysid_time);
  } else {
    if (rorl == TurnDirection::Right) {
      lt->change_sysid_mode(sys.test.sysid_duty, -sys.test.sysid_duty,
                            sys.test.sysid_time);
    } else {
      lt->change_sysid_mode(-sys.test.sysid_duty, sys.test.sysid_duty,
                            sys.test.sysid_time);
    }
  }
  mp->reset_gyro_ref_with_check();

  reset_tgt_data();
  reset_ego_data();
  pt->motor_enable();

  req_error_reset();

  if (param->test_log_enable > 0) {
    lt->start_slalom_log();
  }
  if (para) {
    mp->system_identification(MotionType::SYS_ID_PARA, sys.test.sysid_duty,
                              sys.test.sysid_duty, sys.test.sysid_time);
  } else {
    if (rorl == TurnDirection::Right) {
      mp->system_identification(MotionType::SYS_ID_ROLL, sys.test.sysid_duty,
                                -sys.test.sysid_duty, sys.test.sysid_time);
    } else {
      mp->system_identification(MotionType::SYS_ID_ROLL, -sys.test.sysid_duty,
                                sys.test.sysid_duty, sys.test.sysid_time);
    }
  }
  lt->stop_slalom_log();
  pt->motor_disable();
  reset_tgt_data();
  reset_ego_data();
  req_error_reset();
  pt->motor_disable();
  pt->suction_disable();
  lt->stop_slalom_log();

  lt->save_sysid(sysid_log_file);
  ui->coin(120);

  param->sen_ref_p.normal.exist.right45 = backup_r;
  param->sen_ref_p.normal.exist.left45 = backup_l;
  while (1) {
    if (ui->button_state_hold())
      break;
    vTaskDelay(10.0 / portTICK_RATE_MS);
  }
  lt->dump_log_sysid(slalom_log_file);
  while (1) {
    if (ui->button_state_hold())
      break;
    vTaskDelay(10.0 / portTICK_RATE_MS);
  }
}

void MainTask::test_run() {
  mp->reset_gyro_ref_with_check();

  if (sys.test.suction_active) {
    pt->suction_enable(sys.test.suction_duty, sys.test.suction_duty_low);
    vTaskDelay(xDelay1000);
  }

  reset_tgt_data();
  reset_ego_data();
  pt->motor_enable();

  req_error_reset();
  if (param->test_log_enable > 0) {
    lt->start_slalom_log();
  }
  pt->search_mode = true;
  ps.v_max = sys.test.v_max;
  ps.v_end = 20;
  ps.dist = sys.test.dist - 5;
  // + param->offset_start_dist;
  ps.accl = sys.test.accl;
  ps.decel = sys.test.decel;
  ps.sct = SensorCtrlType::Straight;
  if (sys.test.dia == 1) {
    ps.sct = SensorCtrlType::Dia;
  }
  ps.motion_type = MotionType::STRAIGHT;
  ps.dia_mode = false;

  mp->go_straight(ps);
  ps.v_max = 20;
  ps.v_end = sys.test.end_v;
  ps.dist = 5;
  ps.accl = sys.test.accl;
  ps.decel = sys.test.decel;
  mp->go_straight(ps);
  lt->stop_slalom_log();
  // bool front_ctrl = (sensing_result->ego.front_dist < 60);
  vTaskDelay(50.0 / portTICK_RATE_MS);
  pt->motor_disable();
  reset_tgt_data();
  reset_ego_data();
  req_error_reset();
  pt->suction_disable();

  reset_tgt_data();
  reset_ego_data();
  req_error_reset();
  mp->coin();
  lt->save(slalom_log_file);
  ui->coin(120);
  pt->search_mode = false;
  while (1) {
    if (ui->button_state_hold())
      break;
    vTaskDelay(10.0 / portTICK_RATE_MS);
  }
  lt->dump_log(slalom_log_file);
  while (1) {
    if (ui->button_state_hold())
      break;
    vTaskDelay(10.0 / portTICK_RATE_MS);
  }
}
void MainTask::test_back() {
  mp->reset_gyro_ref_with_check();

  if (sys.test.suction_active) {
    pt->suction_enable(sys.test.suction_duty, sys.test.suction_duty_low);
    vTaskDelay(xDelay1000);
  }

  reset_tgt_data();
  reset_ego_data();
  pt->motor_enable();

  req_error_reset();
  if (param->test_log_enable > 0) {
    lt->start_slalom_log();
  }
  ps.v_max = sys.test.v_max;
  ps.v_end = -20;
  ps.dist = sys.test.dist - 5;
  // + param->offset_start_dist;
  ps.accl = sys.test.accl;
  ps.decel = sys.test.decel;
  ps.sct = SensorCtrlType::Straight;
  ps.motion_type = MotionType::STRAIGHT;
  ps.dia_mode = false;

  mp->go_straight(ps);
  ps.v_max = 20;
  ps.v_end = sys.test.end_v;
  ps.dist = 5;
  ps.accl = sys.test.accl;
  ps.decel = sys.test.decel;
  mp->go_straight(ps);
  vTaskDelay(100.0 / portTICK_RATE_MS);
  pt->motor_disable();
  reset_tgt_data();
  reset_ego_data();
  req_error_reset();
  pt->suction_disable();

  lt->stop_slalom_log();
  reset_tgt_data();
  reset_ego_data();
  req_error_reset();
  mp->coin();
  lt->save(slalom_log_file);
  ui->coin(120);

  while (1) {
    if (ui->button_state_hold())
      break;
    vTaskDelay(10.0 / portTICK_RATE_MS);
  }
  lt->dump_log(slalom_log_file);
  while (1) {
    if (ui->button_state_hold())
      break;
    vTaskDelay(10.0 / portTICK_RATE_MS);
  }
}

void MainTask::test_run_sla() {
  mp->reset_gyro_ref_with_check();

  if (sys.test.suction_active) {
    pt->suction_enable(sys.test.suction_duty, sys.test.suction_duty_low);
    vTaskDelay(500.0 / portTICK_PERIOD_MS);
  }

  reset_tgt_data();
  reset_ego_data();
  pt->motor_enable();

  req_error_reset();
  if (param->test_log_enable > 0) {
    lt->start_slalom_log();
  }
  // pt->active_logging(_f);

  ps.v_max = sys.test.v_max;
  ps.v_end = 20;
  ps.dist = 45 + param->offset_start_dist + 90;
  ps.accl = sys.test.accl;
  ps.decel = sys.test.decel;
  ps.sct = SensorCtrlType::Straight;
  ps.dia_mode = false;

  mp->go_straight(ps);
  ps.v_max = 20;
  ps.v_end = sys.test.end_v;
  ps.dist = 5;
  ps.accl = sys.test.accl;
  ps.decel = sys.test.decel;
  mp->go_straight(ps);
  vTaskDelay(100.0 / portTICK_RATE_MS);
  pt->motor_disable();
  reset_tgt_data();
  reset_ego_data();
  req_error_reset();
  pt->suction_disable();

  lt->stop_slalom_log();
  reset_tgt_data();
  reset_ego_data();
  req_error_reset();
  lt->save(slalom_log_file);
  ui->coin(120);

  while (1) {
    if (ui->button_state_hold())
      break;
    vTaskDelay(10.0 / portTICK_RATE_MS);
  }
  lt->dump_log(slalom_log_file);
  while (1) {
    if (ui->button_state_hold())
      break;
    vTaskDelay(10.0 / portTICK_RATE_MS);
  }
}

void MainTask::test_turn() {
  rorl = ui->select_direction();

  mp->reset_gyro_ref_with_check();
  reset_tgt_data();
  reset_ego_data();
  pt->motor_enable();

  req_error_reset();

  if (param->test_log_enable > 0) {
    lt->start_slalom_log();
  }
  // pt->active_logging();
  pr.w_max = sys.test.w_max;
  pr.alpha = sys.test.alpha;
  pr.w_end = 0;
  pr.ang = sys.test.ang * m_PI / 180;
  pr.RorL = rorl;

  mp->pivot_turn(pr);
  vTaskDelay(100.0 / portTICK_RATE_MS);
  pt->motor_disable();
  pt->suction_disable();

  lt->stop_slalom_log();
  reset_tgt_data();
  reset_ego_data();
  req_error_reset();
  lt->save(slalom_log_file);
  ui->coin(120);

  while (1) {
    if (ui->button_state_hold())
      break;
    vTaskDelay(10.0 / portTICK_RATE_MS);
  }
  lt->dump_log(slalom_log_file);
  while (1) {
    if (ui->button_state_hold())
      break;
    vTaskDelay(10.0 / portTICK_RATE_MS);
  }
}

void MainTask::test_pivot_n() {
  rorl = ui->select_direction();

  mp->reset_gyro_ref_with_check();
  reset_tgt_data();
  reset_ego_data();
  pt->motor_enable();

  req_error_reset();

  if (param->test_log_enable > 0) {
    lt->start_slalom_log();
  }
  // pt->active_logging();
  pr.w_max = sys.test.w_max;
  pr.alpha = sys.test.alpha;
  pr.w_end = 0;
  pr.ang = sys.test.ang * m_PI / 180;
  pr.RorL = rorl;

  for (int i = 0; i < sys.test.turn_times; i++) {
    mp->pivot_turn(pr);
    vTaskDelay(50.0 / portTICK_RATE_MS);
  }
  pt->motor_disable();
  pt->suction_disable();

  lt->stop_slalom_log();
  reset_tgt_data();
  reset_ego_data();
  req_error_reset();
  lt->save(slalom_log_file);
  ui->coin(120);

  while (1) {
    if (ui->button_state_hold())
      break;
    vTaskDelay(10.0 / portTICK_RATE_MS);
  }
  lt->dump_log(slalom_log_file);
  while (1) {
    if (ui->button_state_hold())
      break;
    vTaskDelay(10.0 / portTICK_RATE_MS);
  }
}

void MainTask::test_pivot_n2() {
  rorl = ui->select_direction();
  search_ctrl->set_lgc(lgc);
  search_ctrl->set_motion_plannning(mp);
  pc->set_logic(lgc);
  pc->set_userinterface(ui);
  load_slalom_param(0, 0);
  // sr = search_ctrl->exec(param_set, SearchMode::ALL);

  mp->reset_gyro_ref_with_check();
  reset_tgt_data();
  reset_ego_data();
  pt->motor_enable();

  req_error_reset();

  if (param->test_log_enable > 0) {
    lt->start_slalom_log();
  }
  // pt->active_logging();
  pr.w_max = sys.test.w_max;
  pr.alpha = sys.test.alpha;
  pr.w_end = 0;
  pr.ang = sys.test.ang * m_PI / 180;
  pr.RorL = rorl;

  for (int i = 0; i < sys.test.turn_times; i++) {
    search_ctrl->pivot(param_set, 0);
    vTaskDelay(50.0 / portTICK_RATE_MS);
  }
  pt->motor_disable();
  pt->suction_disable();

  lt->stop_slalom_log();
  reset_tgt_data();
  reset_ego_data();
  req_error_reset();
  lt->save(slalom_log_file);
  ui->coin(120);

  while (1) {
    if (ui->button_state_hold())
      break;
    vTaskDelay(10.0 / portTICK_RATE_MS);
  }
  lt->dump_log(slalom_log_file);
  while (1) {
    if (ui->button_state_hold())
      break;
    vTaskDelay(10.0 / portTICK_RATE_MS);
  }
}

void MainTask::test_sla() {

  if (file_idx >= tpp.file_list_size) {
    printf("%d %d\n", file_idx, tpp.file_list_size);
    ui->error();
    return;
  }
  load_turn_param_profiles(true);
  load_slalom_param(file_idx, file_idx);
  sla_p = param_set.map[static_cast<TurnType>(sys.test.sla_type)];
  auto sla_p2 = param_set.map[static_cast<TurnType>(sys.test.sla_type2)];
  printf("slalom params\n");
  printf("v = %f\n", sla_p.v);
  printf("ang = %f\n", sla_p.ang * 180 / m_PI);
  printf("radius =  %f\n", sla_p.rad);
  printf("time =  %f\n", sla_p.time);
  printf("n = %d\n", sla_p.pow_n);

  rorl = ui->select_direction();
  rorl2 = (rorl == TurnDirection::Right) ? (TurnDirection::Left)
                                         : (TurnDirection::Right);

  backup_r = param->sen_ref_p.normal.exist.right45;
  backup_l = param->sen_ref_p.normal.exist.left45;
  // if (sys.test.ignore_opp_sen) {
  //   if (rorl == TurnDirection::Right) {
  //     param->sen_ref_p.normal.exist.right45 = 1;
  //   } else {
  //     param->sen_ref_p.normal.exist.left45 = 1;
  //   }
  // }
  mp->reset_gyro_ref_with_check();

  if (sys.test.suction_active) {
    pt->suction_enable(sys.test.suction_duty, sys.test.suction_duty_low);
    vTaskDelay(xDelay1000);
  }

  reset_tgt_data();
  reset_ego_data();
  pt->motor_enable();

  req_error_reset();

  if (param->test_log_enable > 0) {
    lt->start_slalom_log();
  }

  ps.v_max = sla_p.v;
  ps.v_end = sla_p.v;
  ps.dist = 90 + param->offset_start_dist;
  if (sys.test.start_turn > 0) {
    ps.dist = param->offset_start_dist;
  }
  ps.accl = sys.test.accl;
  ps.decel = sys.test.decel;
  ps.sct = SensorCtrlType::Straight;
  ps.motion_type = MotionType::STRAIGHT;
  ps.dia_mode = false;
  mp->go_straight(ps);

  nm.v_max = sla_p.v;
  nm.v_end = sla_p.v;
  nm.accl = sys.test.accl;
  nm.decel = sys.test.decel;
  nm.is_turn = false;

  mp->slalom(sla_p, rorl, nm, false);

  if (sys.test.sla_return > 0) {
    const auto type2 = static_cast<TurnType>(sys.test.sla_type2);
    bool dia = type2 == TurnType::Dia45_2 || type2 == TurnType::Dia135_2 ||
               type2 == TurnType::Dia90;

    mp->slalom(sla_p2, rorl2, nm, dia);
  }

  ps.v_max = sla_p.v;
  ps.v_end = sys.test.end_v;
  ps.dist = 90;

  if (sys.test.sla_return == 0) {
    if (static_cast<TurnType>(sys.test.sla_type) == TurnType::Dia45 ||
        static_cast<TurnType>(sys.test.sla_type) == TurnType::Dia135) {
      ps.dist = 45 * ROOT2;
    }
  } else if (sys.test.sla_return == 1) {
    if (static_cast<TurnType>(sys.test.sla_type) == TurnType::Dia90) {
      ps.dist = 45 * ROOT2;
    }
  }
  if (sys.test.ignore_opp_sen > 0) {
    ps.v_max = sys.test.v_max;
    ps.dist = sys.test.dist;
  }
  ps.sct = SensorCtrlType::NONE;
  if (static_cast<TurnType>(sys.test.sla_type) == TurnType::Dia45 ||
      static_cast<TurnType>(sys.test.sla_type) == TurnType::Dia135) {
    if (sys.test.ignore_opp_sen == 2) {
      ps.sct = SensorCtrlType::Dia;
    }
  }
  ps.accl = sys.test.accl;
  ps.decel = sys.test.decel;

  mp->go_straight(ps);

  vTaskDelay(100.0 / portTICK_RATE_MS);
  pt->motor_disable();
  reset_tgt_data();
  reset_ego_data();
  req_error_reset();
  pt->motor_disable();
  pt->suction_disable();
  lt->stop_slalom_log();

  lt->save(slalom_log_file);
  ui->coin(120);

  param->sen_ref_p.normal.exist.right45 = backup_r;
  param->sen_ref_p.normal.exist.left45 = backup_l;
  while (1) {
    if (ui->button_state_hold())
      break;
    vTaskDelay(10.0 / portTICK_RATE_MS);
  }
  lt->dump_log(slalom_log_file);
  while (1) {
    if (ui->button_state_hold())
      break;
    vTaskDelay(10.0 / portTICK_RATE_MS);
  }
}

void MainTask::test_search_sla() {

  file_idx = 0;

  if (file_idx >= tpp.file_list_size) {
    ui->error();
    return;
  }

  load_slalom_param(0, 0);
  sla_p = param_set.map[TurnType::Normal];
  str_p = param_set.str_map[StraightType::Search];

  printf("str: \n");
  printf("- v_max: %f\n", str_p.v_max);
  printf("- accl: %f\n", str_p.accl);
  printf("- decel: %f\n", str_p.decel);

  rorl = ui->select_direction();
  backup_r = param->sen_ref_p.normal.exist.right45;
  backup_l = param->sen_ref_p.normal.exist.left45;
  // if (sys.test.ignore_opp_sen) {
  //   if (rorl == TurnDirection::Right) {
  //     param->sen_ref_p.normal.exist.right45 = 1;
  //   } else {
  //     param->sen_ref_p.normal.exist.left45 = 1;
  //   }
  // }
  rorl2 = (rorl == TurnDirection::Right) ? (TurnDirection::Left)
                                         : (TurnDirection::Right);
  mp->reset_gyro_ref_with_check();

  if (sys.test.suction_active) {
    pt->suction_enable(sys.test.suction_duty, sys.test.suction_duty_low);
    vTaskDelay(xDelay1000);
  }

  reset_tgt_data();
  reset_ego_data();
  pt->motor_enable();

  req_error_reset();

  if (param->test_log_enable > 0) {
    lt->start_slalom_log();
  }

  ps.v_max = str_p.v_max;
  ps.v_end = str_p.v_max;
  ps.dist = 45 + param->offset_start_dist_search;
  ps.accl = str_p.accl;
  ps.decel = str_p.decel;
  ps.sct = SensorCtrlType::Straight;

  mp->go_straight(ps);

  nm.v_max = str_p.v_max;
  nm.v_end = sla_p.v;
  nm.accl = str_p.accl;
  nm.decel = str_p.decel;
  nm.is_turn = false;

  mp->slalom(sla_p, rorl, nm);
  for (int i = 0; i < sys.test.turn_times; i++) {
    mp->slalom(sla_p, rorl2, nm);
  }

  ps.v_max = sla_p.v;
  ps.v_end = 20;
  ps.dist = 45 - 5;
  ps.accl = sys.test.accl;
  ps.decel = sys.test.decel;
  ps.sct = SensorCtrlType::NONE;
  mp->go_straight(ps);
  ps.v_max = 20;
  ps.v_end = sys.test.end_v;
  ps.dist = 5;
  ps.accl = sys.test.accl;
  ps.decel = sys.test.decel;
  ps.sct = SensorCtrlType::NONE;
  mp->go_straight(ps);

  vTaskDelay(100.0 / portTICK_RATE_MS);
  pt->motor_disable();
  reset_tgt_data();
  reset_ego_data();
  req_error_reset();
  pt->suction_disable();
  lt->stop_slalom_log();

  lt->save(slalom_log_file);
  ui->coin(120);

  param->sen_ref_p.normal.exist.right45 = backup_r;
  param->sen_ref_p.normal.exist.left45 = backup_l;
  while (1) {
    if (ui->button_state_hold())
      break;
    vTaskDelay(10.0 / portTICK_RATE_MS);
  }
  lt->dump_log(slalom_log_file);
  while (1) {
    if (ui->button_state_hold())
      break;
    vTaskDelay(10.0 / portTICK_RATE_MS);
  }
}

// 探索中の前壁制御
void MainTask::test_front_ctrl(bool mode) {
  file_idx = sys.test.file_idx;

  if (file_idx >= tpp.file_list_size) {
    ui->error();
    return;
  }

  mp->reset_gyro_ref_with_check();

  reset_tgt_data();
  reset_ego_data();
  pt->motor_enable();
  req_error_reset();

  mp->front_ctrl(mode);
  pt->motor_disable();
  ui->coin(120);

  while (1) {
    if (ui->button_state_hold())
      break;
    vTaskDelay(10.0 / portTICK_RATE_MS);
  }
  lt->dump_log(slalom_log_file);
  while (1) {
    if (ui->button_state_hold())
      break;
    vTaskDelay(10.0 / portTICK_RATE_MS);
  }
}

void MainTask::test_front_wall_offset() {
  printf("search_walloff_offset= %f, %f\n",
         param->sen_ref_p.search_exist.offset_l,
         param->sen_ref_p.search_exist.offset_r);

  mp->reset_gyro_ref_with_check();

  if (sys.test.suction_active) {
    pt->suction_enable(sys.test.suction_duty, sys.test.suction_duty_low);
    vTaskDelay(xDelay1000);
  }

  reset_tgt_data();
  reset_ego_data();
  pt->motor_enable();

  req_error_reset();
  if (param->test_log_enable > 0) {
    lt->start_slalom_log();
  }
  // pt->active_logging(_f);

  ps.v_max = sys.test.v_max;
  ps.v_end = sys.test.v_max;
  ps.dist = 90 + 45;
  ps.accl = sys.test.accl;
  ps.decel = sys.test.decel;
  ps.sct = SensorCtrlType::Straight;
  ps.wall_off_req = WallOffReq::NONE;
  ps.wall_off_dist_r = 0;
  ps.wall_off_dist_l = 0;
  ps.dia_mode = false;
  mp->go_straight(ps);

  ps.dist = 85;
  if (sensing_result->ego.left90_mid_dist < 150 &&
      sensing_result->ego.right90_mid_dist < 150) {
    ps.dist -= (param->front_dist_offset2 - sensing_result->ego.front_mid_dist);
  }

  ps.v_max = sys.test.v_max;
  ps.v_end = 20;
  mp->go_straight(ps);

  ps.v_max = 20;
  ps.v_end = sys.test.end_v;
  ps.dist = 5;
  mp->go_straight(ps);

  vTaskDelay(100.0 / portTICK_RATE_MS);
  pt->motor_disable();
  reset_tgt_data();
  reset_ego_data();
  req_error_reset();
  pt->suction_disable();

  lt->stop_slalom_log();
  reset_tgt_data();
  reset_ego_data();
  req_error_reset();
  lt->save(slalom_log_file);
  ui->coin(120);

  while (1) {
    if (ui->button_state_hold())
      break;
    vTaskDelay(10.0 / portTICK_RATE_MS);
  }
  lt->dump_log(slalom_log_file);
  while (1) {
    if (ui->button_state_hold())
      break;
    vTaskDelay(10.0 / portTICK_RATE_MS);
  }
}

void MainTask::test_dia_walloff() {
  rorl = ui->select_direction();
  rorl2 = (rorl == TurnDirection::Right) ? (TurnDirection::Left)
                                         : (TurnDirection::Right);
  if (rorl == TurnDirection::Right) {
    param->sen_ref_p.normal.exist.right45 = 1;
  } else {
    param->sen_ref_p.normal.exist.left45 = 1;
  }
  mp->reset_gyro_ref_with_check();

  // if (sys.test.suction_active) {
  //   pt->suction_enable(sys.test.suction_duty, sys.test.suction_duty_low);
  //   vTaskDelay(500.0 / portTICK_PERIOD_MS);
  // }

  reset_tgt_data();
  reset_ego_data();
  pt->motor_enable();

  req_error_reset();
  if (param->test_log_enable > 0) {
    lt->start_slalom_log();
  }
  // pt->active_logging(_f);

  ps.v_max = sys.test.v_max;
  ps.v_end = sys.test.v_max;
  ps.dist = 90 + param->offset_start_dist;
  ps.accl = sys.test.accl;
  ps.decel = sys.test.decel;
  ps.sct = SensorCtrlType::Straight;
  ps.wall_off_req = WallOffReq::NONE;
  ps.wall_off_dist_r = 0;
  ps.wall_off_dist_l = 0;
  ps.dia_mode = false;
  mp->go_straight(ps);

  sla_p = paramset_list[file_idx].map[TurnType::Dia45];
  nm.v_max = sla_p.v;
  nm.v_end = sla_p.v;
  nm.accl = sys.test.accl;
  nm.decel = sys.test.decel;
  nm.is_turn = false;
  mp->slalom(sla_p, rorl, nm, false);

  ps.dist = 45 * std::sqrt(2);
  ps.dia_mode = true;

  mp->wall_off_dia(rorl2, ps);

  ps.dist = ps.dist - 5;
  ps.v_max = sys.test.v_max;
  ps.v_end = 20;
  ps.accl = sys.test.accl;
  ps.decel = sys.test.decel;
  ps.wall_off_req = WallOffReq::NONE;
  ps.sct = SensorCtrlType::NONE;
  mp->go_straight(ps);

  ps.v_max = 20;
  ps.v_end = sys.test.end_v;
  ps.dist = 5;
  ps.accl = sys.test.accl;
  ps.decel = sys.test.decel;
  ps.sct = SensorCtrlType::NONE;
  mp->go_straight(ps);

  vTaskDelay(100.0 / portTICK_RATE_MS);
  pt->motor_disable();
  reset_tgt_data();
  reset_ego_data();
  req_error_reset();
  pt->suction_disable();

  lt->stop_slalom_log();
  reset_tgt_data();
  reset_ego_data();
  req_error_reset();
  lt->save(slalom_log_file);
  ui->coin(120);

  while (1) {
    if (ui->button_state_hold())
      break;
    vTaskDelay(10.0 / portTICK_RATE_MS);
  }
  lt->dump_log(slalom_log_file);
  while (1) {
    if (ui->button_state_hold())
      break;
    vTaskDelay(10.0 / portTICK_RATE_MS);
  }
}

void MainTask::test_sla_walloff() {
  rorl = ui->select_direction();

  if (rorl == TurnDirection::Right) {
    param->sen_ref_p.normal.exist.right45 = 1;
  } else {
    param->sen_ref_p.normal.exist.left45 = 1;
  }
  mp->reset_gyro_ref_with_check();

  if (sys.test.suction_active) {
    pt->suction_enable(sys.test.suction_duty, sys.test.suction_duty_low);
    vTaskDelay(500.0 / portTICK_PERIOD_MS);
  }

  reset_tgt_data();
  reset_ego_data();
  pt->motor_enable();

  req_error_reset();
  if (param->test_log_enable > 0) {
    lt->start_slalom_log();
  }
  // pt->active_logging(_f);

  ps.v_max = sys.test.v_max;
  ps.v_end = sys.test.v_max;
  ps.dist = 45 + 45 + param->offset_start_dist;
  ps.accl = sys.test.accl;
  ps.decel = sys.test.decel;
  ps.sct = SensorCtrlType::Straight;
  ps.wall_off_req = WallOffReq::NONE;
  ps.wall_off_dist_r = 0;
  ps.wall_off_dist_l = 0;
  ps.dia_mode = false;
  mp->go_straight(ps);

  ps.dist = 90 - 5;
  mp->wall_off(rorl, ps);
  ps.v_max = sys.test.v_max;
  ps.v_end = 20;
  ps.accl = sys.test.accl;
  ps.decel = sys.test.decel;
  ps.wall_off_req = WallOffReq::NONE;
  mp->go_straight(ps);

  ps.v_max = 20;
  ps.v_end = sys.test.end_v;
  ps.dist = 5;
  ps.accl = sys.test.accl;
  ps.decel = sys.test.decel;
  mp->go_straight(ps);

  vTaskDelay(100.0 / portTICK_RATE_MS);
  pt->motor_disable();
  reset_tgt_data();
  reset_ego_data();
  req_error_reset();
  pt->suction_disable();

  lt->stop_slalom_log();
  reset_tgt_data();
  reset_ego_data();
  req_error_reset();
  lt->save(slalom_log_file);
  ui->coin(120);

  while (1) {
    if (ui->button_state_hold())
      break;
    vTaskDelay(10.0 / portTICK_RATE_MS);
  }
  lt->dump_log(slalom_log_file);
  while (1) {
    if (ui->button_state_hold())
      break;
    vTaskDelay(10.0 / portTICK_RATE_MS);
  }
}

void MainTask::save_maze_data(bool write) {
  mount();
  auto *f = fopen(maze_log_file.c_str(), "wb");
  if (f == NULL)
    return;
  if (write) {
    for (const auto d : lgc->map) {
      fprintf(f, "%d,", d);
    }
  } else {
    printf("delete maze data\n");
    fprintf(f, "null");
  }
  fclose(f);

  printf("wall: [");
  // for (const auto d : lgc->map) {
  for (int x = 0; x < sys.maze_size; x++) {
    for (int y = 0; y < sys.maze_size; y++) {
      auto d = lgc->map[x + y * sys.maze_size];
      printf("%d,", (d & 0x0f));
    }
  }
  printf("]\n");

  printf("wall2: [");
  // for (const auto d : lgc->map) {
  for (int x = 0; x < sys.maze_size; x++) {
    for (int y = 0; y < sys.maze_size; y++) {
      auto d = lgc->map[x + y * sys.maze_size];
      printf("%d,", (d & 0xff));
    }
    printf("\n");
  }
  printf("]\n");
  umount();
}
void MainTask::save_maze_kata_data(bool write) {
  mount();
  auto *f = fopen(maze_log_kata_file.c_str(), "wb");
  if (f == NULL)
    return;
  if (write) {
    for (const auto d : lgc->map) {
      fprintf(f, "%d,", d);
    }
  } else {
    printf("delete maze data\n");
    fprintf(f, "null");
  }
  fclose(f);
  umount();
}
void MainTask::save_maze_return_data(bool write) {
  mount();
  auto *f = fopen(maze_log_return_file.c_str(), "wb");
  if (f == NULL)
    return;
  if (write) {
    for (const auto d : lgc->map) {
      fprintf(f, "%d,", d);
    }
  } else {
    printf("delete maze data\n");
    fprintf(f, "null");
  }
  fclose(f);
  umount();
}

void MainTask::read_maze_data() {
  mount();
  auto *f = fopen(maze_log_file.c_str(), "rb");
  if (f == NULL)
    return;
  // char line_buf[LINE_BUF_SIZE];
  std::string str = "";
  while (fgets(line_buf, sizeof(line_buf), f) != NULL) {
    printf("%s", line_buf);
    // printf("_______\n");
    str += std::string(line_buf);
  }
  printf("\n");
  fclose(f);
  // std::string str = std::string(line_buf);
  if (str == "null")
    return;
  auto map_list = split(str, ',');
  printf("map_list.size = %d\n", map_list.size());
  for (int i = 0; i < map_list.size(); i++) {
    lgc->set_native_wall_data(i, stoi(map_list[i]));
  }
  printf("read maze data!!!\n");

  printf("wall: [");
  // for (const auto d : lgc->map) {
  for (int x = 0; x < sys.maze_size; x++) {
    for (int y = 0; y < sys.maze_size; y++) {
      auto d = lgc->map[x + y * sys.maze_size];
      printf("%d,", (d & 0x0f));
    }
  }
  printf("]\n");

  printf("wall2: [");
  // for (const auto d : lgc->map) {
  for (int x = 0; x < sys.maze_size; x++) {
    for (int y = 0; y < sys.maze_size; y++) {
      auto d = lgc->map[x + y * sys.maze_size];
      printf("%d,", (d & 0xff));
    }
    printf("\n");
  }
  printf("]\n");
  umount();
}

void MainTask::path_run(int idx, int idx2) {
  // printf("read_param\n");
  // printf("before: %d bytes\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
  load_slalom_param(idx, idx2);

  // printf("after: %d bytes\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

  // printf("path_create\n");
  // printf("before: %d bytes\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
  pc->other_route_map.clear();
  const bool res = pc->path_create(false);
  if (!res) {
    ui->error();
    return;
  }
  pc->convert_large_path(true);
  pc->diagonalPath(true, true);

  pc->path_s2.clear();
  pc->path_t2.clear();
  for (int i = 0; i < pc->path_t.size(); i++) {
    pc->path_s2.push_back(pc->path_s[i]);
    pc->path_t2.push_back(pc->path_t[i]);
  }
  //速度ベース経路導出
  auto result = pc->timebase_path_create(false, param_set);
  if (result == 1) { //成功
    pc->path_s.clear();
    pc->path_t.clear();
    for (int i = 0; i < pc->path_t2.size(); i++) {
      pc->path_s.push_back(pc->path_s2[i]);
      pc->path_t.push_back(pc->path_t2[i]);
    }
  } else { //失敗
    pc->other_route_map.clear();
    const bool res = pc->path_create(false);
    if (!res) {
      ui->error();
      return;
    }
    pc->convert_large_path(true);
    pc->diagonalPath(true, true);
  }
  pc->print_path();
  printf("%f\n", pc->route.time);
  // printf("after: %d bytes\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

  const auto rorl = ui->select_direction2();
  const auto backup_l45 = param->sen_ref_p.normal.exist.left45;
  const auto backup_r45 = param->sen_ref_p.normal.exist.right45;
  if (rorl == TurnDirection::Right) {

  } else {
    pc->other_route_map.clear();
    const bool res = pc->path_create(false);
    if (!res) {
      ui->error();
      return;
    }
    pc->convert_large_path(true);
    pc->diagonalPath(true, true);
    pc->print_path();
  }
  mp->exec_path_running(param_set);

  param->sen_ref_p.normal.exist.left45 = backup_l45;
  param->sen_ref_p.normal.exist.right45 = backup_r45;

  // param->fast_log_enable = 0; //１回きり
}