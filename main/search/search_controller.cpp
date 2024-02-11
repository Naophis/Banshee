#include "include/search_controller.hpp"

SearchController::SearchController() {
  adachi = std::make_shared<Adachi>();
  ego = std::make_shared<ego_t>();
  adachi->set_ego(ego);
}
SearchController::~SearchController() {}

void SearchController::set_lgc(std::shared_ptr<MazeSolverBaseLgc> &_lgc) {
  lgc = _lgc;
  adachi->set_logic(_lgc);
  lgc->set_ego(ego);
}

void SearchController::set_motion_plannning(
    std::shared_ptr<MotionPlanning> &_mp) {
  mp = _mp;
}

void SearchController::set_planning_task(std::shared_ptr<PlanningTask> &_pt) {
  pt = _pt;
}

void SearchController::set_sensing_entity(
    std::shared_ptr<sensing_result_entity_t> &_entity) {
  sensing_result = _entity;
}

void SearchController::set_logging_task(std::shared_ptr<LoggingTask> &_lt) {
  lt = _lt;
}

void SearchController::set_userinterface(std::shared_ptr<UserInterface> &_ui) {
  ui = _ui;
}
void SearchController::reset() {
  ego->dir = Direction::North;
  ego->x = 0;
  ego->y = 1;
  ego->prev_motion = 0; // not use
  lgc->set_default_wall_data();
  adachi->reset_goal();
}
MotionResult SearchController::go_straight_wrapper(param_set_t &p_set,
                                                   float diff,
                                                   StraightType st) {
  // param_straight_t p;
  p.v_max = p_set.str_map[st].v_max;
  if (p.v_max + 10 < pt->mpc_next_ego.v) {
    p.v_max = p_set.str_map[StraightType::FastRun].v_max;
  }
  p.v_end = p_set.str_map[st].v_max;
  if (st == StraightType::FastRunDia) {
    p.v_end = p_set.str_map[StraightType::Search].v_max;
  }

  p.accl = p_set.str_map[st].accl;
  p.decel = p_set.str_map[st].decel;

  p.dist = param->cell - diff;
  p.motion_type = MotionType::STRAIGHT;
  p.wall_off_req = WallOffReq::NONE;
  p.sct = SensorCtrlType::Straight;
  p.wall_off_dist_r = param->sen_ref_p.search_exist.offset_r;
  p.wall_off_dist_l = param->sen_ref_p.search_exist.offset_l;
  p.dia_mode = false;

  p.search_str_wide_ctrl_l = false;
  p.search_str_wide_ctrl_r = false;

  bool left_exist =
      sensing_result->ego.left45_dist < param->sen_ref_p.search_exist.left45;
  bool right_exist =
      sensing_result->ego.right45_dist < param->sen_ref_p.search_exist.right45;
  p.search_str_wide_ctrl_l = left_exist;
  p.search_str_wide_ctrl_r = right_exist;
  bool near_left =
      left_exist && //
      sensing_result->ego.left45_dist < param->sen_ref_p.search_ref.left45;
  bool near_right =
      right_exist && //
      sensing_result->ego.right45_dist < param->sen_ref_p.search_ref.right45;

  bool far_left =
      left_exist && //
      sensing_result->ego.left45_dist > param->sen_ref_p.search_ref.left90;
  bool far_right =
      right_exist && //
      sensing_result->ego.right45_dist > param->sen_ref_p.search_ref.right90;

  if (!left_exist && !right_exist) {
    auto exist_right45 = param->sen_ref_p.normal.exist.right45;
    auto exist_left45 = param->sen_ref_p.normal.exist.left45;
    if (!p.search_str_wide_ctrl_l) {
      param->sen_ref_p.normal.exist.left45 = 0;
    }
    if (!p.search_str_wide_ctrl_r) {
      param->sen_ref_p.normal.exist.right45 = 0;
    }
    auto res = mp->go_straight(p, adachi, true);
    param->sen_ref_p.normal.exist.left45 = exist_left45;
    param->sen_ref_p.normal.exist.right45 = exist_right45;
    return res;
  }
  if ((near_left && right_exist)) {
    return straight_offset(p_set, TurnDirection::Right, diff);
  } else if ((near_right && left_exist)) {
    return straight_offset(p_set, TurnDirection::Left, diff);
  } else if ((near_left && !right_exist) || (near_left && left_exist) ||
             (far_left && !right_exist)) {
    return straight_offset(p_set, TurnDirection::Left, diff);
  } else if ((near_right && !left_exist) || (near_left && right_exist) ||
             (far_right && !left_exist)) {
    return straight_offset(p_set, TurnDirection::Right, diff);
  }
  p.search_str_wide_ctrl_l = left_exist;
  p.search_str_wide_ctrl_r = right_exist;
  return mp->go_straight(p, fake_adachi, false);

  return MotionResult::NONE;
}
MotionResult SearchController::slalom(param_set_t &p_set,
                                      const TurnDirection td, float diff) {
  slalom_param2_t sp = p_set.map[TurnType::Normal];

  next_motion_t nm;
  nm.v_max = p_set.map[TurnType::Normal].v;
  // nm.v_max = p_set.str_map[StraightType::Search].v_max;
  nm.v_end = p_set.map[TurnType::Normal].v;
  nm.accl = p_set.str_map[StraightType::Search].accl;
  nm.decel = p_set.str_map[StraightType::Search].decel;
  nm.is_turn = false;
  nm.skip_wall_off = false;

  return mp->slalom(sp, td, nm);
}
void SearchController::front_wall_ctrl() {
  // mp->reset_tgt_data();
  // mp->reset_ego_data();
  // vTaskDelay(25.0 / portTICK_RATE_MS);
  pt->motor_enable();
  mp->front_ctrl(false);
  vTaskDelay(5.0 / portTICK_RATE_MS);
  pt->motor_disable();
  mp->reset_tgt_data();
  mp->reset_ego_data();
}
MotionResult SearchController::pivot(param_set_t &p_set, float diff) {
  // param_straight_t p;
  MotionResult res;

  bool left_exist =
      sensing_result->ego.left45_dist < param->sen_ref_p.search_exist.left90;
  bool right_exist =
      sensing_result->ego.right45_dist < param->sen_ref_p.search_exist.right90;
  bool near =
      sensing_result->ego.left45_dist < sensing_result->ego.right45_dist;

  p.v_max = p_set.str_map[StraightType::Search].v_max;
  p.v_end = 20;
  p.accl = p_set.str_map[StraightType::Search].accl;
  p.decel = p_set.str_map[StraightType::Search].decel;
  p.dist = param->pivot_straight - diff;
  p.search_str_wide_ctrl_l = p.search_str_wide_ctrl_r = false;
  bool back_enable = false;
  if (10 < sensing_result->ego.left90_mid_dist &&
      sensing_result->ego.left90_mid_dist < param->pivot_back_enable_front_th &&
      10 < sensing_result->ego.right90_mid_dist &&
      sensing_result->ego.right90_mid_dist <
          param->pivot_back_enable_front_th) {
    p.dist += (sensing_result->ego.front_mid_dist - param->front_dist_offset0);
    back_enable = true;
  }
  p.motion_type = MotionType::PIVOT_PRE;
  p.sct = SensorCtrlType::Straight;
  p.wall_off_req = WallOffReq::NONE;
  if (p.dist < 10) {
    p.dist = 10;
  }
  res = mp->go_straight(p);

  // if (res == MotionResult::ERROR)
  //   return MotionResult::ERROR;

  p.v_max = 20;
  p.v_end = 5;
  p.accl = p_set.str_map[StraightType::Search].accl;
  p.decel = p_set.str_map[StraightType::Search].decel;
  p.dist = 2;
  p.motion_type = MotionType::PIVOT_PRE2;
  p.sct = SensorCtrlType::Straight;
  p.wall_off_req = WallOffReq::NONE;
  res = mp->go_straight(p);
  if (res == MotionResult::ERROR)
    return MotionResult::ERROR;

  bool flag = false;
  float tmp_dist = 0;
  pr.w_max = p_set.str_map[StraightType::Search].w_max;
  pr.alpha = p_set.str_map[StraightType::Search].alpha;
  pr.w_end = p_set.str_map[StraightType::Search].w_end;

  flag = (left_exist || right_exist);
  // pr.ang = (flag) ? m_PI / 2 : m_PI;
  pr.ang = (flag) ? param->pivot_angle_90 : param->pivot_angle_180;

  bool front_ctrl2 = false;
  if (!left_exist && !right_exist) {
    pr.RorL = TurnDirection::Right;
  } else if (left_exist && right_exist) {
    if (near) {
      pr.RorL = TurnDirection::Right;
    } else {
      pr.RorL = TurnDirection::Left;
    }
    front_ctrl2 = true;
  } else if (left_exist) {
    pr.RorL = TurnDirection::Left;
    front_ctrl2 = true;
  } else if (right_exist) {
    pr.RorL = TurnDirection::Right;
    front_ctrl2 = true;
  }

  bool front_ctrl =
      (sensing_result->ego.front_dist < param->front_dist_offset_pivot_th);
  pt->motor_disable();
  mp->reset_tgt_data();
  mp->reset_ego_data();

  if (adachi->goal_step && !saved) {
    vTaskDelay(1.0 / portTICK_RATE_MS);
    save_maze_data();
    vTaskDelay(1.0 / portTICK_RATE_MS);
    saved = true;
  }
  vTaskDelay(5.0 / portTICK_RATE_MS);

  mp->reset_tgt_data();
  mp->reset_ego_data();

  if (front_ctrl) {
    front_wall_ctrl();
  }

  // vTaskDelay(25.0 / portTICK_RATE_MS);
  // mp->reset_tgt_data();
  // mp->reset_ego_data();
  // mp->req_error_reset();
  // pt->motor_enable();

  if (flag) {
    p.motion_type = MotionType::PIVOT_OFFSET;
    p.sct = SensorCtrlType::NONE;
    // pr.ang = m_PI / 2;
    pr.ang = param->pivot_angle_90;
    res = mp->pivot_turn(pr);
    vTaskDelay(1.0 / portTICK_RATE_MS);
    front_ctrl = (sensing_result->ego.front_dist < param->search_front_ctrl_th);
    pt->motor_disable();
    vTaskDelay(1.0 / portTICK_RATE_MS);

    if (front_ctrl2) {
      front_wall_ctrl();
    }
    vTaskDelay(1.0 / portTICK_RATE_MS);

    // mp->reset_tgt_data();
    pt->motor_enable();
    mp->reset_tgt_data();
    mp->reset_ego_data();
    p.dist = 0.1;
    p.motion_type = MotionType::PIVOT_PRE2;
    p.sct = SensorCtrlType::NONE;
    res = mp->go_straight(p);
    pt->motor_disable();
    mp->reset_tgt_data();
    mp->reset_ego_data();
    vTaskDelay(5.0 / portTICK_RATE_MS);
    mp->reset_tgt_data();
    mp->reset_ego_data();
    pt->motor_enable();
    mp->reset_tgt_data();
    mp->reset_ego_data();
    res = mp->pivot_turn(pr);
    pt->motor_disable();
    mp->reset_tgt_data();
    mp->reset_ego_data();
    vTaskDelay(1.0 / portTICK_RATE_MS);
  } else {
    // pr.ang = m_PI;
    pr.ang = param->pivot_angle_180;
    res = mp->pivot_turn(pr);
    pt->motor_disable();
    mp->reset_tgt_data();
    mp->reset_ego_data();
    vTaskDelay(1.0 / portTICK_RATE_MS);
  }

  // if (res == MotionResult::ERROR)
  //   return MotionResult::ERROR;

  if (back_enable) {
    // ui->coin(100);
    vTaskDelay(5.0 / portTICK_RATE_MS);
    pt->motor_enable();
    mp->reset_tgt_data();
    mp->reset_ego_data();
    p.v_max = -p_set.str_map[StraightType::Search].v_max;
    p.v_end = -10; // p_set.str_map[StraightType::Search].v_max;
    p.accl = -p_set.str_map[StraightType::Search].accl;
    p.decel = -p_set.str_map[StraightType::Search].decel;
    p.dist = -param->pivot_back_dist0;
    p.motion_type = MotionType::BACK_STRAIGHT;
    p.sct = SensorCtrlType::NONE;
    p.wall_off_req = WallOffReq::NONE;
    res = mp->go_straight(p);
    mp->reset_tgt_data();
    mp->reset_ego_data();
    vTaskDelay(5.0 / portTICK_RATE_MS);
    pt->motor_disable();
    vTaskDelay(5.0 / portTICK_RATE_MS);
  } else {
    vTaskDelay(5.0 / portTICK_RATE_MS);
    pt->motor_enable();
    mp->reset_tgt_data();
    mp->reset_ego_data();
    p.v_max = -p_set.str_map[StraightType::Search].v_max;
    p.v_end = -10; // p_set.str_map[StraightType::Search].v_max;
    p.accl = -p_set.str_map[StraightType::Search].accl;
    p.decel = -p_set.str_map[StraightType::Search].decel;
    p.dist = -param->pivot_back_dist1;
    p.motion_type = MotionType::BACK_STRAIGHT;
    p.sct = SensorCtrlType::NONE;
    p.wall_off_req = WallOffReq::NONE;
    res = mp->go_straight(p);
    mp->reset_tgt_data();
    mp->reset_ego_data();
    vTaskDelay(5.0 / portTICK_RATE_MS);
    pt->motor_disable();
    vTaskDelay(5.0 / portTICK_RATE_MS);
  }

  // mp->reset_gyro_ref();
  // ui->coin(100);

  if (adachi != nullptr) {
    adachi->update();
  }

  vTaskDelay(5.0 / portTICK_RATE_MS);
  mp->reset_tgt_data();
  mp->reset_ego_data();
  pt->motor_enable();
  // p.v_max = p_set.str_map[StraightType::Search].v_max;
  // p.v_end = p_set.str_map[StraightType::Search].v_max;

  p.v_max = p.v_end = p_set.map[TurnType::Normal].v;

  p.accl = p_set.str_map[StraightType::Search].accl;
  p.decel = p_set.str_map[StraightType::Search].decel;
  if (back_enable) {
    p.dist =
        param->cell / 2 + param->pivot_back_dist0 - param->pivot_back_offset;
  } else {
    p.dist = param->cell / 2;
  }
  p.motion_type = MotionType::PIVOT_AFTER;
  p.sct = SensorCtrlType::Straight;
  p.wall_off_req = WallOffReq::NONE;
  pt->motor_enable();
  res = mp->go_straight(p);
  // ui->coin(100);

  return res;
}

MotionResult SearchController::pivot90(param_set_t &p_set,
                                       const TurnDirection td, float diff) {
  // param_straight_t p;
  MotionResult res;
  const auto tmp_v = param->cell / 2;
  p.v_max = p_set.str_map[StraightType::Search].v_max;
  p.v_end = tmp_v;
  p.accl = p_set.str_map[StraightType::Search].accl;
  p.decel = p_set.str_map[StraightType::Search].decel;
  p.dist = param->cell / 2 - 5 - diff;
  p.search_str_wide_ctrl_l = p.search_str_wide_ctrl_r = false;
  if (10 < sensing_result->ego.left90_mid_dist &&
      sensing_result->ego.left90_mid_dist < param->pivot_back_enable_front_th &&
      10 < sensing_result->ego.right90_mid_dist &&
      sensing_result->ego.right90_mid_dist <
          param->pivot_back_enable_front_th) {
    p.dist +=
        (sensing_result->ego.front_mid_dist - param->front_dist_offset_pivot);
  }
  p.motion_type = MotionType::PIVOT_PRE;
  p.sct = SensorCtrlType::Straight;
  p.wall_off_req = WallOffReq::NONE;

  // float offset_dist = 45;

  if (p.dist < 10) {
    p.dist = 10;
  }

  res = mp->go_straight(p);

  p.v_max = tmp_v;
  p.v_end = 5;
  p.accl = p_set.str_map[StraightType::Search].accl;
  p.decel = p_set.str_map[StraightType::Search].decel;
  p.dist = 2;
  p.motion_type = MotionType::PIVOT_PRE2;
  p.sct = SensorCtrlType::Straight;
  p.wall_off_req = WallOffReq::NONE;
  res = mp->go_straight(p);
  if (res == MotionResult::ERROR)
    return MotionResult::ERROR;

  bool front_ctrl =
      (sensing_result->ego.front_dist < param->search_front_ctrl_th);
  if (front_ctrl) {
    front_wall_ctrl();
    // vTaskDelay(25.0 / portTICK_RATE_MS);
  }

  pr.w_max = p_set.str_map[StraightType::Search].w_max;
  pr.alpha = p_set.str_map[StraightType::Search].alpha;
  pr.w_end = p_set.str_map[StraightType::Search].w_end;
  // pr.ang = m_PI / 2;
  pr.ang = param->pivot_angle_90;
  pr.RorL = td;

  // pt->motor_disable();
  // mp->reset_tgt_data();
  // mp->reset_ego_data();
  // vTaskDelay(25.0 / portTICK_RATE_MS);

  if (adachi != nullptr) {
    adachi->update();
    vTaskDelay(10.0 / portTICK_RATE_MS);
  }

  // mp->reset_tgt_data();
  // mp->reset_ego_data();
  // pt->motor_enable();
  res = mp->pivot_turn(pr);
  pt->motor_disable();
  mp->reset_tgt_data();
  mp->reset_ego_data();
  // vTaskDelay(25.0 / portTICK_RATE_MS);

  // mp->reset_tgt_data();
  // mp->reset_ego_data();

  p.v_max = p_set.str_map[StraightType::Search].v_max;
  p.v_end = p_set.str_map[StraightType::Search].v_max;
  p.accl = p_set.str_map[StraightType::Search].accl;
  p.decel = p_set.str_map[StraightType::Search].decel;
  p.search_str_wide_ctrl_l = p.search_str_wide_ctrl_r = false;
  p.dist = param->cell / 2;
  // if (offset_dist != 45) {
  //   p.dist = offset_dist;
  // }
  p.motion_type = MotionType::PIVOT_AFTER;
  p.sct = SensorCtrlType::Straight;
  p.wall_off_req = WallOffReq::NONE;
  pt->motor_enable();
  res = mp->go_straight(p);

  return res;
}

MotionResult SearchController::timeup_motion(param_set_t &p_set) {
  // param_straight_t p;
  MotionResult res;

  pr.w_max = p_set.str_map[StraightType::Search].w_max / 2;
  pr.alpha = p_set.str_map[StraightType::Search].alpha;
  pr.w_end = p_set.str_map[StraightType::Search].w_end;
  pr.ang = m_PI * 2 * 2;
  pr.RorL = TurnDirection::Right;

  res = mp->pivot_turn(pr);
  pt->motor_disable();
  mp->reset_tgt_data();
  mp->reset_ego_data();

  return res;
}

MotionResult SearchController::finish(param_set_t &p_set) {
  // param_straight_t p;
  p.v_max = p_set.str_map[StraightType::Search].v_max;
  p.v_end = 10;
  p.accl = p_set.str_map[StraightType::Search].accl;
  p.decel = p_set.str_map[StraightType::Search].decel;
  p.dist = param->cell / 2 - 5;
  p.search_str_wide_ctrl_l = p.search_str_wide_ctrl_r = false;
  mp->go_straight(p);

  p.v_max = 20;
  p.v_end = 5;
  p.accl = p_set.str_map[StraightType::Search].accl;
  p.decel = p_set.str_map[StraightType::Search].decel;
  p.dist = 2;
  return mp->go_straight(p);
}

SearchResult SearchController::exec(param_set_t &p_set, SearchMode sm) {
  unsigned long long start_time = pt->global_msec_timer;
  unsigned long long end_time = pt->global_msec_timer;
  pt->search_mode = true;
  adachi->lgc->reset_dist_map();
  mp->reset_gyro_ref_with_check();
  if (param->search_log_enable > 0)
    lt->start_slalom_log();
  reset();

  for (const auto p : lgc->goal_list) {
    lgc->map[p.x + p.y * lgc->maze_size] =
        (lgc->map[p.x + p.y * lgc->maze_size] & 0x0f);
  }

  p.v_max = p.v_end = p_set.str_map[StraightType::Search].v_max;
  p.accl = p_set.str_map[StraightType::Search].accl;
  p.decel = p_set.str_map[StraightType::Search].decel;
  p.dist = param->cell / 2 + param->offset_start_dist_search;
  p.sct = SensorCtrlType::Straight;
  p.motion_type = MotionType::STRAIGHT;
  // p.wall_off_req = WallOffReq::SEARCH;
  p.wall_off_req = WallOffReq::NONE;
  p.wall_off_dist_r = param->sen_ref_p.search_exist.offset_r;
  p.wall_off_dist_l = param->sen_ref_p.search_exist.offset_l;

  pt->motor_enable();
  mp->reset_tgt_data();
  mp->reset_ego_data();
  mp->go_straight(p);

  int back_cnt = 0;
  bool first = false;
  // pivot(p_set);
  saved = false;
  int go_home_fix_cnt = 0;
  adachi->sm = sm;
  bool timeup = false;
  bool state = true;
  int goal_state = 0;
  while (1) {
    mr = MotionResult::NONE;
    // sensing(ego);
    bool is_stepped = lgc->is_stepped(ego->x, ego->y);
    bool front_is_stepped =
        lgc->is_front_cell_stepped(ego->x, ego->y, ego->dir);
    judge_wall();
    // 片道モードでゴールしたらbreak
    if (sm == SearchMode::Kata) {
      if (adachi->goal_step) {
        break;
      }
    }
    //往復モードでゴールしたらbreak
    if (sm == SearchMode::Return) {
      if (adachi->goal_step) {
        if (ego->x == 0 && ego->y == 0) {
          break;
        } else {
          adachi->subgoal_list.clear();
        }
      }
    }

    // 足立法で行き先決定
    const float before = mp->tgt_val->global_pos.dist;
    Motion next_motion = Motion::NONE;
    // if (adachi->goal_step && goal_state == 0) {
    //   goal_state = 1;
    //   next_motion = adachi->exec(false, true);
    // } else {
    // }
    next_motion = adachi->exec(false, false);

    const float after = mp->tgt_val->global_pos.dist;
    adachi->diff = std::abs(after - before);

    // go_straight_wrapper(p_set);
    if (next_motion == Motion::Straight) {
      Motion next_motion2 = Motion::NONE;
      // const float before2 = mp->tgt_val->global_pos.dist;
      if (front_is_stepped) {
        next_motion2 = adachi->exec(true, false);
      }
      const float after2 = mp->tgt_val->global_pos.dist;
      if (front_is_stepped && next_motion2 == Motion::Straight) {
        adachi->diff = std::abs(after2 - before);
        mr = go_straight_wrapper(p_set, adachi->diff, StraightType::FastRun);
      } else if (front_is_stepped && next_motion2 != Motion::Straight) {
        adachi->diff = std::abs(after2 - before);
        mr = go_straight_wrapper(p_set, adachi->diff, StraightType::FastRunDia);
      } else {
        mr = go_straight_wrapper(p_set, adachi->diff, StraightType::Search);
      }
    } else if (next_motion == Motion::TurnRight) {
      // mr = slalom(p_set, TurnDirection::Right, adachi->diff);
      if (judge2()) {
        mr = pivot90(p_set, TurnDirection::Right, adachi->diff);
      } else {
        mr = slalom(p_set, TurnDirection::Right, adachi->diff);
      }
    } else if (next_motion == Motion::TurnLeft) {
      // mr = slalom(p_set, TurnDirection::Left, adachi->diff);
      if (judge2()) {
        mr = pivot90(p_set, TurnDirection::Left, adachi->diff);
      } else {
        mr = slalom(p_set, TurnDirection::Left, adachi->diff);
      }
    } else if (next_motion == Motion::Back) {
      mr = pivot(p_set, adachi->diff);
      // break;
    } else if (next_motion == Motion::NONE) {
      break;
    } else {
      break;
    }

    if (mr == MotionResult::ERROR) {
      state = false;
      break;
    }

    if (next_motion == Motion::Back) {
      back_cnt++;
    } else {
      back_cnt = 0;
    }

    if (back_cnt >= 4) {
      state = false;
      break;
    }
    if (sm == SearchMode::ALL) {
      if (adachi->goal_step) {
        if (ego->x == 0 && ego->y == 0) {
          break;
        }
        if (adachi->subgoal_list.size() == 0 && adachi->pt_list.size() == 1 &&
            adachi->pt_list[0].x == 0 && adachi->pt_list[0].y == 0) {
          // break;
          go_home_fix_cnt++;
        } else {
          go_home_fix_cnt = 0;
        }
        if (go_home_fix_cnt == 10) {
          // break;
        }
      }
    }
    end_time = pt->global_msec_timer;
    if (ABS((end_time - start_time) / 1000) > param->seach_timer) {
      timeup = true;
      break;
    }
  }
  finish(p_set);
  pt->motor_disable();
  if (timeup) {
    timeup_motion(p_set);
  }
  if (state) {
    save_maze_data();
    mp->coin();
    mp->coin();
  }
  lt->stop_slalom_log();
  lt->save(slalom_log_file);
  mp->coin();
  pt->search_mode = false;
  while (1) {
    if (ui->button_state_hold())
      break;
    vTaskDelay(10.0 / portTICK_RATE_MS);
  }
  if (param->search_log_enable > 0) {
    lt->dump_log(slalom_log_file);
  }
  if (mr == MotionResult::ERROR) {
    return SearchResult::FAIL;
  }
  return SearchResult::SUCCESS;
}

MotionResult SearchController::straight_offset(param_set_t &p_set,
                                               const TurnDirection td,
                                               float diff) {
  p.v_max = p_set.str_map[StraightType::Search].v_max;
  p.v_end = 5;
  p.accl = p_set.str_map[StraightType::Search].accl;
  p.decel = p_set.str_map[StraightType::Search].decel;
  p.dist = param->cell2 / 2 - diff;
  p.motion_type = MotionType::PIVOT_PRE2;
  p.sct = SensorCtrlType::Straight;
  p.wall_off_req = WallOffReq::NONE;
  mp->go_straight(p);
  pt->motor_disable();
  mp->reset_tgt_data();
  mp->reset_ego_data();
  vTaskDelay(25.0 / portTICK_RATE_MS);

  pr.w_max = p_set.str_map[StraightType::Search].w_max;
  pr.alpha = p_set.str_map[StraightType::Search].alpha;
  pr.w_end = p_set.str_map[StraightType::Search].w_end;
  // pr.ang = m_PI / 2;
  pr.ang = param->pivot_angle_90;
  pr.RorL = td;

  pt->motor_enable();
  mp->reset_tgt_data();
  mp->reset_ego_data();
  mp->pivot_turn(pr);
  pt->motor_disable();
  mp->reset_tgt_data();
  mp->reset_ego_data();
  front_wall_ctrl();
  vTaskDelay(25.0 / portTICK_RATE_MS);

  pr.RorL =
      (td == TurnDirection::Right) ? TurnDirection::Left : TurnDirection::Right;

  pt->motor_enable();
  mp->reset_tgt_data();
  mp->reset_ego_data();
  mp->pivot_turn(pr);
  pt->motor_disable();
  mp->reset_tgt_data();
  mp->reset_ego_data();

  p.v_max = p_set.str_map[StraightType::Search].v_max;
  p.v_end = p_set.str_map[StraightType::Search].v_max;
  p.accl = p_set.str_map[StraightType::Search].accl;
  p.decel = p_set.str_map[StraightType::Search].decel;
  p.dist = param->cell2 / 2;
  p.motion_type = MotionType::PIVOT_AFTER;
  p.sct = SensorCtrlType::Straight;
  p.wall_off_req = WallOffReq::NONE;
  pt->motor_enable();
  return mp->go_straight(p);
}
bool SearchController::judge2() {
  const auto near = (10 < sensing_result->ego.left90_mid_dist &&
                     sensing_result->ego.left90_mid_dist <
                         param->front_dist_offset_pivot_th &&
                     10 < sensing_result->ego.right90_mid_dist &&
                     sensing_result->ego.right90_mid_dist <
                         param->front_dist_offset_pivot_th);

  const auto diff =
      (10 < sensing_result->ego.left90_mid_dist) &&
      (10 < sensing_result->ego.right90_mid_dist) &&
      (sensing_result->ego.left90_mid_dist < param->judge_pivot) &&
      (sensing_result->ego.right90_mid_dist < param->judge_pivot) &&
      (std::abs(sensing_result->ego.left90_mid_dist -
                sensing_result->ego.right90_mid_dist) > param->front_diff_th);

  return near | diff;
}

void SearchController::judge_wall() {
  wall_n = false;
  wall_e = false;
  wall_w = false;
  wall_s = false;

  if (ego->dir == Direction::North) {
    wall_n = sensing_result->ego.left90_mid_dist <
                 param->sen_ref_p.search_exist.front &&
             sensing_result->ego.right90_mid_dist <
                 param->sen_ref_p.search_exist.front;
    wall_e = sensing_result->ego.right45_dist <
             param->sen_ref_p.search_exist.right45;
    wall_w =
        sensing_result->ego.left45_dist < param->sen_ref_p.search_exist.left45;
  } else if (ego->dir == Direction::East) {
    wall_e = sensing_result->ego.left90_mid_dist <
                 param->sen_ref_p.search_exist.front &&
             sensing_result->ego.right90_mid_dist <
                 param->sen_ref_p.search_exist.front;
    wall_s = sensing_result->ego.right45_dist <
             param->sen_ref_p.search_exist.right45;
    wall_n =
        sensing_result->ego.left45_dist < param->sen_ref_p.search_exist.left45;
  } else if (ego->dir == Direction::West) {
    wall_w = sensing_result->ego.left90_mid_dist <
                 param->sen_ref_p.search_exist.front &&
             sensing_result->ego.right90_mid_dist <
                 param->sen_ref_p.search_exist.front;
    wall_n = sensing_result->ego.right45_dist <
             param->sen_ref_p.search_exist.right45;
    wall_s =
        sensing_result->ego.left45_dist < param->sen_ref_p.search_exist.left45;
  } else if (ego->dir == Direction::South) {
    wall_s = sensing_result->ego.left90_mid_dist <
                 param->sen_ref_p.search_exist.front &&
             sensing_result->ego.right90_mid_dist <
                 param->sen_ref_p.search_exist.front;
    wall_w = sensing_result->ego.right45_dist <
             param->sen_ref_p.search_exist.right45;
    wall_e =
        sensing_result->ego.left45_dist < param->sen_ref_p.search_exist.left45;
  }
  lgc->set_wall_data(ego->x, ego->y, Direction::North, wall_n);
  lgc->set_wall_data(ego->x, ego->y + 1, Direction::South, wall_n);

  lgc->set_wall_data(ego->x, ego->y, Direction::East, wall_e);
  lgc->set_wall_data(ego->x + 1, ego->y, Direction::West, wall_e);

  lgc->set_wall_data(ego->x, ego->y, Direction::West, wall_w);
  lgc->set_wall_data(ego->x - 1, ego->y, Direction::East, wall_w);

  lgc->set_wall_data(ego->x, ego->y, Direction::South, wall_s);
  lgc->set_wall_data(ego->x, ego->y - 1, Direction::North, wall_s);

  lgc->set_wall_data(0, 0, Direction::East, true);
  lgc->set_wall_data(1, 0, Direction::West, true);
}
void SearchController::set_input_param_entity(
    std::shared_ptr<input_param_t> &_param) {
  param = _param;
}

void SearchController::print_maze() {
  printf("\r\n");
  for (int j = lgc->maze_size - 1; j >= 0; j--) {
    printf("+");
    for (int i = 0; i < lgc->maze_size; i++) {
      if (lgc->existWall(i, j, Direction::North)) {
        if (lgc->maze_size == 16)
          printf("---+");
        else
          printf("----+");
      } else {
        printf("    +");
      }
    }
    printf("\r\n");
    for (int i = 0; i < lgc->maze_size; i++) {
      if (lgc->existWall(i, j, Direction::West)) {
        printf("| ");
      } else {
        printf("  ");
      }
      if (lgc->maze_size == 16)
        printf("%2x ", lgc->dist[i + j * lgc->maze_size]);
      else
        printf("%3x", lgc->dist[i + j * lgc->maze_size]);
    }
    printf("|\r\n");
  }
  printf("+");

  for (int i = 0; i < lgc->maze_size; i++) {
    if (lgc->existWall(i, 0, Direction::South)) {
      if (lgc->maze_size == 16)
        printf("---+");
      else
        printf("----+");
    } else {
      if (lgc->maze_size == 16)
        printf("    +");
      else
        printf("     +");
    }
  }
  printf("\r\n");
}
void SearchController::save_maze_data() {
  // auto *f = fopen(maze_log_kata_file.c_str(), "wb");
  mount();
  auto *f = fopen(maze_log_file.c_str(), "wb");
  if (f == NULL)
    return;
  for (const auto d : lgc->map) {
    fprintf(f, "%d,", d);
  }
  fclose(f);
  umount();
}