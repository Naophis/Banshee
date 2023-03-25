#include "adachi.hpp"

Adachi::Adachi() {}

Adachi::~Adachi() {}

void Adachi::set_logic(std::shared_ptr<MazeSolverBaseLgc> &_lgc) { lgc = _lgc; }

void Adachi::set_ego(std::shared_ptr<ego_t> &_ego) { ego = _ego; }

void Adachi::setNextDirection(int x2, int y2, Direction dir,
                              Direction &next_dir, int &val) {
  bool isWall = lgc->existWall(ego->x, ego->y, dir);
  bool step = lgc->isStep(x2, y2, dir);
  unsigned int dist = lgc->get_dist_val(x2, y2);

  bool bool1 = goaled && (pt_list.size() == 1);

  if (!goaled || is_go_home() || (bool1)) {
    if (!isWall && !step && dist < val) {
      next_dir = dir;
      val = dist;
    } else if (!isWall && step && dist < val) {
      next_dir = dir;
      val = dist;
    }
  } else {
    //  if (!isWall && !step2) {
    //    next_dir = dir;
    //    val = dist; //未探索有線固定
    //  } else
    if (!isWall && !step && dist < val) {
      next_dir = dir;
      val = dist;
    } else if (!isWall && step && dist < val) {
      next_dir = dir;
      val = dist;
    }
  }
}
void Adachi::setNextDirection2(int x2, int y2, Direction dir,
                               Direction &next_dir, int &val) {
  bool isWall = lgc->existWall(ego->x, ego->y, dir);
  bool step = lgc->isStep(x2, y2, dir);
  unsigned int dist = lgc->get_dist_val(x2, y2);

  if (!goaled) {
    if (!isWall && !step && dist < val) {
      next_dir = dir;
      val = dist;
    } else if (!isWall && step && dist < val) {
      next_dir = dir;
      val = dist;
    }
  } else {
    //  if (!isWall && !step2) {
    //    next_dir = dir;
    //    val = dist; //未探索有線固定
    //  } else
    if (!isWall && !step && dist <= val) {
      next_dir = dir;
      val = dist;
    } else if (!isWall && step && dist < val) {
      next_dir = dir;
      val = dist;
    }
  }
}

bool Adachi::is_goal(int x, int y) {
  for (const auto p : lgc->goal_list)
    if (p.x == x && p.y == 0)
      return true;
  return false;
}

void Adachi::deadEnd(int egox, int egoy) {
  bool head = true;
  while (head) {
    head = false;
    for (char i = -1; i <= 1; i++) {
      for (char j = -1; j <= 1; j++) {
        const unsigned char x = egox + i;
        const unsigned char y = egoy + j;
        const unsigned char temp = lgc->get_map_val(x, y) & 0x0f;
        if (x == 0 && y == 0)
          continue;
        if (is_goal(x, y))
          continue;
        if (egox == x && egoy == y)
          continue;

        if (temp == 0x07) {
          lgc->updateWall(x, y, Direction::South);
          head = true;
        }
        if (temp == 0x0b) {
          lgc->updateWall(x, y, Direction::West);
          head = true;
        }
        if (temp == 0x0d) {
          lgc->updateWall(x, y, Direction::East);
          head = true;
        }
        if (temp == 0x0e) {
          lgc->updateWall(x, y, Direction::North);
          head = true;
        }
      }
    }
  }
}

Direction Adachi::detect_next_direction() {
  int dist_val = lgc->get_max_step_val();
  Direction next_dir = Direction::Undefined;

  const bool enable_back = lgc->get_dist_val(ego->x, ego->y) > limit2;
  // const bool stop = (!is_go_home() && ego->x == 0 && ego->y == 1);
  const bool stop = true;
  if (ego->dir == Direction::North) {
    setNextDirection(ego->x, ego->y + 1, Direction::North, next_dir, dist_val);
    setNextDirection2(ego->x + 1, ego->y, Direction::East, next_dir, dist_val);
    setNextDirection2(ego->x - 1, ego->y, Direction::West, next_dir, dist_val);
    if (enable_back || is_go_home() || stop)
      setNextDirection(ego->x, ego->y - 1, Direction::South, next_dir,
                       dist_val);
  } else if (ego->dir == Direction::East) {
    setNextDirection(ego->x + 1, ego->y, Direction::East, next_dir, dist_val);
    setNextDirection2(ego->x, ego->y - 1, Direction::South, next_dir, dist_val);
    setNextDirection2(ego->x, ego->y + 1, Direction::North, next_dir, dist_val);
    if (enable_back || is_go_home() || stop)
      setNextDirection(ego->x - 1, ego->y, Direction::West, next_dir, dist_val);
  } else if (ego->dir == Direction::West) {
    setNextDirection(ego->x - 1, ego->y, Direction::West, next_dir, dist_val);
    setNextDirection2(ego->x, ego->y + 1, Direction::North, next_dir, dist_val);
    setNextDirection2(ego->x, ego->y - 1, Direction::South, next_dir, dist_val);
    if (enable_back || is_go_home() || stop)
      setNextDirection(ego->x + 1, ego->y, Direction::East, next_dir, dist_val);
  } else if (ego->dir == Direction::South) {
    setNextDirection(ego->x, ego->y - 1, Direction::South, next_dir, dist_val);
    setNextDirection2(ego->x - 1, ego->y, Direction::West, next_dir, dist_val);
    setNextDirection2(ego->x + 1, ego->y, Direction::East, next_dir, dist_val);
    if (enable_back || is_go_home() || stop)
      setNextDirection(ego->x, ego->y + 1, Direction::North, next_dir,
                       dist_val);
  }
  return next_dir;
}

void Adachi::get_next_pos(Direction next_direction) {
  if (next_direction == Direction::Undefined) {
    if (ego->dir == Direction::North)
      next_direction = Direction::South;
    else if (ego->dir == Direction::East)
      next_direction = Direction::West;
    else if (ego->dir == Direction::West)
      next_direction = Direction::East;
    else if (ego->dir == Direction::South)
      next_direction = Direction::North;
  }

  if (next_direction == Direction::North)
    ego->y++;
  else if (next_direction == Direction::East)
    ego->x++;
  else if (next_direction == Direction::West)
    ego->x--;
  else if (next_direction == Direction::South)
    ego->y--;

  ego->dir = next_direction;
  return;
}

Motion Adachi::get_next_motion(Direction next_direction) {
  if (ego->dir == next_direction)
    return Motion::Straight;

  if (ego->dir == Direction::North) {
    if (next_direction == Direction::East)
      return Motion::TurnRight;
    else if (next_direction == Direction::West)
      return Motion::TurnLeft;
  } else if (ego->dir == Direction::East) {
    if (next_direction == Direction::South)
      return Motion::TurnRight;
    else if (next_direction == Direction::North)
      return Motion::TurnLeft;
  } else if (ego->dir == Direction::West) {
    if (next_direction == Direction::North)
      return Motion::TurnRight;
    else if (next_direction == Direction::South)
      return Motion::TurnLeft;
  } else if (ego->dir == Direction::South) {
    if (next_direction == Direction::West)
      return Motion::TurnRight;
    else if (next_direction == Direction::East)
      return Motion::TurnLeft;
  }
  return Motion::Back;
}

bool Adachi::is_go_home() {
  if (pt_list.size() == 1 && pt_list[0].x == 0 && pt_list[0].y == 0)
    return true;
  return false;
}
void Adachi::goal_step_check() {
  for (auto it = lgc->goal_list_origin.begin();
       it != lgc->goal_list_origin.end(); it++) {
    if ((*it).x == ego->x && (*it).y == ego->y) {
      it = lgc->goal_list_origin.erase(it);
      break;
    }
  }
  if (lgc->goal_list_origin.size() == 1) {
    for (auto it = lgc->goal_list_origin.begin();
         it != lgc->goal_list_origin.end(); it++) {
      if (lgc->isStep((*it).x, (*it).y, Direction::North) &&
          lgc->isStep((*it).x, (*it).y, Direction::East) &&
          lgc->isStep((*it).x, (*it).y, Direction::West) &&
          lgc->isStep((*it).x, (*it).y, Direction::South)) {
        it = lgc->goal_list_origin.erase(it);
        break;
      }
    }
  }
  if (lgc->goal_list_origin.size() == 0) {
    goal_step = true;
  } else {
    goal_step = false;
  }
}

Motion Adachi::exec(bool is_stepped, bool force_back) {
  int calc_cnt = 0;
  goal_step_check();
  goaled = goal_step;

  if (!goaled) {
    pt_list.clear();
    pt_list.shrink_to_fit();
  }
  if (goaled) {
    subgoal_list.erase(ego->x + ego->y * lgc->maze_size);
  }

  // deadEnd(ego->x, ego->y);
  if (goaled) {
    if (subgoal_list.size() == 0) {
      pt_list.clear();
      pt_list.shrink_to_fit();
      tmp_p.x = tmp_p.y = 0;
      pt_list.emplace_back(tmp_p);
    } else {
      pt_list.clear();
      pt_list.shrink_to_fit();
      for (auto itr = subgoal_list.begin(); itr != subgoal_list.end(); ++itr) {
        tmp_p.x = itr->first % lgc->maze_size;
        tmp_p.y = itr->first / lgc->maze_size;
        pt_list.emplace_back(tmp_p);
      }
    }
    lgc->set_goal_pos2(pt_list);
    if (!is_stepped) {
      lgc->update_dist_map(0, goaled); // 再更新したらもう1回歩数マップ生成
    }
    if (pt_list.size() == 1 && pt_list[0].x == 0 && pt_list[0].y == 0) {
      goal_startpos_lock = true; //ゴール固定
    } else {
      goal_startpos_lock = false;
    }
  } else {
    if (!is_stepped) {
      lgc->update_dist_map(0, goaled); // search
    }
  }

  if (goal_startpos_lock) {
    if (ego->x == 0 && ego->y == 0) {
      lgc->set_param3();
      calc_cnt += lgc->searchGoalPosition(false, subgoal_list);
      if (!is_stepped) {
        lgc->update_dist_map(1, false); // search
      }
      return Motion::NONE;
    }
  }
  Direction next_dir = detect_next_direction();
  if (force_back) {
    if (ego->dir == Direction::North) {
      next_dir = Direction::South;
    } else if (ego->dir == Direction::East) {
      next_dir = Direction::West;
    } else if (ego->dir == Direction::West) {
      next_dir = Direction::East;
    } else if (ego->dir == Direction::South) {
      next_dir = Direction::North;
    }
  }
  Motion next_motion = get_next_motion(next_dir);

  if (!is_stepped) {
    get_next_pos(next_dir);
    lgc->remove_goal_pos3();
    goal_step_check();
  }

  // create_path(path, next_motion);
  // ROS_INFO("calc_cnt = %d", calc_cnt);

  // return static_cast<int>(next_motion);
  return next_motion;
}

void Adachi::update() {
  goal_step_check();
  if (goal_step && sm == SearchMode::ALL) {
    if ((subgoal_list.find(ego->x + ego->y * lgc->maze_size) !=
         subgoal_list.end())) {
      subgoal_list.erase(ego->x + ego->y * lgc->maze_size);
    }
    {
      lgc->set_param3();
      lgc->searchGoalPosition(true, subgoal_list);
      cost_mode = 3;
    }
    // if (subgoal_list.size() == 0) {
    //   lgc->set_param4();
    //   lgc->searchGoalPosition(true, subgoal_list);
    //   cost_mode = 4;
    // }
    // if (subgoal_list.size() == 0) {
    //   lgc->set_param1();
    //   lgc->searchGoalPosition(true, subgoal_list);
    //   cost_mode = 1;
    // }
    // if (subgoal_list.size() == 0) {
    //   lgc->set_param2();
    //   lgc->searchGoalPosition(true, subgoal_list);
    //   cost_mode = 2;
    // }
  }
  // lgc->reset_dist_map();
  // if (subgoal_list.size() == 0) {
  //   pt_list.clear();
  //   tmp_p.x = tmp_p.y = 0;
  //   pt_list.emplace_back(tmp_p);
  // } else {
  //   pt_list.clear();
  //   for (auto itr = subgoal_list.begin(); itr != subgoal_list.end(); ++itr) {
  //     tmp_p.x = itr->first % lgc->maze_size;
  //     tmp_p.y = itr->first / lgc->maze_size;
  //     pt_list.emplace_back(tmp_p);
  //   }
  // }
  // lgc->set_goal_pos2(pt_list);
  // lgc->update_dist_map(0, goaled); // 再更新したらもう1回歩数マップ生成
}