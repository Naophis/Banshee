
#include "logic.hpp"

void MazeSolverBaseLgc::init(const int _maze_size, const int _max_step_val) {
  maze_size = _maze_size;
  max_step_val = _max_step_val;
  maze_list_size = maze_size * maze_size;

  map.resize(maze_list_size);
  dist.resize(maze_list_size);
  vector_dist.resize(maze_list_size);
  updateMap.resize(maze_list_size);
  q_list.resize(maze_list_size + 1);
  while (!vq_list.empty()) {
    vq_list.pop();
  }
  goal_list3.clear();
}

void MazeSolverBaseLgc::data_economize() {
  map.clear();
  dist.clear();
  vector_dist.clear();
  updateMap.clear();
  q_list.clear();
  // vq_list.clear();
  while (!vq_list.empty()) {
    vq_list.pop();
  }
  goal_list3.clear();
}

void MazeSolverBaseLgc::set_ego(std::shared_ptr<ego_t> &_ego) { ego = _ego; }

void MazeSolverBaseLgc::set_goal_pos(const vector<point_t> &list) {
  goal_list.clear();
  goal_list_origin.clear();
  goal_list.shrink_to_fit();
  goal_list_origin.shrink_to_fit();
  for (const auto p : list) {
    goal_list.emplace_back(p);
    goal_list_origin.emplace_back(p);
  }
}

void MazeSolverBaseLgc::set_goal_pos2(const vector<point_t> &pt_list) {
  goal_list2.clear();
  goal_list2.shrink_to_fit();
  for (const auto p : pt_list)
    goal_list2.emplace_back(p);
}

bool MazeSolverBaseLgc::valid_map_list_idx(const int x, const int y) {
  if (x < 0 || x >= maze_size || y < 0 || y >= maze_size)
    return false;

  if ((x + y * maze_size) >= maze_list_size)
    return false;

  return true;
}

bool MazeSolverBaseLgc::candidate_end(const int x, const int y) {
  unsigned char temp = map[x + y * maze_size] & 0x0f;
  return (temp == 0x0e || temp == 0x0d || temp == 0x0b || temp == 0x07 ||
          temp == 0x0f);
}
void MazeSolverBaseLgc::updateWall(int x, int y, Direction dir) {
  if (dir == Direction::North) {
    set_wall_data(x, y, Direction::North, true);
    set_wall_data(x, y + 1, Direction::South, true);
  } else if (dir == Direction::East) {
    set_wall_data(x, y, Direction::East, true);
    set_wall_data(x + 1, y, Direction::West, true);
  } else if (dir == Direction::West) {
    set_wall_data(x, y, Direction::West, true);
    set_wall_data(x - 1, y, Direction::East, true);
  } else if (dir == Direction::South) {
    set_wall_data(x, y, Direction::South, true);
    set_wall_data(x, y - 1, Direction::North, true);
  }
}

void MazeSolverBaseLgc::remove_goal_pos3() {
  for (auto it = goal_list3.begin(); it != goal_list3.end();) {
    bool flag1 = false;
    flag1 |= is_stepped((*it).x, (*it).y);
    flag1 |= get_dist_val((*it).x, (*it).y) == max_step_val;

    if (flag1)
      it = goal_list3.erase(it);
    else
      it++;
  }
}

void MazeSolverBaseLgc::reset_dist_map() {
  int c = 0;
  for (int i = 0; i < maze_size; i++)
    for (int j = 0; j < maze_size; j++)
      dist[c++] = max_step_val;
  reset_done = true;
}

void MazeSolverBaseLgc::update_dist_map(const int mode,
                                        const bool search_mode) {
  int c = 0;
  if (!reset_done) {
    for (int i = 0; i < maze_size; i++) {
      for (int j = 0; j < maze_size; j++) {
        dist[c++] = max_step_val;
      }
    }
  }
  reset_done = false;

  int head = 0;
  int tail = 0;
  if (!search_mode) {
    for (const auto g : goal_list) {
      int idx = g.x + g.y * maze_size;
      dist[idx] = 0;
      q_list[tail].x = g.x;
      q_list[tail].y = g.y;
      tail++;
    }
    // for (const auto g : goal_list3) {
    //   int idx = g.x + g.y * maze_size;
    //   dist[idx] = 0;
    //   q_list[tail].x = g.x;
    //   q_list[tail].y = g.y;
    //   tail++;
    // }
  } else {
    for (const auto g : goal_list2) {
      int idx = g.x + g.y * maze_size;
      dist[idx] = 0;
      q_list[tail].x = g.x;
      q_list[tail].y = g.y;
      tail++;
    }
    for (const auto g : goal_list3) {
      int idx = g.x + g.y * maze_size;
      dist[idx] = 0;
      q_list[tail].x = g.x;
      q_list[tail].y = g.y;
      tail++;
    }
  }
  int pt1;
  int b;
  int X = 0, Y = 0;
  int i, j;
  while (head != tail) {
    Y = q_list[head].y;
    X = q_list[head].x;
    head++;
    pt1 = get_dist_val(X, Y) + 1;
    for (const auto D : direction_list) {
      i = 0;
      j = 0;
      if (D == Direction::North)
        j = 1;
      else if (D == Direction::East)
        i = 1;
      else if (D == Direction::West)
        i = -1;
      else if (D == Direction::South)
        j = -1;

      b = false;
      if (mode == 1)
        b = isProceed(X, Y, D);
      else
        b = !existWall(X, Y, D);

      if (b && get_dist_val(X + i, Y + j) == max_step_val) {
        if (X + i < 0 || X + i >= maze_size || Y + j < 0 ||
            Y + j >= maze_size) {
        } else {
          set_dist_val(X + i, Y + j, pt1);
          q_list[tail].x = X + i;
          q_list[tail].y = Y + j;
          tail++;
        }
      }
    }
    if (search_mode) {
      if (ego->x == X && ego->y == Y) {
        break;
      }
    }
  }
}

void MazeSolverBaseLgc::set_dist_val(const int x, const int y, const int val) {
  if (valid_map_list_idx(x, y))
    dist[x + y * maze_size] = val;
}

int MazeSolverBaseLgc::get_dist_val(int x, int y) {
  if (valid_map_list_idx(x, y))
    return dist[x + y * maze_size];
  else
    return max_step_val;
}

float MazeSolverBaseLgc::get_diadist_n_val(const int x, const int y) {
  if (valid_map_list_idx(x, y))
    return vector_dist[x + y * maze_size].n;
  else
    return vector_max_step_val;
}

float MazeSolverBaseLgc::get_diadist_e_val(const int x, const int y) {
  if (valid_map_list_idx(x, y))
    return vector_dist[x + y * maze_size].e;
  else
    return vector_max_step_val;
}

int MazeSolverBaseLgc::get_map_val(const int x, const int y) {
  if (valid_map_list_idx(x, y))
    return map[x + y * maze_size];
  else
    return 0xff;
}

bool MazeSolverBaseLgc::isProceed(const int x, const int y, Direction dir) {
  if (valid_map_list_idx(x, y))
    return ((get_map_val(x, y) / static_cast<int>(dir)) & 0x11) == 0x10;
  else
    return false;
}

bool MazeSolverBaseLgc::existWall(const int x, const int y, Direction dir) {
  if (valid_map_list_idx(x, y))
    return ((get_map_val(x, y) / static_cast<int>(dir)) & 0x01) == 0x01;
  else
    return true;
}

void MazeSolverBaseLgc::set_map_val(const int x, const int y, const int val) {
  if (valid_map_list_idx(x, y))
    map[x + y * maze_size] = val;
}
void MazeSolverBaseLgc::set_map_val(int idx, int val) { map[idx] = val; }

void MazeSolverBaseLgc::set_wall_data(const int x, const int y, Direction dir,
                                      const bool isWall) {
  if (valid_map_list_idx(x, y)) {
    int idx = x + y * maze_size;
    map[idx] |= (0x10 * static_cast<int>(dir));
    if (isWall)
      map[idx] |= 0x01 * static_cast<int>(dir);
    else
      map[idx] = (map[idx] & 0xf0) |
                 (map[idx] & (~(0x01 * static_cast<int>(dir)) & 0x0f));
  }
}
void MazeSolverBaseLgc::set_native_wall_data(const int idx,
                                             const uint8_t data) {
  if (idx >= maze_list_size)
    return;
  map[idx] = data;
}

void MazeSolverBaseLgc::set_default_wall_data() {
  for (int i = 0; i < maze_size; i++) {
    // set_map
    set_wall_data(i, maze_size - 1, Direction::North, true);
    set_wall_data(maze_size - 1, i, Direction::East, true);
    set_wall_data(0, i, Direction::West, true);
    set_wall_data(i, 0, Direction::South, true);
  }
  set_wall_data(0, 0, Direction::East, true);
  set_wall_data(0, 0, Direction::North, false);
  set_wall_data(1, 0, Direction::West, true);
  set_wall_data(0, 1, Direction::South, false);
}

bool MazeSolverBaseLgc::isStep(const int x, const int y, Direction dir) {
  if (valid_map_list_idx(x, y))
    return ((map[x + y * maze_size] / static_cast<int>(dir)) & 0x10) == 0x10;
  return false;
}

void MazeSolverBaseLgc::back_home() {
  clear_goal();
  point_t p;
  p.x = p.y = 0;
  goal_list.emplace_back(p);
}

void MazeSolverBaseLgc::clear_goal() {
  goal_list.clear();
  goal_list.shrink_to_fit();
}

void MazeSolverBaseLgc::append_goal(const int x, const int y) {
  point_t p;
  p.x = p.y = 0;
  goal_list.emplace_back(p);
}

int MazeSolverBaseLgc::get_max_step_val() { return max_step_val; }

int MazeSolverBaseLgc::clear_vector_distmap() {
  int tail = 0;
  for (char i = 0; i < maze_size; i++) {
    for (char j = 0; j < maze_size; j++) {
      int idx = i + j * maze_size;
      vector_dist[idx].n = vector_max_step_val;
      vector_dist[idx].e = vector_max_step_val;
      vector_dist[idx].w = vector_max_step_val;
      vector_dist[idx].s = vector_max_step_val;
      vector_dist[idx].v = 0;
      vector_dist[idx].N1 = 0;
      vector_dist[idx].NE = 0;
      vector_dist[idx].E1 = 0;
      vector_dist[idx].SE = 0;
      vector_dist[idx].S1 = 0;
      vector_dist[idx].SW = 0;
      vector_dist[idx].W1 = 0;
      vector_dist[idx].NW = 0;
      vector_dist[idx].W1 = 0;
      vector_dist[idx].step = 0;
      updateMap[idx] = 0;
    }
  }
  while (!vq_list.empty()) {
    vq_list.pop();
  }
  for (const auto p : goal_list) {
    unsigned char x = p.x;
    unsigned char y = p.y;
    int idx = x + y * maze_size;
    if (!existWall(x, y, Direction::North)) {
      vector_dist[idx].n = 0;
      dir_pt.x = x;
      dir_pt.y = y;
      dir_pt.dir = Direction::North;
      dir_pt.dist2 = 0;
      vq_list.push(dir_pt);
      tail++;
      if (y < maze_size - 1)
        vector_dist[(x) + (y + 1) * maze_size].s = 0;
    }
    if (!existWall(x, y, Direction::East)) {
      vector_dist[idx].e = 0;
      dir_pt.x = x;
      dir_pt.y = y;
      dir_pt.dir = Direction::East;
      dir_pt.dist2 = 0;
      vq_list.push(dir_pt);
      tail++;
      if (x < maze_size - 1)
        vector_dist[(x + 1) + (y)*maze_size].w = 0;
    }
    if (!existWall(x, y, Direction::West)) {
      vector_dist[idx].w = 0;
      dir_pt.x = x;
      dir_pt.y = y;
      dir_pt.dir = Direction::West;
      dir_pt.dist2 = 0;
      vq_list.push(dir_pt);
      tail++;
      if (x > 0)
        vector_dist[(x - 1) + (y)*maze_size].e = 0;
    }
    if (!existWall(x, y, Direction::South)) {
      vector_dist[idx].s = 0;
      dir_pt.x = x;
      dir_pt.y = y;
      dir_pt.dir = Direction::South;
      dir_pt.dist2 = 0;
      vq_list.push(dir_pt);
      tail++;
      if (y > 0)
        vector_dist[(x) + (y - 1) * maze_size].n = 0;
    }
  }

  return tail;
}

int MazeSolverBaseLgc::clear_vector_distmap(
    unordered_map<unsigned int, unsigned char> &subgoal_list) {
  int tail = 0;
  for (char i = 0; i < maze_size; i++) {
    for (char j = 0; j < maze_size; j++) {
      int idx = i + j * maze_size;
      if (vector_dist[idx].n == vector_max_step_val &&
          vector_dist[idx].e == vector_max_step_val &&
          vector_dist[idx].w == vector_max_step_val &&
          vector_dist[idx].s == vector_max_step_val)
        subgoal_list.erase(idx);
      else if (subgoal_list.count(idx) > 0) {
        subgoal_list[idx]++;
        if (maze_size > 20) {
          if (subgoal_list[idx] > 45)
            subgoal_list.erase(idx);
        } else {
          if (subgoal_list[idx] > 25)
            subgoal_list.erase(idx);
        }
      }

      vector_dist[idx].n = vector_max_step_val;
      vector_dist[idx].e = vector_max_step_val;
      vector_dist[idx].w = vector_max_step_val;
      vector_dist[idx].s = vector_max_step_val;
      vector_dist[idx].v = 0;
      vector_dist[idx].N1 = 0;
      vector_dist[idx].NE = 0;
      vector_dist[idx].E1 = 0;
      vector_dist[idx].SE = 0;
      vector_dist[idx].S1 = 0;
      vector_dist[idx].SW = 0;
      vector_dist[idx].W1 = 0;
      vector_dist[idx].NW = 0;
      vector_dist[idx].W1 = 0;
      vector_dist[idx].step = 0;
      updateMap[idx] = 0;
    }
  }
  while (!vq_list.empty()) {
    vq_list.pop();
  }

  for (const auto p : goal_list) {
    unsigned char x = p.x;
    unsigned char y = p.y;
    int idx = x + y * maze_size;
    if (!existWall(x, y, Direction::North)) {
      vector_dist[idx].n = 0;
      dir_pt.x = x;
      dir_pt.y = y;
      dir_pt.dir = Direction::North;
      dir_pt.dist2 = 0;
      vq_list.push(dir_pt);
      tail++;
      if (y < maze_size - 1)
        vector_dist[(x) + (y + 1) * maze_size].s = 0;
    }
    if (!existWall(x, y, Direction::East)) {
      vector_dist[idx].e = 0;
      dir_pt.x = x;
      dir_pt.y = y;
      dir_pt.dir = Direction::East;
      dir_pt.dist2 = 0;
      vq_list.push(dir_pt);
      tail++;
      if (x < maze_size - 1)
        vector_dist[(x + 1) + (y)*maze_size].w = 0;
    }
    if (!existWall(x, y, Direction::West)) {
      vector_dist[idx].w = 0;
      dir_pt.x = x;
      dir_pt.y = y;
      dir_pt.dir = Direction::West;
      dir_pt.dist2 = 0;
      vq_list.push(dir_pt);
      tail++;
      if (x > 0)
        vector_dist[(x - 1) + (y)*maze_size].e = 0;
    }
    if (!existWall(x, y, Direction::South)) {
      vector_dist[idx].s = 0;
      dir_pt.x = x;
      dir_pt.y = y;
      dir_pt.dir = Direction::South;
      dir_pt.dist2 = 0;
      vq_list.push(dir_pt);
      tail++;
      if (y > 0)
        vector_dist[(x) + (y - 1) * maze_size].n = 0;
    }
  }

  return tail;
}

int MazeSolverBaseLgc::haveVectorLv(const int x, const int y, Direction dir) {
  const int idx = x + y * maze_size;
  if (dir == Direction::North) {
    if (vector_dist[idx].N1 > borderLv2)
      return 2;
    else if (vector_dist[idx].N1 > borderLv1)
      return 1;
  } else if (dir == Direction::NorthEast) {
    if (vector_dist[idx].NE > borderLv2d)
      return 2;
    else if (vector_dist[idx].NE > borderLv1d)
      return 1;
  } else if (dir == Direction::East) {
    if (vector_dist[idx].E1 > borderLv2)
      return 2;
    else if (vector_dist[idx].E1 > borderLv1)
      return 1;
  } else if (dir == Direction::SouthEast) {
    if (vector_dist[idx].SE > borderLv2d)
      return 2;
    else if (vector_dist[idx].SE > borderLv1d)
      return 1;
  } else if (dir == Direction::South) {
    if (vector_dist[idx].S1 > borderLv2)
      return 2;
    else if (vector_dist[idx].S1 > borderLv1)
      return 1;
  } else if (dir == Direction::SouthWest) {
    if (vector_dist[idx].SW > borderLv2d)
      return 2;
    else if (vector_dist[idx].SW > borderLv1d)
      return 1;
  } else if (dir == Direction::West) {
    if (vector_dist[idx].W1 > borderLv2)
      return 2;
    else if (vector_dist[idx].W1 > borderLv1)
      return 1;
  } else if (dir == Direction::NorthWest) {
    if (vector_dist[idx].NW > borderLv2d)
      return 2;
    else if (vector_dist[idx].NW > borderLv1d)
      return 1;
  }
  return 0;
}

float MazeSolverBaseLgc::getDistV(const int x, const int y, Direction dir) {
  if (valid_map_list_idx(x, y)) {
    if (dir == Direction::North)
      return vector_dist[x + y * maze_size].n;
    else if (dir == Direction::East)
      return vector_dist[x + y * maze_size].e;
    else if (dir == Direction::West)
      return vector_dist[x + y * maze_size].w;
    else if (dir == Direction::South)
      return vector_dist[x + y * maze_size].s;
  }
  return vector_max_step_val;
}

void MazeSolverBaseLgc::setDistV(const int x, const int y, Direction dir,
                                 const float val) {
  if (valid_map_list_idx(x, y)) {
    const int idx = x + y * maze_size;
    if (dir == Direction::North) {
      vector_dist[idx].n = val;
      if (y < maze_size - 1)
        vector_dist[(x) + (y + 1) * maze_size].s = val;
    } else if (dir == Direction::East) {
      vector_dist[idx].e = val;
      if (x < maze_size - 1)
        vector_dist[(x + 1) + (y)*maze_size].w = val;
    } else if (dir == Direction::West) {
      vector_dist[idx].w = val;
      if (x > 0)
        vector_dist[(x - 1) + (y)*maze_size].e = val;
    } else if (dir == Direction::South) {
      vector_dist[idx].s = val;
      if (y > 0)
        vector_dist[(x) + (y - 1) * maze_size].n = val;
    }
  }
}

bool MazeSolverBaseLgc::isUpdated(const int x, const int y, Direction dir) {
  if (!valid_map_list_idx(x, y))
    return true;

  const int idx = x + y * maze_size;
  if (dir == Direction::North)
    return (updateMap[idx] & 0x01) == 0x01;
  else if (dir == Direction::East)
    return (updateMap[idx] & 0x02) == 0x02;
  else if (dir == Direction::West)
    return (updateMap[idx] & 0x04) == 0x04;
  else if (dir == Direction::South)
    return (updateMap[idx] & 0x08) == 0x08;
  else if (dir == Direction::NorthEast)
    return (updateMap[idx] & 0x10) == 0x10;
  else if (dir == Direction::SouthEast)
    return (updateMap[idx] & 0x20) == 0x20;
  else if (dir == Direction::SouthWest)
    return (updateMap[idx] & 0x40) == 0x40;
  else if (dir == Direction::NorthWest)
    return (updateMap[idx] & 0x80) == 0x80;

  return false;
}

void MazeSolverBaseLgc::addVector(const int x, const int y, Direction dir,
                                  float val) {
  if (valid_map_list_idx(x, y)) {
    if (val < 15)
      val++;
    const int idx = x + y * maze_size;
    if (dir == Direction::North)
      vector_dist[idx].N1 = val;
    else if (dir == Direction::NorthEast)
      vector_dist[idx].NE = val;
    else if (dir == Direction::East)
      vector_dist[idx].E1 = val;
    else if (dir == Direction::SouthEast)
      vector_dist[idx].SE = val;
    else if (dir == Direction::South)
      vector_dist[idx].S1 = val;
    else if (dir == Direction::SouthWest)
      vector_dist[idx].SW = val;
    else if (dir == Direction::West)
      vector_dist[idx].W1 = val;
    else if (dir == Direction::NorthWest)
      vector_dist[idx].NW = val;
  }
}

void MazeSolverBaseLgc::updateMapCheck(const int x, const int y,
                                       Direction dir) {
  if (valid_map_list_idx(x, y)) {
    const int idx = x + y * maze_size;
    if (dir == Direction::North)
      updateMap[idx] |= 0x01;
    else if (dir == Direction::East)
      updateMap[idx] |= 0x02;
    else if (dir == Direction::West)
      updateMap[idx] |= 0x04;
    else if (dir == Direction::South)
      updateMap[idx] |= 0x08;
    else if (dir == Direction::NorthEast)
      updateMap[idx] |= 0x10;
    else if (dir == Direction::SouthEast)
      updateMap[idx] |= 0x20;
    else if (dir == Direction::SouthWest)
      updateMap[idx] |= 0x40;
    else if (dir == Direction::NorthWest)
      updateMap[idx] |= 0x80;
  }
}

int MazeSolverBaseLgc::getVector(const int x, const int y, Direction dir) {
  if (valid_map_list_idx(x, y)) {
    const int idx = x + y * maze_size;
    if (dir == Direction::North)
      return vector_dist[idx].N1;
    else if (dir == Direction::NorthEast)
      return vector_dist[idx].NE;
    else if (dir == Direction::East)
      return vector_dist[idx].E1;
    else if (dir == Direction::SouthEast)
      return vector_dist[idx].SE;
    else if (dir == Direction::South)
      return vector_dist[idx].S1;
    else if (dir == Direction::SouthWest)
      return vector_dist[idx].SW;
    else if (dir == Direction::West)
      return vector_dist[idx].W1;
    else if (dir == Direction::NorthWest)
      return vector_dist[idx].NW;
  }
  return 0;
}

bool MazeSolverBaseLgc::is_unknown(const int x, const int y, Direction dir) {
  if (valid_map_list_idx(x, y)) {
    // return (map[x + y * maze_size] & 0xf0) != 0xf0;
    if (dir == Direction::North)
      return (map[x + y * maze_size] & 0x10) == 0x00;
    else if (dir == Direction::East)
      return (map[x + y * maze_size] & 0x20) == 0x00;
    else if (dir == Direction::West)
      return (map[x + y * maze_size] & 0x40) == 0x00;
    else if (dir == Direction::South)
      return (map[x + y * maze_size] & 0x80) == 0x00;
  }
  return false;
}

void MazeSolverBaseLgc::simplesort(const int tail) {}

unsigned int MazeSolverBaseLgc::updateVectorMap(const bool isSearch) {
  unsigned int head = 0;
  unsigned int tail = clear_vector_distmap();
  unsigned int c = 0;

  while (!vq_list.empty()) {
    const auto now_pos = vq_list.top();
    vq_list.pop();
    int X = now_pos.x;
    int Y = now_pos.y;
    Direction dir = now_pos.dir;
    int i = 0;
    int j = 0;
    Direction d[3] = {Direction::North, Direction::North, Direction::North};
    Direction d2[3] = {Direction::North, Direction::North, Direction::North};
    float now = getDistV(X, Y, dir);
    if (dir == Direction::North) {
      j = 1;
      d[0] = Direction::North;     // N
      d[1] = Direction::NorthEast; // NE
      d[2] = Direction::NorthWest; // NW
      d2[0] = Direction::North;    // N
      d2[1] = Direction::East;     // E
      d2[2] = Direction::West;     // W
    } else if (dir == Direction::East) {
      i = 1;
      d[0] = Direction::East;      // E
      d[1] = Direction::SouthEast; // SE
      d[2] = Direction::NorthEast; // NE
      d2[0] = Direction::East;     // E
      d2[1] = Direction::South;    // S
      d2[2] = Direction::North;    // N
    } else if (dir == Direction::West) {
      i = -1;
      d[0] = Direction::West;      // W
      d[1] = Direction::NorthWest; // NW
      d[2] = Direction::SouthWest; // SW
      d2[0] = Direction::West;     // W
      d2[1] = Direction::North;    // N
      d2[2] = Direction::South;    // S
    } else if (dir == Direction::South) {
      j = -1;
      d[0] = Direction::South;     // S
      d[1] = Direction::SouthWest; // SW
      d[2] = Direction::SouthEast; // SE
      d2[0] = Direction::South;    // S
      d2[1] = Direction::West;     // W
      d2[2] = Direction::East;     // E
    }
    // c++;
    for (int k = 0; k < 3; k++) {
      c++;

      if (!existWall(X + i, Y + j, d2[k]) &&
          (isSearch || isStep(X + i, Y + j, d2[k]))) {
        int v = haveVectorLv(X, Y, d[k]);
        float tmp = now;
        if (dir == d2[k]) {
          if (v >= 2) {
            tmp += St3;
          } else if (v == 1) {
            tmp += St2;
          } else {
            tmp += St1;
          }
          if (tmp <= getDistV(X + i, Y + j, d2[k])) {
            if (!isUpdated(X + i, Y + j, d2[k])) {
              setDistV(X + i, Y + j, d2[k], tmp);
              dir_pt.x = (X + i);
              dir_pt.y = (Y + j);
              dir_pt.dir = d2[k];
              dir_pt.dist2 = tmp;
              vq_list.push(dir_pt);
              simplesort(tail);
              tail++;
              updateMapCheck(X + i, Y + j, d2[k]);
            }
          }
          addVector(X + i, Y + j, d[k], getVector(X, Y, d[k]));
        } else {
          if (v == 2) {
            tmp += Dia3;
          } else if (v == 1) {
            tmp += Dia2;
          } else {
            tmp += Dia;
          }
          if (tmp <= getDistV(X + i, Y + j, d2[k])) {
            if (!isUpdated(X + i, Y + j, d2[k])) {
              setDistV(X + i, Y + j, d2[k], tmp);
              dir_pt.x = (X + i);
              dir_pt.y = (Y + j);
              dir_pt.dir = d2[k];
              dir_pt.dist2 = tmp;
              vq_list.push(dir_pt);
              simplesort(tail);
              tail++;
              updateMapCheck(X + i, Y + j, d2[k]);
            }
          }
          addVector(X + i, Y + j, d[k], getVector(X, Y, d[k]));
        }
      }
    }
    head++;
  }
  return c;
}

unsigned int MazeSolverBaseLgc::updateVectorMap(
    const bool isSearch,
    unordered_map<unsigned int, unsigned char> &subgoal_list) {
  unsigned int head = 0;
  unsigned int tail = clear_vector_distmap(subgoal_list);
  unsigned int c = 0;

  while (!vq_list.empty()) {
    const auto now_pos = vq_list.top();
    vq_list.pop();
    int X = now_pos.x;
    int Y = now_pos.y;
    Direction dir = now_pos.dir;

    if ((map[X + Y * maze_size] & 0xf0) == 0xf0) {
      subgoal_list.erase(X + Y * maze_size);
    }

    int i = 0;
    int j = 0;
    Direction d[3] = {Direction::North, Direction::North, Direction::North};
    Direction d2[3] = {Direction::North, Direction::North, Direction::North};
    float now = getDistV(X, Y, dir);
    if (dir == Direction::North) {
      j = 1;
      d[0] = Direction::North;     // N
      d[1] = Direction::NorthEast; // NE
      d[2] = Direction::NorthWest; // NW
      d2[0] = Direction::North;    // N
      d2[1] = Direction::East;     // E
      d2[2] = Direction::West;     // W
    } else if (dir == Direction::East) {
      i = 1;
      d[0] = Direction::East;      // E
      d[1] = Direction::SouthEast; // SE
      d[2] = Direction::NorthEast; // NE
      d2[0] = Direction::East;     // E
      d2[1] = Direction::South;    // S
      d2[2] = Direction::North;    // N
    } else if (dir == Direction::West) {
      i = -1;
      d[0] = Direction::West;      // W
      d[1] = Direction::NorthWest; // NW
      d[2] = Direction::SouthWest; // SW
      d2[0] = Direction::West;     // W
      d2[1] = Direction::North;    // N
      d2[2] = Direction::South;    // S
    } else if (dir == Direction::South) {
      j = -1;
      d[0] = Direction::South;     // S
      d[1] = Direction::SouthWest; // SW
      d[2] = Direction::SouthEast; // SE
      d2[0] = Direction::South;    // S
      d2[1] = Direction::West;     // W
      d2[2] = Direction::East;     // E
    }
    // c++;
    for (int k = 0; k < 3; k++) {
      c++;

      if (!existWall(X + i, Y + j, d2[k]) &&
          (isSearch || isStep(X + i, Y + j, d2[k]))) {
        int v = haveVectorLv(X, Y, d[k]);
        float tmp = now;
        if (dir == d2[k]) {
          if (v >= 2) {
            tmp += St3;
          } else if (v == 1) {
            tmp += St2;
          } else {
            tmp += St1;
          }
          if (tmp <= getDistV(X + i, Y + j, d2[k])) {
            if (!isUpdated(X + i, Y + j, d2[k])) {
              setDistV(X + i, Y + j, d2[k], tmp);
              dir_pt.x = (X + i);
              dir_pt.y = (Y + j);
              dir_pt.dir = d2[k];
              dir_pt.dist2 = tmp;
              vq_list.push(dir_pt);
              simplesort(tail);
              tail++;
              updateMapCheck(X + i, Y + j, d2[k]);
            }
          }
          addVector(X + i, Y + j, d[k], getVector(X, Y, d[k]));
        } else {
          if (v == 2) {
            tmp += Dia3;
          } else if (v == 1) {
            tmp += Dia2;
          } else {
            tmp += Dia;
          }
          if (tmp <= getDistV(X + i, Y + j, d2[k])) {
            if (!isUpdated(X + i, Y + j, d2[k])) {
              setDistV(X + i, Y + j, d2[k], tmp);
              dir_pt.x = (X + i);
              dir_pt.y = (Y + j);
              dir_pt.dir = d2[k];
              dir_pt.dist2 = tmp;
              vq_list.push(dir_pt);
              simplesort(tail);
              tail++;
              updateMapCheck(X + i, Y + j, d2[k]);
            }
          }
          addVector(X + i, Y + j, d[k], getVector(X, Y, d[k]));
        }
      }
    }
    head++;
  }
  return c;
}
void MazeSolverBaseLgc::step_cell(int x, int y, Direction d) {
  if (valid_map_list_idx(x, y)) {
    if (d == Direction::North)
      map[x + y * maze_size] |= 0x10;
    else if (d == Direction::East)
      map[x + y * maze_size] |= 0x20;
    else if (d == Direction::West)
      map[x + y * maze_size] |= 0x40;
    else if (d == Direction::South)
      map[x + y * maze_size] |= 0x80;
  }
}

bool MazeSolverBaseLgc::is_stepped(int x, int y) {
  if (valid_map_list_idx(x, y))
    return ((map[x + y * maze_size]) & 0xf0) == 0xf0;
  return false;
}
bool MazeSolverBaseLgc::is_front_cell_stepped(int x, int y, Direction dir) {
  if (valid_map_list_idx(x, y)) {
    if (dir == Direction::North) {
      if (valid_map_list_idx(x, y + 1)) {
        return ((map[x + (y + 1) * maze_size]) & 0xf0) == 0xf0;
      }
    } else if (dir == Direction::East) {
      if (valid_map_list_idx(x + 1, y)) {
        return ((map[(x + 1) + y * maze_size]) & 0xf0) == 0xf0;
      }
    } else if (dir == Direction::West) {
      if (valid_map_list_idx(x - 1, y)) {
        return ((map[(x - 1) + y * maze_size]) & 0xf0) == 0xf0;
      }
    } else if (dir == Direction::South) {
      if (valid_map_list_idx(x, y - 1)) {
        return ((map[x + (y - 1) * maze_size]) & 0xf0) == 0xf0;
      }
    }
  }
  return false;
}

float MazeSolverBaseLgc::getDistVector(const int x, const int y,
                                       Direction dir) {
  if (valid_map_list_idx(x, y)) {
    if (existWall(x, y, dir))
      return VectorMax;
    if (dir == Direction::North)
      return vector_dist[x + y * maze_size].n;
    else if (dir == Direction::East)
      return vector_dist[x + y * maze_size].e;
    else if (dir == Direction::West)
      return vector_dist[x + y * maze_size].w;
    else if (dir == Direction::South)
      return vector_dist[x + y * maze_size].s;
  }
  return VectorMax;
}

void MazeSolverBaseLgc::setNextRootDirectionPathUnKnown(
    int x, int y, Direction dir, Direction now_dir, Direction &nextDirection,
    float &Value) {
  const bool isWall = existWall(x, y, dir);
  // const bool step = isStep(x, y, dir);
  const float dist = isWall ? vector_max_step_val : getDistVector(x, y, dir);
  if (static_cast<int>(now_dir) * static_cast<int>(dir) == 8)
    return;
  if (!isWall && dist < Value) {
    nextDirection = dir;
    Value = dist;
  }
}

bool MazeSolverBaseLgc::arrival_goal_position(const int x, const int y) {
  for (const auto p : goal_list)
    if (p.x == x && p.y == y)
      return true;
  return false;
}

unsigned int MazeSolverBaseLgc::searchGoalPosition(
    const bool isSearch,
    unordered_map<unsigned int, unsigned char> &subgoal_list) {
  Direction next_dir = Direction::North;
  Direction now_dir = Direction::North;
  int x = 0;
  int y = 1;
  // int position = 0;
  // int idx;

  Direction dirLog[3] = {now_dir, now_dir, now_dir};
  point_t pt;
  pt.x = 0;
  pt.y = 0;
  // search_log.clear();
  // search_log.shrink_to_fit();

  unsigned int cnt = updateVectorMap(isSearch, subgoal_list);
  while (true) {
    now_dir = next_dir;
    dirLog[2] = dirLog[1];
    dirLog[1] = dirLog[0];
    dirLog[0] = now_dir;
    Value = vector_max_step_val;
    next_dir = Direction::Undefined;
    pt.x = x;
    pt.y = y;
    // search_log.emplace_back(pt);

    if (arrival_goal_position(x, y))
      break;

    // const unsigned int position = getDistVector(x, y, now_dir);
    float position = getDistVector(x, y, now_dir);

    if (now_dir == Direction::North) {
      position = getDistVector(x, y, Direction::South);
    } else if (now_dir == Direction::East) {
      position = getDistVector(x, y, Direction::West);
    } else if (now_dir == Direction::West) {
      position = getDistVector(x, y, Direction::East);
    } else if (now_dir == Direction::South) {
      position = getDistVector(x, y, Direction::North);
    }
    setNextRootDirectionPathUnKnown(x, y, Direction::North, now_dir, next_dir,
                                    position);
    setNextRootDirectionPathUnKnown(x, y, Direction::East, now_dir, next_dir,
                                    position);
    setNextRootDirectionPathUnKnown(x, y, Direction::West, now_dir, next_dir,
                                    position);
    setNextRootDirectionPathUnKnown(x, y, Direction::South, now_dir, next_dir,
                                    position);

    if (dirLog[0] == dirLog[1] || dirLog[0] != dirLog[2])
      priorityStraight2(x, y, now_dir, dirLog[0], position, next_dir);
    else
      priorityStraight2(x, y, now_dir, dirLog[1], position, next_dir);

    if (next_dir == Direction::North) {
      if (is_unknown(x, y, Direction::North))
        subgoal_list[x + (y + 1) * maze_size] = 1;
    } else if (next_dir == Direction::East) {
      if (is_unknown(x, y, Direction::East))
        subgoal_list[x + 1 + y * maze_size] = 1;
    } else if (next_dir == Direction::West) {
      if (is_unknown(x, y, Direction::West))
        subgoal_list[x - 1 + y * maze_size] = 1;
    } else if (next_dir == Direction::South) {
      if (is_unknown(x, y, Direction::South))
        subgoal_list[x + (y - 1) * maze_size] = 1;
    }
    // if (next_dir == Direction::North) {
    //   if (!is_stepped(x, y + 1))
    //     subgoal_list[x + (y + 1) * maze_size] = 1;
    // } else if (next_dir == Direction::East) {
    //   if (!is_stepped(x + 1, y))
    //     subgoal_list[x + 1 + y * maze_size] = 1;
    // } else if (next_dir == Direction::West) {
    //   if (!is_stepped(x - 1, y))
    //     subgoal_list[x - 1 + y * maze_size] = 1;
    // } else if (next_dir == Direction::South) {
    //   if (!is_stepped(x, y - 1))
    //     subgoal_list[x + (y - 1) * maze_size] = 1;
    // }

    if (next_dir == Direction::North)
      y++;
    else if (next_dir == Direction::East)
      x++;
    else if (next_dir == Direction::West)
      x--;
    else if (next_dir == Direction::South)
      y--;

    if (next_dir == Direction::Undefined)
      break;
  }
  // pt_list.erase(std::unique(pt_list.begin(), pt_list.end()), pt_list.end());

  return cnt;
}
void MazeSolverBaseLgc::priorityStraight2(int x, int y, Direction now_dir,
                                          Direction dir, float &dist_val,
                                          Direction &next_dir) {
  const bool isWall = existWall(x, y, dir);
  const bool step = isStep(x, y, dir);
  const float dist = isWall ? vector_max_step_val : getDistVector(x, y, dir);
  if (static_cast<int>(now_dir) * static_cast<int>(dir) == 8)
    return;
  if (!isWall && step && dist <= dist_val) {
    next_dir = dir;
    dist_val = dist;
  }
}
MazeSolverBaseLgc::MazeSolverBaseLgc(/* args */) {}

MazeSolverBaseLgc::~MazeSolverBaseLgc() {}