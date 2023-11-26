#ifndef PATH_CREATOR_H
#define PATH_CREATOR_H

#include "include/defines.hpp"
#include "stdio.h"
#include <vector>

#include "include/ui.hpp"
#include "logic.hpp"
#include "maze_solver.hpp"
#include "trajectory_creator.hpp"

// constexpr int checkQlength = 256;
constexpr int R = 1;
constexpr int L = 2;
constexpr unsigned int vector_max_step_val = 180 * 1024;
constexpr float MAX = vector_max_step_val;
class PathCreator {
private:
  int get_dist_val(int x, int y);
  void updateVectorMap(const bool isSearch);

  void add_path_s(int idx, float val);
  void append_path_s(float val);
  void append_path_t(int val);
  void setNextRootDirectionPath(int x, int y, Direction now_dir, Direction dir,
                                float &val, Direction &next_dir);
  Direction get_next_pos(int &x, int &y, Direction dir,
                         Direction next_direction);

  Motion get_next_motion(Direction now_dir, Direction next_direction);

  void priorityStraight2(int x, int y, Direction now_dir, Direction dir,
                         float &dist_val, Direction &next_dir);

  void pathOffset();

  priority_queue<candidate_route_info_t, vector<candidate_route_info_t>,
                 CompairCandiRoute>
      route_q;
  priority_queue<route_t, vector<route_t>, CompairRoute> route_list;

  std::unordered_map<int, int> stepped;

  TrajectoryCreator tc;
  void checkOtherRoot(int x, int y, float now, Direction now_dir);
  void checkOtherRoot(int x, int y, Direction now_dir, float now_pos);

  param_straight_t ps;

public:
  std::unordered_map<int, candidate_route_info_t> other_route_map;
  std::unordered_map<int, candidate_route_info_t> other_route_map_bk;
  std::shared_ptr<MazeSolverBaseLgc> lgc;
  std::shared_ptr<UserInterface> ui;
  void set_logic(std::shared_ptr<MazeSolverBaseLgc> &_lgc);
  void set_userinterface(std::shared_ptr<UserInterface> &_ui);

  std::vector<float> path_s;
  std::vector<unsigned char> path_t;
  std::vector<float> path_time_s;
  std::vector<float> path_time_t;
  std::vector<float> path_time_total;
  vector<float> path_s2;
  vector<unsigned char> path_t2;

  struct CompairPath {
    bool operator()(path_set_t const &p1, path_set_t const &p2) {
      // 時間最短。同じならパスの長さが短い方が優先。
      if (p1.time > p2.time) {
        return true;
      } else if (p1.time == p2.time) {
        if (p1.path_t.size() > p2.path_t.size()) {
          return true;
        }
      }
      return false;
    };
  };
  std::priority_queue<path_set_t, vector<path_set_t>, CompairPath> path_set_map;

  int path_size;

  PathCreator(/* args */);
  ~PathCreator();
  bool path_create(bool is_search);
  bool path_create(bool is_search, int tgt_x, int tgt_y, Direction tgt_dir,
                   bool &use);
  bool path_create_with_change(bool is_search, int tgt_x, int tgt_y,
                               Direction tgt_dir,
                               path_create_status_t &pc_state,
                               param_set_t &p_set);
  void path_reflash();
  void convert_large_path(bool b1);
  void diagonalPath(bool isFull, bool a1);
  void print_path();
  void print_path2();

  float calc_goal_time(param_set_t &p_set);
  float timebase_path_create(bool is_search, param_set_t &p_set, path_set_t &p);
  path_create_status_t pc_result;
  route_t route;
  float go_straight_dummy(float v1, float vmax, float v2, float ac, float diac,
                          float dist);
  float slalom_dummy(TurnType turn_type, TurnDirection td, param_set_t &p_set);

  char asc(float d, float d2);
};

#endif