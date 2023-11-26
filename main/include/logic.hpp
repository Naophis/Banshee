#ifndef MAZE_SOLVER_BASE_LGC_H
#define MAZE_SOLVER_BASE_LGC_H

#include "include/defines.hpp"
#include "stdio.h"
#include <vector>

#include "maze_solver.hpp"
#include <unordered_map>

#include <queue>
using namespace std;

class MazeSolverBaseLgc {
public:
  MazeSolverBaseLgc();

  ~MazeSolverBaseLgc();

  void reset_dist_map();
  bool reset_done = true;

  void data_economize();

  void step_cell(int x, int y, Direction d);

  void set_map_val(int idx, int val);

  void init(const int _maze_size, const int _max_step_val);

  void set_goal_pos(const vector<point_t> &v);

  void update_dist_map(const int mode, const bool search_mode);

  int get_dist_val(int x, int y);

  float get_diadist_n_val(const int x, const int y);

  float get_diadist_e_val(const int x, const int y);

  int get_map_val(const int x, const int y);

  void set_dist_val(const int x, const int y, const int val);

  void set_map_val(const int x, const int y, const int val);

  bool isProceed(const int x, const int y, Direction dir);

  bool existWall(const int x, const int y, Direction dir);

  void set_wall_data(const int x, const int y, Direction dir,
                     const bool isWall);

  void set_native_wall_data(const int idx, const uint8_t data);

  void set_default_wall_data();

  bool valid_map_list_idx(const int x, const int y);

  bool isStep(const int x, const int y, Direction dir);

  bool is_stepped(int x, int y);
  bool is_front_cell_stepped(int x, int y, Direction now_dir);
  void back_home();

  int get_max_step_val();

  unsigned int updateVectorMap(const bool isSearch);
  unsigned int
  updateVectorMap(const bool isSearch,
                  unordered_map<unsigned int, unsigned char> &subgoal_list);
  unsigned int
  searchGoalPosition(const bool isSearch,
                     unordered_map<unsigned int, unsigned char> &subgoal_list);

  bool candidate_end(const int x, const int y);

  int clear_vector_distmap();
  int clear_vector_distmap(
      unordered_map<unsigned int, unsigned char> &subgoal_list);

  int haveVectorLv(const int x, const int y, Direction dir);

  float getDistV(const int x, const int y, Direction dir);

  void setDistV(const int x, const int y, Direction dir, const float val);

  void addVector(const int x, const int y, Direction dir, float val);

  bool isUpdated(const int x, const int y, Direction dir);

  int getVector(const int x, const int y, Direction dir);

  void updateMapCheck(const int x, const int y, Direction dir);

  void simplesort(const int tail);

  void clear_goal();

  void append_goal(const int x, const int y);

  unsigned int searchGoalPosition(const bool isSearch,
                                  vector<point_t> &pt_list);

  bool is_unknown(const int x, const int y, Direction dir);

  void setNextRootDirectionPathUnKnown(int x, int y, Direction dir,
                                       Direction now_dir,
                                       Direction &nextDirection, float &Value);

  float getDistVector(const int x, const int y, Direction dir);

  bool arrival_goal_position(const int x, const int y);

  void set_goal_pos2(const vector<point_t> &pt_list);

  void updateWall(int x, int y, Direction dir);

  void remove_goal_pos3();

  unsigned int maze_size;
  unsigned int max_step_val;

  vector<vector_map_t> vector_dist;
  vector<point_t> search_log;

  vector<point_t> goal_list;
  vector<point_t> goal_list2;
  vector<point_t> goal_list3;
  vector<point_t> goal_list_origin;

  void priorityStraight2(int x, int y, Direction now_dir, Direction dir,
                         float &dist_val, Direction &next_dir);

  vector<unsigned char> map;
  vector<unsigned int> dist;
  vector<unsigned char> updateMap;
  vector<point_t> q_list;
  // vector<dir_pt_t> vq_list;
  priority_queue<dir_pt_t, vector<dir_pt_t>, CompairDirPt> vq_list;
  void set_ego(std::shared_ptr<ego_t> &_ego);
  unsigned int vector_max_step_val = 180 * 1024;
  unsigned int VectorMax = vector_max_step_val;
  const float VectorMaxF = (float)vector_max_step_val;
  int param_num = 1;
  void set_param_num(int type) { param_num = type; }

  void set_param() {
    if (param_num == 1) {
      set_param3();
    } else if (param_num == 2) {
      set_param2();
    } else if (param_num == 3) {
      set_param1();
    } else if (param_num == 4) {
      set_param4();
    } else if (param_num == 5) {
      set_param5();
    } else {
      set_param3(); // default
    }
  }

private:
  std::shared_ptr<ego_t> ego;
  unsigned int maze_list_size;
  unsigned int goal_list_size;
  int Value = 0;
  unsigned int borderLv1 = 0;
  unsigned int borderLv2 = 2;
  unsigned int borderLv1d = 0;
  unsigned int borderLv2d = 2;
  unsigned int MAX_VALUE = 4095;

  unsigned int bias = 1;
  unsigned int strBias = 3;
  unsigned int diaBias = 3;

  unsigned int minus = 4;

  // unsigned int Dia = 5;
  // unsigned int Dia2 = 3;
  // unsigned int Dia3 = 2;

  // unsigned int St1 = 7;
  // unsigned int St2 = 2;
  // unsigned int St3 = 1;

  float cell_size = 7;

  float Dia = cell_size * 1.41421356 / 2;
  float Dia2 = cell_size * 1.41421356 / 2 * 3 / 5;
  float Dia3 = cell_size * 1.41421356 / 2 * 2 / 5;

  float St1 = cell_size;
  float St2 = cell_size * 2 / 7;
  float St3 = cell_size * 1 / 7;
  dir_pt_t dir_pt;

  void set_param1() {
    cell_size = 180;
    Dia = cell_size * 1.41421356 / 2;
    Dia2 = cell_size * 1.41421356 / 2;
    Dia3 = cell_size * 1.41421356 / 2;
    St1 = cell_size;
    St2 = cell_size;
    St3 = cell_size;
  }
  void set_param2() {
    cell_size = 1;
    Dia = 1;
    Dia2 = 1;
    Dia3 = 1;
    St1 = 1;
    St2 = 1;
    St3 = 1;
  }

  void set_param3() {
    cell_size = 7;
    Dia = cell_size * 1.41421356 / 2;
    Dia2 = cell_size * 1.41421356 / 2 * 3 / 5;
    Dia3 = cell_size * 1.41421356 / 2 * 2 / 5;

    St1 = cell_size;
    St2 = cell_size * 2 / 7;
    St3 = cell_size * 1 / 7;
  }

  void set_param4() {
    cell_size = 7;
    Dia = cell_size * 1.41421356 / 2;
    Dia2 = cell_size * 1.41421356 / 2 * 3 / 5;
    Dia3 = cell_size * 1.41421356 / 2 * 2 / 5;

    St1 = cell_size;
    St2 = cell_size * 3 / 7;
    St3 = cell_size * 2 / 7;
  }

  void set_param5() {
    cell_size = 1;
    float tmp = 1;
    float tmp2 = 2;
    Dia = cell_size * 1.41421356 / tmp;
    Dia2 = cell_size * 1.41421356 / tmp;
    Dia3 = cell_size * 1.41421356 / tmp;
    St1 = cell_size / tmp2;
    St2 = cell_size / (tmp2 + 1);
    St3 = cell_size / (tmp2 + 3);
  }
};

#endif