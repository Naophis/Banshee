#ifndef TRAJECTORY_CREATOR_H
#define TRAJECTORY_CREATOR_H

#include "maze_solver.hpp"
#include "stdio.h"
#include <cmath>
#include <vector>

using namespace std;

class TrajectoryCreator {
private:
  float get_run_dir(Direction dir, float ang);
  float get_turn_tgt_ang(TurnType type);
  float get_turn_rad(TurnType type);
  float get_front_dist(TurnType type, bool dia);
  float get_back_dist(TurnType type, bool dia);
  float get_slalom_etn(TurnType type, bool dia);
  float get_slalom_time(TurnType type, bool dia);

  void fix_pos(ego_odom_t &ego, TurnType type, TurnDirection turn_dir,
               trajectory_point_t &trj_ele);
  float Et2(float t, float s, float N);
  void run_straight(vector<trajectory_point_t> &trajectory, float dist,
                    trajectory_point_t &trj_ele, ego_odom_t &ego,
                    run_param_t &param, char type);
  void slalom(vector<trajectory_point_t> &trajectory, int turn_num,
              trajectory_point_t &trj_ele, ego_odom_t &ego, run_param_t &param,
              bool dia);
  float cell_size = 180;
  float half_cell_size = 90;
  float get_dist_pt(trajectory_point_t &from, trajectory_point_t &to);
  void get_next_from_pt(trajectory_point_t &from, trajectory_point_t &to);
  void get_next_from_pt_x(trajectory_point_t &from, trajectory_point_t &to);
  void get_next_from_pt_y(trajectory_point_t &from, trajectory_point_t &to);

  void get_next_from_theta_x(trajectory_point_t &from, trajectory_point_t &to,
                             float next_x);
  void get_next_from_theta_y(trajectory_point_t &from, trajectory_point_t &to,
                             float next_y);

  base_trajectory_pattern_t zipped_trj;

  void slalom2(vector<trajectory_point_t> &trajectory, int turn_num,
               trajectory_point_t &trj_ele, ego_odom_t &ego, run_param_t &param,
               bool dia);

  void repaint_slalom(vector<trajectory_point_t> &trajectory,
                      vector<trajectory_point_t> &base_trj,
                      trajectory_point_t &trj_ele, ego_odom_t &ego,
                      run_param_t &param, TurnDirection turn_dir,
                      TurnType type);

public:
  TrajectoryCreator(/* args */);
  ~TrajectoryCreator();
  void init();
  void exec(path_struct &base_path, vector<trajectory_point_t> &trajectory);
  void exec2(path_struct &base_path, vector<trajectory_point_t> &trajectory);
  void exec3(path_struct &base_path, vector<trajectory_point_t> &trajectory);
  void make_chopped_trajectory(vector<trajectory_point_t> &trajectory,
                               int turn_num, bool dia);

  Direction get_next_dir(Direction dir, TurnType type, TurnDirection turn_dir);
  TurnType get_turn_type(int turn_num);
  TurnType get_turn_type(int turn_num, bool dia);

  TurnDirection get_turn_dir(int turn_dir);
  double chop_dt = 3;
  slalom_data_t sla_data;
};

#endif