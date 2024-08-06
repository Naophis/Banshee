#ifndef MotionPlanning_HPP
#define MotionPlanning_HPP

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "include/adachi.hpp"
#include "include/defines.hpp"
#include "include/logging_task.hpp"
#include "include/path_creator.hpp"
#include "include/planning_task.hpp"
#include "include/trajectory_creator.hpp"
#include "include/ui.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

class MotionPlanning {
public:
  MotionPlanning() {}
  virtual ~MotionPlanning() {}

  void set_tgt_val(std::shared_ptr<motion_tgt_val_t> &_tgt_val);

  void set_sensing_entity(std::shared_ptr<sensing_result_entity_t> &_entity);
  void set_input_param_entity(std::shared_ptr<input_param_t> &_param);

  void set_userinterface(std::shared_ptr<UserInterface> &_ui);
  void set_path_creator(std::shared_ptr<PathCreator> &_pc);

  std::shared_ptr<Adachi> fake_adachi;

  MotionResult go_straight(param_straight_t &p);
  MotionResult go_straight(param_straight_t &p, std::shared_ptr<Adachi> &adachi,
                           bool search_mode);
  MotionResult pivot_turn(param_roll_t &p);
  void normal_slalom(param_normal_slalom_t &p, param_straight_t &p_str);
  void n_slalom(param_normal_slalom_t &p, param_straight_t &p_str);

  MotionResult slalom(slalom_param2_t &sp, TurnDirection dir,
                      next_motion_t &next_motion, bool dia);
  MotionResult slalom(slalom_param2_t &sp, TurnDirection dir,
                      next_motion_t &next_motion);
  MotionResult slalom(slalom_param2_t &sp, TurnDirection dir,
                      next_motion_t &next_motion, bool dia,
                      std::shared_ptr<Adachi> &adachi, bool search_mode);
  MotionResult search_front_ctrl(param_straight_t &p);

  MotionResult wall_off(param_straight_t &p, bool dia);

  void reset_tgt_data();
  void reset_ego_data();
  void reset_gyro_ref();
  void reset_gyro_ref_with_check();
  void coin();
  void keep();

  void exec_path_running(param_set_t &param_set);

  void set_planning_task(std::shared_ptr<PlanningTask> &pt);
  void set_logging_task(std::shared_ptr<LoggingTask> &lt);

  MotionResult front_ctrl(bool limit);

  void wall_off(TurnDirection td, param_straight_t &ps_front);
  bool wall_off_dia(TurnDirection td, param_straight_t &ps_front);
  void req_error_reset();

  void system_identification(MotionType mt, float duty_l, float duty_r,
                             float time);

  std::shared_ptr<motion_tgt_val_t> tgt_val;

  QueueHandle_t *qh;
  void set_queue_handler(QueueHandle_t &_qh) { qh = &_qh; }
  TaskHandle_t *th;
  void set_task_handler(TaskHandle_t &_th) { th = &_th; }

  void notify() {
    xTaskNotifyGive(notify_handle); //
  }

private:
  TaskHandle_t notify_handle = xTaskGetCurrentTaskHandle();
  void calc_dia135_offset(param_straight_t &front, param_straight_t &back,
                          TurnDirection dir, bool exec_wall_off);
  void calc_dia45_offset(param_straight_t &front, param_straight_t &back,
                         TurnDirection dir, bool exec_wall_off);
  std::shared_ptr<UserInterface> ui;

  std::shared_ptr<sensing_result_entity_t> sensing_result;
  std::shared_ptr<input_param_t> param;
  std::shared_ptr<PathCreator> pc;
  TrajectoryCreator tc;
  std::shared_ptr<PlanningTask> pt;
  std::shared_ptr<LoggingTask> lt;
  // const float th_offset_dist = 5.0;
  param_straight_t ps_front;
  param_straight_t ps_back;
  ego_odom_t ego;
  bool dia = false;
  param_straight_t ps;
  next_motion_t nm;
  MotionResult res_f;
};
#endif