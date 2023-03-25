#ifndef SEARCH_CONTROLLER_HPP
#define SEARCH_CONTROLLER_HPP

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "defines.hpp"
#include "include/adachi.hpp"
#include "include/logging_task.hpp"
#include "include/motion_planning.hpp"
#include "include/planning_task.hpp"

class SearchController {
public:
  SearchController();
  ~SearchController();
  void reset();
  SearchResult exec(param_set_t &p_set, SearchMode sm);

  void set_lgc(std::shared_ptr<MazeSolverBaseLgc> &_lgc);
  void set_motion_plannning(std::shared_ptr<MotionPlanning> &_mp);
  void set_planning_task(std::shared_ptr<PlanningTask> &_pt);
  void set_sensing_entity(std::shared_ptr<sensing_result_entity_t> &_entity);
  void set_logging_task(std::shared_ptr<LoggingTask> &_lt);
  void set_userinterface(std::shared_ptr<UserInterface> &_ui);

  void print_maze();
  void set_input_param_entity(std::shared_ptr<input_param_t> &_param);
  void save_maze_data();
  MotionResult pivot(param_set_t &p_set, float diff);
  MotionResult pivot90(param_set_t &p_set, const TurnDirection td, float diff);
  MotionResult timeup_motion(param_set_t &p_set);

private:
  bool saved = false;
  MotionResult mr = MotionResult::NONE;
  std::shared_ptr<MotionPlanning> mp;
  std::shared_ptr<MazeSolverBaseLgc> lgc;
  std::shared_ptr<PlanningTask> pt;
  std::shared_ptr<Adachi> adachi;
  std::shared_ptr<LoggingTask> lt;
  std::shared_ptr<UserInterface> ui;
  std::shared_ptr<input_param_t> param;

  std::shared_ptr<ego_t> ego;
  std::shared_ptr<sensing_result_entity_t> sensing_result;
  std::shared_ptr<Adachi> fake_adachi;

  MotionResult go_straight_wrapper(param_set_t &p_set, float diff,
                                   StraightType st);
  MotionResult slalom(param_set_t &p_set, const TurnDirection td, float diff);
  MotionResult straight_offset(param_set_t &p_set, const TurnDirection td,
                               float diff);
  MotionResult finish(param_set_t &p_set);

  void front_wall_ctrl();

  void judge_wall();
  bool judge2();
  bool wall_n = false;
  bool wall_e = false;
  bool wall_w = false;
  bool wall_s = false;
  param_straight_t p;
  param_roll_t pr;
};

#endif