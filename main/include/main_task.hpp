#ifndef MAIN_TASK_HPP
#define MAIN_TASK_HPP

#include "defines.hpp"
#include "driver/pcnt.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "icm20689.hpp"
#include "include/maze_solver.hpp"
#include "include/planning_task.hpp"
#include <driver/adc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_system.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"

#include "include/adachi.hpp"
#include "include/logging_task.hpp"
#include "include/logic.hpp"
#include "include/motion_planning.hpp"
#include "include/path_creator.hpp"
#include "include/search_controller.hpp"
#include "include/ui.hpp"

// #include "libs/nlohmnn-json/json.hpp"
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "cJSON.h"
#include <esp_heap_caps.h>
#include <fstream>

class MainTask {
public:
  MainTask();

  // MainTask(const exia_io_t &_exia) : exia_io(_exia);
  virtual ~MainTask();

  void create_task(const BaseType_t xCoreID);
  static void task_entry_point(void *task_instance);

  void set_sensing_entity(std::shared_ptr<sensing_result_entity_t> &_entity);
  void set_input_param_entity(std::shared_ptr<input_param_t> &_param);
  void set_tgt_val(std::shared_ptr<motion_tgt_val_t> &_tgt_val);

  void set_planning_task(std::shared_ptr<PlanningTask> &pt);
  void set_logging_task(std::shared_ptr<LoggingTask> &lt);

  virtual void task();
  void check_battery();

  TurnType cast_turn_type(std::string str);
  std::shared_ptr<UserInterface> ui;
  QueueHandle_t *qh;

  void set_queue_handler(QueueHandle_t &_qh) {
    qh = &_qh;
    mp->set_queue_handler(_qh);
    ui->set_queue_handler(_qh);
  }
  void notify() {
    xTaskNotifyGive(notify_handle); //
  }
  std::shared_ptr<MotionPlanning> mp;

  bool ready = false;

private:
  int test_search_mode = 0;
  SearchResult sr;
  xTaskHandle handle = 0;
  TaskHandle_t notify_handle = xTaskGetCurrentTaskHandle();

  std::shared_ptr<PlanningTask> pt;
  std::shared_ptr<LoggingTask> lt;

  std::shared_ptr<sensing_result_entity_t> sensing_result;
  std::shared_ptr<input_param_t> param;
  std::shared_ptr<motion_tgt_val_t> tgt_val;

  param_straight_t ps;
  param_roll_t pr;
  param_normal_slalom_t pns;
  system_t sys;
  turn_param_profile_t tpp;
  unordered_map<unsigned char, vector<pair<TurnType, string>>> turn_map;

  const char *base_path = "/spiflash";
  esp_vfs_fat_mount_config_t mount_config;
  wl_handle_t s_wl_handle = WL_INVALID_HANDLE;
  // profile_idx_t p_idx;
  std::unordered_map<TurnType, int> p_idx;
  param_set_t sp;
  straight_param_t str_p;
  slalom_param2_t sp2;
  LED_bit lbit;
  TurnDirection rorl;
  TurnDirection rorl2;
  next_motion_t nm;
  param_set_t p_set;

  const TickType_t xDelay1000 = 1000.0 / portTICK_PERIOD_MS;
  const TickType_t xDelay500 = 500.0 / portTICK_PERIOD_MS;
  const TickType_t xDelay100 = 100.0 / portTICK_PERIOD_MS;
  const TickType_t xDelay50 = 50.0 / portTICK_PERIOD_MS;
  const TickType_t xDelay10 = 10.0 / portTICK_PERIOD_MS;
  const TickType_t xDelay1 = 1.0 / portTICK_PERIOD_MS;
  void dump1();
  void dump2();
  int select_mode();
  void keep_pivot();
  void echo_sensing_result_with_json();
  // void entity_to_json(nlohmann::json &j);
  void recieve_data();
  void test_run();
  void test_back();

  void reset_ego_data();
  void reset_tgt_data();
  void req_error_reset();
  void test_turn();
  void test_pivot_n();
  void test_pivot_n2();
  void test_sla();
  void test_run_sla();
  void test_search_sla();
  void test_front_wall_offset();
  void test_sla_walloff();
  void test_dia_walloff();
  void test_front_ctrl(bool mode);
  void rx_uart_json();
  void path_run(int idx, int idx2);

  void save_json_data(std::string &str);
  vector<string> split(const string &s, char delim);
  void load_param();
  void load_hw_param();
  void load_offset_param();
  void load_sensor_param();
  void load_sys_param();
  void load_turn_param_profiles(bool const_mode);
  void load_slalom_param();
  void load_slalom_param(int idx, int idx2);
  void load_sla(int idx, string turn_name, slalom_param2_t &sla_p);
  void load_slas(int idx, vector<pair<TurnType, string>> &turn_name_list,
                 std::unordered_map<TurnType, slalom_param2_t> &turn_map);
  void
  load_straight(int idx,
                std::unordered_map<StraightType, straight_param_t> &str_map);
  void load_circuit_path();
  void save_maze_data(bool write);
  void save_maze_kata_data(bool write);
  void save_maze_return_data(bool write);
  void read_maze_data();

  void test_system_identification(bool para);

  std::vector<param_set_t> paramset_list;
  param_set_t param_set;
  std::shared_ptr<MazeSolverBaseLgc> lgc;
  // std::shared_ptr<Adachi> adachi;
  std::shared_ptr<SearchController> search_ctrl;
  std::shared_ptr<PathCreator> pc;
  int mode_num;
  float backup_l;
  float backup_r;
  int file_idx;
  slalom_param2_t sla_p;
};

#endif