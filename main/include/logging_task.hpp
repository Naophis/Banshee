#ifndef LOGGING_TASK_HPP
#define LOGGING_TASK_HPP

#include "defines.hpp"
#include "esp_system.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gen_code_conv_single2half/half_type.h"

#include <esp_heap_caps.h>
#include <fstream>

class LoggingTask {
public:
  LoggingTask(){};
  ~LoggingTask(){};

  void create_task(const BaseType_t xCoreID);
  static void task_entry_point(void *task_instance);
  virtual void task();
  void dump();

  void
  set_sensing_entity(std::shared_ptr<sensing_result_entity_t> &_sensing_result);
  void set_input_param_entity(std::shared_ptr<input_param_t> &_param);
  void set_tgt_val(std::shared_ptr<motion_tgt_val_t> &_tgt_val);

  void start_slalom_log();
  void stop_slalom_log();

  void dump_log(const std::string file_name);
  void dump_log_sysid(const std::string file_name);

  void save(const std::string file_name);
  void save_sysid(const std::string file_name);

  void change_sysid_mode(float duty_l, float duty_r, int time);

  void exec_log();

  bool active_slalom_log = false;

private:
  bool log_mode = true;
  bool req_logging_active = false;
  xTaskHandle handle = 0;
  float16_bitmap fbm;
  uint16_bitmap uibm;

  bool *receive_logging_active_req;
  QueueHandle_t qh;
  std::shared_ptr<sensing_result_entity_t> sensing_result;
  std::shared_ptr<input_param_t> param;
  std::shared_ptr<motion_tgt_val_t> tgt_val;

  int idx_slalom_log = 0;
  FILE *f_slalom_log;
  // std::vector<std::shared_ptr<log_data_t>> log_vec;
  std::vector<std::shared_ptr<log_data_t2>> log_vec;
  std::vector<std::shared_ptr<sysid_log>> sysidlog_vec;
  float calc_sensor(float data, float a, float b, char motion_type);
  float duty_l = 0;
  float duty_r = 0;
  int time = 0;

  template <typename T, typename... Args>
  std::unique_ptr<T> make_unique(Args &&... args) {
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
  }
};

#endif