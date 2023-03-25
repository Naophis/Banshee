#ifndef BASE_TASK_HPP
#define BASE_TASK_HPP

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "defines.hpp"

class BaseTask {
public:
  BaseTask() = delete;

  BaseTask(const char *_name, int _priority, uint32_t _stack_size = 8192)
      : name(_name), priority(_priority), stack_size(_stack_size) {}

  virtual ~BaseTask() { delete_task(); }

  void create_task(const BaseType_t xCoreID) {
    // xTaskCreate(task_entry_point, name, stack_size, this, priority, &handle);
    xTaskCreatePinnedToCore(task_entry_point, name, stack_size, this, priority,
                            &handle, xCoreID);
  }

  void delete_task() { vTaskDelete(handle); }

protected:
  xTaskHandle handle = 0;
  const char *name;
  int priority;
  uint32_t stack_size;

  virtual void task() = 0;

  static void task_entry_point(void *task_instance) {
    static_cast<BaseTask *>(task_instance)->task();
  }
};

#endif