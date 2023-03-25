#ifndef TASK_CONTROLLER_H
#define TASK_CONTROLLER_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "defines.hpp"

class TaskController {
public:
  TaskController();
  ~TaskController();
  void init();
  void sensing_task(void *arg);

protected:
  xTaskHandle handle = 0;
};

#endif