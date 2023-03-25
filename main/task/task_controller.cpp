#include "include/task_controller.hpp"

TaskController::TaskController() {}

TaskController::~TaskController() {}

void TaskController::init() {
  //
}

void TaskController::sensing_task(void *arg) {
  const TickType_t xDelay = 1 / portTICK_PERIOD_MS;
  int i = 0;
  while (1) {
    if (i % 2 == 0) {
      gpio_set_level(LED5, 1);
      gpio_set_level(LED4, 0);
    } else {
      gpio_set_level(LED4, 1);
      gpio_set_level(LED5, 0);
    }
    i++;
    vTaskDelay(xDelay);
  }
}