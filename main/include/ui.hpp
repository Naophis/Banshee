#ifndef UI_HPP
#define UI_HPP

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "include/Music.hpp"
#include "include/defines.hpp"

enum class MODE : int {
  SEARCH = 0,
  SEARCH2 = 1,
  FAST1 = 2,
  FAST2 = 3,
};

class UserInterface {
public:
  UserInterface(){};
  virtual ~UserInterface(){};

  void motion_check();
  void set_sensing_entity(std::shared_ptr<sensing_result_entity_t> &_entity);
  void set_tgt_val(std::shared_ptr<motion_tgt_val_t> &_tgt_val);

  void LED_on(int byte);
  void LED_off(int byte);
  void LED_off_all();
  void LED_on_all();

  void LED(int byte, int state);
  void LED_bit(int b0, int b1, int b2, int b3, int b4);
  void LED_otherwise(int byte, int state);

  int encoder_operation();
  bool button_state();
  bool button_state_hold();
  void music_async(MUSIC m, int time);
  void music_sync(MUSIC m, int time);
  void hello_exia();
  void coin(int t);
  void error();

  TurnDirection select_direction();
  TurnDirection select_direction2();
  QueueHandle_t *qh;

  void set_queue_handler(QueueHandle_t &_qh) { qh = &_qh; }

private:
  std::shared_ptr<sensing_result_entity_t> sensing_result;
  std::shared_ptr<motion_tgt_val_t> tgt_val;

  void LED_on_off(gpio_num_t gpio_num, int on_off);
};

#endif