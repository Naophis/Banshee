#ifndef SENSING_TASK_HPP
#define SENSING_TASK_HPP

#include "as5147p.hpp"
#include "defines.hpp"
#include "driver/pcnt.h"
#include "driver/timer.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "icm20689.hpp"
#include "lsm6dsr.hpp"
#include "main_task.hpp"
#include <deque>
#include <driver/adc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

class SensingTask {
public:
  SensingTask();
  virtual ~SensingTask();
  void create_task(const BaseType_t xCoreID);

  std::shared_ptr<sensing_result_entity_t> sensing_result;
  std::shared_ptr<PlanningTask> pt;
  void set_sensing_entity(std::shared_ptr<sensing_result_entity_t> &_entity);
  void set_planning_task(std::shared_ptr<PlanningTask> &_pt);

  QueueHandle_t *qh;
  void set_queue_handler(QueueHandle_t &_qh) { qh = &_qh; }
  std::shared_ptr<MainTask> mt;
  void set_main_task(std::shared_ptr<MainTask> &_mt) {
    mt = _mt; //
  }

  static void task_entry_point(void *task_instance);
  virtual void task();
  static void task_entry_point0(void *task_instance);

  void timer_10us_callback_main();
  void timer_200us_callback_main();
  void timer_250us_callback_main();
  void set_input_param_entity(std::shared_ptr<input_param_t> &_param);

  // ICM20689 gyro_if;
  LSM6DSR gyro_if;
  AS5147P enc_if;
  bool is_ready() { return ready; }
  std::deque<int> gyro_q;
  void set_tgt_val(std::shared_ptr<motion_tgt_val_t> &_tgt_val);

private:
  volatile int cnt_a = 0;
  esp_timer_handle_t timer_200us;
  esp_timer_handle_t timer_250us;
  esp_timer_handle_t timer_10us;

  motion_tgt_val_t *receive_req;

  static void timer_10us_callback(void *arg);
  static void timer_200us_callback(void *arg);
  static void timer_250us_callback(void *arg);

  void calc_vel(float dt);
  void set_gpio_state(gpio_num_t gpio_num, int state) {
    const int num = (int)gpio_num;
    if (num < 32) {
      if (state) {
        GPIO.out_w1ts = BIT(num);
      } else {
        GPIO.out_w1tc = BIT(num);
      }
    } else {
      if (state) {
        GPIO.out1_w1ts.val = BIT(num - 32);
      } else {
        GPIO.out1_w1tc.val = BIT(num - 32);
      }
    }
  }

  float calc_sensor(float data, float a, float b);
  volatile int lec_cnt = 0;
  std::shared_ptr<input_param_t> param;
  int led_light_delay_cnt = 10000;
  xTaskHandle handle = 0;
  bool ready;
  timer_isr_handle_t handle_isr;
  void encoder_init(const pcnt_unit_t unit, const gpio_num_t pinA,
                    const gpio_num_t pinB);
  const TickType_t xDelay = 1 / portTICK_PERIOD_MS;

  // void timer_isr(void *parameters);
  std::shared_ptr<motion_tgt_val_t> tgt_val;

  bool itr_state = true;
  static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
  static const adc_atten_t atten = ADC_ATTEN_DB_11;
  pcnt_config_t pcnt_config_0 = {
      // .pulse_gpio_num = pinA,
      // .ctrl_gpio_num = pinB,
      .lctrl_mode = PCNT_MODE_KEEP,
      .hctrl_mode = PCNT_MODE_REVERSE,
      .pos_mode = PCNT_COUNT_INC,
      .neg_mode = PCNT_COUNT_DEC,
      .counter_h_lim = ENCODER_H_LIM_VAL,
      .counter_l_lim = ENCODER_L_LIM_VAL,
      // .unit = unit,
      .channel = PCNT_CHANNEL_0,
  };
  pcnt_config_t pcnt_config_1 = {
      // .pulse_gpio_num = pinB,
      // .ctrl_gpio_num = pinA,
      .lctrl_mode = PCNT_MODE_REVERSE,
      .hctrl_mode = PCNT_MODE_KEEP,
      .pos_mode = PCNT_COUNT_INC,
      .neg_mode = PCNT_COUNT_DEC,
      .counter_h_lim = ENCODER_H_LIM_VAL,
      .counter_l_lim = ENCODER_L_LIM_VAL,
      // .unit = unit,
      .channel = PCNT_CHANNEL_1,
  };
};

#endif