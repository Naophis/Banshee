#include "driver/adc.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/mcpwm.h"
#include "driver/pcnt.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/timer.h"
#include "driver/uart.h"
#include "esp_adc_cal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "include/adachi.hpp"
#include "include/defines.hpp"
#include "include/sensing.hpp"
#include "sdkconfig.h"
#include "soc/adc_channel.h"
#include "soc/ledc_periph.h"
#include "xtensa/core-macros.h"
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <esp_task_wdt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
// #include "VL53L0X.h"
// #include "esp_efuse_rtc_calib.h"

#include "include/logging_task.hpp"
#include "include/main_task.hpp"
#include "include/planning_task.hpp"
#include "include/sensing_task.hpp"

#include "esp_system.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "esp_debug_helpers.h"
#include "rom/uart.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include <eigen3/Eigen/Eigen>

// #include <Eigen/Core>

void init_uart() {
  uart_config_t uart_config;
  uart_config.baud_rate = 3 * 1000 * 1000;
  uart_config.data_bits = UART_DATA_8_BITS;
  uart_config.parity = UART_PARITY_DISABLE;
  uart_config.stop_bits = UART_STOP_BITS_1;
  uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  // uart_config.rx_flow_ctrl_thresh = 122;
  uart_config.source_clk = UART_SCLK_APB;
  int intr_alloc_flags = 0;
  uart_param_config(UART_NUM_0, &uart_config);
  uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags);
  uart_param_config(UART_NUM_0, &uart_config);
  uart_set_pin(UART_NUM_0, TXD, RXD, RTS, CTS);
}

void init_gpio() {
  gpio_config_t io_conf;
  // 割り込みをしない
  io_conf.intr_type = GPIO_INTR_DISABLE;
  // 出力モード
  io_conf.mode = GPIO_MODE_OUTPUT;
  // 設定したいピンのビットマスク
  io_conf.pin_bit_mask = 0;
  // io_conf.pin_bit_mask |= 1ULL << LED_R90;
  // io_conf.pin_bit_mask |= 1ULL << LED_R45;
  // io_conf.pin_bit_mask |= 1ULL << LED_L45;
  // io_conf.pin_bit_mask |= 1ULL << LED_L90;
  io_conf.pin_bit_mask |= 1ULL << LED_A0;
  io_conf.pin_bit_mask |= 1ULL << LED_A1;
  io_conf.pin_bit_mask |= 1ULL << LED_EN;

  io_conf.pin_bit_mask |= 1ULL << L_CW_CCW1;
  io_conf.pin_bit_mask |= 1ULL << R_CW_CCW1;
  io_conf.pin_bit_mask |= 1ULL << SUCTION_PWM;

  io_conf.pin_bit_mask |= 1ULL << Motor_L_PWM;
  io_conf.pin_bit_mask |= 1ULL << Motor_R_PWM;

  // io_conf.pin_bit_mask |= 1ULL << LED1;
  // io_conf.pin_bit_mask |= 1ULL << LED2;
  // io_conf.pin_bit_mask |= 1ULL << LED3;
  // io_conf.pin_bit_mask |= 1ULL << LED4;
  // io_conf.pin_bit_mask |= 1ULL << LED5;

  io_conf.pin_bit_mask |= 1ULL << BUZZER;

  // io_conf.pin_bit_mask |= 1ULL << SUCTION_PWM;

  // 内部プルダウンしない
  io_conf.pull_down_en = (gpio_pulldown_t)0;
  // 内部プルアップしない
  io_conf.pull_up_en = (gpio_pullup_t)0;
  // 設定をセットする
  gpio_config(&io_conf);
  gpio_set_level(SUCTION_PWM, 0);
  // gpio_set_direction((gpio_num_t)GPIO_OUTPUT_IO_8,
}

DRAM_ATTR std::shared_ptr<input_param_t> param =
    std::make_shared<input_param_t>();
DRAM_ATTR std::shared_ptr<sensing_result_entity_t> sensing_entity =
    std::make_shared<sensing_result_entity_t>();
DRAM_ATTR std::shared_ptr<motion_tgt_val_t> tgt_val =
    std::make_shared<motion_tgt_val_t>();
DRAM_ATTR std::shared_ptr<pid_error_entity_t> error_entity =
    std::make_shared<pid_error_entity_t>();
DRAM_ATTR std::shared_ptr<PlanningTask> pt = std::make_shared<PlanningTask>();
DRAM_ATTR std::shared_ptr<LoggingTask> lt = std::make_shared<LoggingTask>();
DRAM_ATTR std::shared_ptr<MainTask> mt = std::make_shared<MainTask>();
DRAM_ATTR std::shared_ptr<SensingTask> st = std::make_shared<SensingTask>();
// std::shared_ptr<Sensing> sn = std::make_shared<Sensing>();
TaskHandle_t xTaskHandler;
// SensingTask st;

std::shared_ptr<sensing_result_entity_t> get_sensing_entity() {
  return sensing_entity;
}

extern "C" void app_main() {
  // Adachi adachi;

  init_gpio();
  init_uart();

  QueueHandle_t xQueue;
  xQueue = xQueueCreate(4, sizeof(motion_tgt_val_t *));

  gpio_set_level(SUCTION_PWM, 0);
  int c = 0;

  gpio_set_level(SUCTION_PWM, 0);
  param->tire = 12.0;
  param->dt = 0.001;
  param->motor_pid.p = 0.175;
  param->motor_pid.i = 0.0175;
  param->motor_pid.d = 0.0;
  param->gyro_pid.p = 2.5;
  param->gyro_pid.i = 0.75;
  param->gyro_pid.d = 0.0;
  tgt_val->ego_in.v = 0;
  tgt_val->ego_in.w = 0;
  param->gyro_param.gyro_w_gain_left = 0.0002645;

  // st.set_sensing_entity(sensing_entity);
  // st.set_tgt_val(tgt_val);
  // st.set_main_task(mt);
  // st.set_input_param_entity(param);
  // st.set_planning_task(pt);
  // st.set_queue_handler(xQueue);
  // st.set_task_handler(xTaskHandler);
  // st.create_task(0);
  st->set_sensing_entity(sensing_entity);
  st->set_tgt_val(tgt_val);
  st->set_main_task(mt);
  st->set_input_param_entity(param);
  st->set_planning_task(pt);
  st->set_queue_handler(xQueue);
  st->set_task_handler(xTaskHandler);
  st->create_task(0);

  // sn->init();

  pt->set_sensing_entity(sensing_entity);
  pt->set_input_param_entity(param);
  pt->set_tgt_val(tgt_val);
  pt->set_error_entity(error_entity);
  pt->set_queue_handler(xQueue);
  pt->set_task_handler(xTaskHandler);
  // pt->set_task_handler((TaskHandle_t) NULL);
  // pt->set_sensing(sn);
  pt->create_task(0);

  lt->set_sensing_entity(sensing_entity);
  lt->set_input_param_entity(param);
  lt->set_tgt_val(tgt_val);
  lt->set_error_entity(error_entity);
  lt->create_task(1);
  pt->set_logging_task(lt);

  mt->set_sensing_entity(sensing_entity);
  mt->set_input_param_entity(param);
  mt->set_tgt_val(tgt_val);
  mt->set_planning_task(pt);
  mt->set_logging_task(lt);
  mt->set_queue_handler(xQueue);
  mt->set_task_handler(xTaskHandler);
  // mt->set_task_handler((TaskHandle_t) NULL);
  mt->create_task(1);

  esp_task_wdt_reset();
  esp_task_wdt_add(xTaskGetIdleTaskHandleForCPU(0));
  esp_task_wdt_add(xTaskGetIdleTaskHandleForCPU(1));

  while (1) {
    vTaskDelay(5000.0 / portTICK_RATE_MS);
    if (mt->ui->button_state()) {
      printf("time_stamp: %d\n", tgt_val->nmr.timstamp);
      printf("motion_type: %d\n", static_cast<int>(tgt_val->motion_type));
      printf("fss.error: %d\n", tgt_val->fss.error);
      printf("motor_en: %d\n", (pt->motor_en) ? 1 : 0);
      printf("suction_en: %d\n", (pt->suction_en) ? 1 : 0);
    }
  }
}