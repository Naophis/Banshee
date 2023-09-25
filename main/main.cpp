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
SensingTask st;

void init_uart() {
  uart_config_t uart_config;
  uart_config.baud_rate = 2 * 1000 * 1000;
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

  io_conf.pin_bit_mask |= 1ULL << A_CW_CCW1;
  io_conf.pin_bit_mask |= 1ULL << B_CW_CCW1;
  // io_conf.pin_bit_mask |= 1ULL << A_CW_CCW2;
  // io_conf.pin_bit_mask |= 1ULL << B_CW_CCW2;
  io_conf.pin_bit_mask |= 1ULL << SUCTION_PWM;

  io_conf.pin_bit_mask |= 1ULL << A_PWM;
  io_conf.pin_bit_mask |= 1ULL << B_PWM;

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
int gyro_mode = 0;
ICM20689 gyro_if;
LSM6DSR gyro_if2;
AS5147P enc_if;
void timer_isr(void *parameters) {
  timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);
  timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_0);
  if (st.is_ready()) {
    gyro_mode++;
    if (gyro_mode == 1 || gyro_mode == 3) {
      gyro_if.req_read2byte_itr(0x47);
    } else if (gyro_mode == 2 || gyro_mode == 4) {
      st.gyro_q.push_back(gyro_if.read_2byte_itr());
    }
    if (st.gyro_q.size() > GY_DQ_SIZE) {
      st.gyro_q.pop_front();
    }
    if (gyro_mode == 4) {
      gyro_mode = 0;
    }
  }
}
void hwtimer_init(void) {
  timer_config_t config;
  config.alarm_en = TIMER_ALARM_EN;
  config.counter_en = TIMER_PAUSE;
  config.clk_src = TIMER_SRC_CLK_APB;
  config.auto_reload = TIMER_AUTORELOAD_EN;
  config.counter_dir = TIMER_COUNT_UP;
  config.divider = 8; // 80Mhz / divider
  timer_init(TIMER_GROUP_0, TIMER_0, &config);
  timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, GY_CYCLE);
  timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_isr, NULL, 0, NULL);
  timer_enable_intr(TIMER_GROUP_0, TIMER_0);
  timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
  timer_start(TIMER_GROUP_0, TIMER_0);
}
std::shared_ptr<input_param_t> param = std::make_shared<input_param_t>();
std::shared_ptr<sensing_result_entity_t> sensing_entity =
    std::make_shared<sensing_result_entity_t>();
std::shared_ptr<motion_tgt_val_t> tgt_val =
    std::make_shared<motion_tgt_val_t>();
std::shared_ptr<PlanningTask> pt = std::make_shared<PlanningTask>();
std::shared_ptr<LoggingTask> lt = std::make_shared<LoggingTask>();
std::shared_ptr<MainTask> mt = std::make_shared<MainTask>();

std::shared_ptr<sensing_result_entity_t> get_sensing_entity() {
  return sensing_entity;
}

extern "C" void app_main3() {
  // init_gpio();
  // init_uart();
  // adc2_config_channel_atten(BATTERY, ADC_ATTEN_DB_11);

  // const esp_timer_create_args_t timer_200ms_args = {
  //     .callback = &timer_200ms_callback, .name = "timer_100ms"};

  // const esp_timer_create_args_t timer_10ms_args = {
  //     .callback = &timer_10ms_callback, .name = "timer_10ms"};

  // ESP_ERROR_CHECK(esp_timer_create(&timer_200ms_args, &timer_1000us));
  // ESP_ERROR_CHECK(esp_timer_create(&timer_10ms_args, &timer_100us));

  // ESP_ERROR_CHECK(esp_timer_start_periodic(timer_1000us, 250)); // 1m second
  // while (1) {
  //   vTaskDelay(pdMS_TO_TICKS(1000));
  // }
}
// ESP_ERROR_CHECK(esp_timer_start_periodic(adc_timer, 1000000)); // 1000m
// second ESP_ERROR_CHECK(esp_timer_start_periodic(adc_timer, 100000)); //
// 100m second ESP_ERROR_CHECK(esp_timer_start_periodic(adc_timer, 10000)); //
// 10m second

// static void multiply2Matrices()
// {
//     Eigen::MatrixXf M(2, 2);
//     Eigen::MatrixXf V(2, 2);
//     for (int i = 0; i <= 1; i++) {
//         for (int j = 0; j <= 1; j++) {
//             M(i, j) = 1;
//             V(i, j) = 2;
//         }
//     }
//     Eigen::MatrixXf Result = M * V;
//     std::cout << "MatrixXf Result = " << std::endl << Result << std::endl;
// }

// static void runSVD()
// {
//     Eigen::MatrixXf C;
//     C.setRandom(27, 18);
//     Eigen::JacobiSVD<Eigen::MatrixXf> svd(C, Eigen::ComputeThinU |
//     Eigen::ComputeThinV); Eigen::MatrixXf Cp = svd.matrixU() *
//     svd.singularValues().asDiagonal() * svd.matrixV().transpose();
//     Eigen::MatrixXf diff = Cp - C;
//     std::cout << "SDV matrix U: " << std::endl << svd.matrixU() << std::endl;
//     std::cout << "SDV singularValues: " << std::endl <<
//     svd.singularValues().transpose() << std::endl; std::cout << "SDV matrix
//     V: " << std::endl << svd.matrixV() << std::endl; std::cout << "diff:\n"
//     << diff.array().abs().sum() << "\n";
// }

// extern "C" void app_main() {
//     std::cout << "Eigen example." << std::endl;
//     multiply2Matrices();
//     runSVD();
//     std::cout << "Example finished!" << std::endl;
// }

extern "C" void app_main() {
  // Adachi adachi;

  init_gpio();
  init_uart();
  // adc2_config_channel_atten(BATTERY, ADC_ATTEN_DB_11);

  // if (true) {
  //   vTaskDelay(100.0 / portTICK_RATE_MS);
  // }

  // const int arraySize = 100;
  // // esp_err_t ret = heap_caps_init();
  // int *array =
  //     (int *)heap_caps_malloc(arraySize * sizeof(int), MALLOC_CAP_SPIRAM);
  // // 確保した配列を使用する

  // if (array == NULL) {
  //   while (true) {
  //     vTaskDelay(5000.0 / portTICK_RATE_MS);
  //   }
  //   printf("Memory allocation failed!\n");
  //   return;
  // }

  // for (int i = 0; i < arraySize; i++) {
  //   array[i] = i;
  // }
  // std::vector<int> vec(array, array + arraySize);
  // vec.push_back(42);
  // for (const auto v : vec) {
  //   printf("%d\n", v);
  // }

  // // 確保したメモリを解放する
  // heap_caps_free(array);

  QueueHandle_t xQueue;
  xQueue = xQueueCreate(1, sizeof(motion_tgt_val_t *));

  // esp_vfs_fat_mount_config_t mount_config;
  // mount_config.max_files = 8;
  // mount_config.format_if_mount_failed = true;
  // mount_config.allocation_unit_size = CONFIG_WL_SECTOR_SIZE;
  // const char *base_path = "/spiflash";
  // wl_handle_t s_wl_handle = WL_INVALID_HANDLE;

  // printf("storage0: try mount\n");
  // esp_err_t err = esp_vfs_fat_spiflash_mount_rw_wl(base_path, "storage0",
  //                                            &mount_config, &s_wl_handle);
  // if (err != ESP_OK) {
  //   printf("storage0: Failed to mount FATFS (%s)\n", esp_err_to_name(err));
  //   return;
  // } else {
  //   printf("storage0: mount OK\n");
  // }
  // gpio_set_level(A_CW_CCW1, 1);
  // gpio_set_level(B_CW_CCW1, 1);
  gpio_set_level(SUCTION_PWM, 0);
  int c = 0;
  // mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM2A, SUCTION_PWM);
  // mcpwm_config_t suction_pwm_conf;
  // memset(&suction_pwm_conf, 0, sizeof(suction_pwm_conf));
  // suction_pwm_conf.frequency = 200000; // PWM周波数= 10kHz,
  // suction_pwm_conf.cmpr_a = 0; // デューティサイクルの初期値（0%）
  // suction_pwm_conf.cmpr_b = 0; // デューティサイクルの初期値（0%）
  // suction_pwm_conf.counter_mode = MCPWM_UP_COUNTER;
  // suction_pwm_conf.duty_mode = MCPWM_DUTY_MODE_0; // アクティブハイ
  // suction_pwm_conf.cmpr_a = 0; // デューティサイクルの初期値（0%）
  // mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_2, &suction_pwm_conf);

  // while (true) {
  //   printf("hello, world %d\n", c++);

  //   mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_B);
  //   float duty = 40.0 * c / 1000;
  //   if (duty > 40) {
  //     duty = 40;
  //   }
  //   mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_A, duty);
  //   mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_A,
  //                       MCPWM_DUTY_MODE_0);

  //   vTaskDelay(10.0 / portTICK_RATE_MS);
  // }

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

  st.set_sensing_entity(sensing_entity);
  st.set_tgt_val(tgt_val);
  st.set_main_task(mt);
  st.set_input_param_entity(param);
  st.create_task(0);

  pt->set_sensing_entity(sensing_entity);
  pt->set_input_param_entity(param);
  pt->set_tgt_val(tgt_val);
  pt->set_queue_handler(xQueue);
  pt->create_task(0);

  lt->set_sensing_entity(sensing_entity);
  lt->set_input_param_entity(param);
  lt->set_tgt_val(tgt_val);
  lt->create_task(1);
  pt->set_logging_task(lt);
  // vTaskDelay(1000.0 / portTICK_RATE_MS);
  // int i = 0;
  // for (const auto &v : lt->log_vec) {
  //   printf("%d, %p %p %p\n", i++, &v, &v->img_v, &v->v_l);
  // }

  // while (true) {
  //   vTaskDelay(5000.0 / portTICK_RATE_MS);
  // }

  mt->set_sensing_entity(sensing_entity);
  mt->set_input_param_entity(param);
  mt->set_tgt_val(tgt_val);
  mt->set_planning_task(pt);
  mt->set_logging_task(lt);
  mt->set_queue_handler(xQueue);
  mt->create_task(1);

  // /* Set the GPIO as a push/pull output */

  // gpio_set_level(LED1, 0);
  // gpio_set_level(LED2, 0);
  // gpio_set_level(LED3, 0);
  // gpio_set_level(LED4, 0);
  // gpio_set_level(LED5, 0);
  esp_task_wdt_reset();
  esp_task_wdt_add(xTaskGetIdleTaskHandleForCPU(0));
  esp_task_wdt_add(xTaskGetIdleTaskHandleForCPU(1));

  // init_i2c_master();
  uint8_t writeBuffer[2];
  writeBuffer[0] = 0x00;
  writeBuffer[1] = 0x00;
  while (1) {
    vTaskDelay(5000.0 / portTICK_RATE_MS);
    // writeBuffer[0] = (i << 5) | 0x18;
    // writeBuffer[0] = (i << 5) | 0x1f;

    // printf("%c[2J", ESC);   /* 画面消去 */
    // printf("%c[0;0H", ESC); /* 戦闘戻す*/
    // printf("%d, %d\n", st.sensing_result->led_sen.left90.raw,
    //        st.sensing_result->led_sen.right90.raw);
    // printf("%d %x\n", i, i << 5);
    // printf("%d %x\n", i, (i << 5) | 0x1F);
    // i2c_master_write_to_device(0, 0x9A, writeBuffer, 2, 1 /
    // portTICK_RATE_MS); printf("%x, %x\n", (i << 5) | 0x18, (0x04 << 5) |
    // 0x18); printf("battery: %f\n", st.sensing_result->battery.data);
    // printf("gyro: %d\n", st.sensing_result->gyro.raw);
    // SCCB_Write(0x9A, writeBuffer[0], writeBuffer[0]);
    // i++;
    // if (i > 6) {
    //   i = 1;
    // }
    // i--;
    // if (i < 1) {
    //   i = 6;
    // }
    if (mt->ui->button_state()) {
      printf("time_stamp: %d\n", tgt_val->nmr.timstamp);
      printf("motion_type: %d\n", static_cast<int>(tgt_val->motion_type));
      printf("fss.error: %d\n", tgt_val->fss.error);
      printf("motor_en: %d\n", (pt->motor_en) ? 1 : 0);
      printf("suction_en: %d\n", (pt->suction_en) ? 1 : 0);
    }
  }
}