#ifndef DEFINES_HPP
#define DEFINES_HPP

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "gen_code_mpc/bus.h"
#include "include/maze_solver.hpp"
#include "sdkconfig.h"
#include "soc/adc_channel.h"
#include "soc/gpio_struct.h"
#include "soc/ledc_periph.h"

#include "fs.hpp"
#include "include/enums.hpp"
#include "include/structs.hpp"
#include "sdkconfig.h"
#include <bitset>
#include <initializer_list>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "icm20689.hpp"
#include "lsm6dsr.hpp"

#define portTICK_RATE_MS portTICK_PERIOD_MS
#define xTaskHandle TaskHandle_t

#define ABS(IN) ((IN) < 0 ? -(IN) : (IN))

constexpr int GY_DQ_SIZE = 2;
constexpr int GY_CYCLE = 2500; // 2500=1/4msec
// constexpr int GY_CYCLE = 1250; // 1250=1/8msec
constexpr int GY_MODE = 0;
constexpr float cell_size = 90;

// constexpr gpio_num_t LED_R90 = GPIO_NUM_8;
// constexpr gpio_num_t LED_R45 = GPIO_NUM_9;
// constexpr gpio_num_t LED_F = GPIO_NUM_13;
// constexpr gpio_num_t LED_L45 = GPIO_NUM_18;
// constexpr gpio_num_t LED_L90 = GPIO_NUM_21;

constexpr gpio_num_t SDA_PIN = GPIO_NUM_36;
constexpr gpio_num_t SCL_PIN = GPIO_NUM_37;

// constexpr unsigned int LED_R90_BIT = 8;
// constexpr unsigned int LED_R45_BIT = 9;
// constexpr unsigned int LED_L45_BIT = 18;
// constexpr unsigned int LED_L90_BIT = 21;

constexpr gpio_num_t LED_EN = GPIO_NUM_10;
constexpr gpio_num_t LED_A0 = GPIO_NUM_11;
constexpr gpio_num_t LED_A1 = GPIO_NUM_12;

constexpr unsigned int LED_EN_BIT = 7;
constexpr unsigned int LED_A0_BIT = 8;
constexpr unsigned int LED_A1_BIT = 9;

constexpr gpio_num_t TXD = GPIO_NUM_43;
constexpr gpio_num_t RXD = GPIO_NUM_44;
constexpr gpio_num_t RTS = GPIO_NUM_15;
constexpr gpio_num_t CTS = GPIO_NUM_16;

constexpr gpio_num_t A_CW_CCW1 = GPIO_NUM_41;
constexpr unsigned int A_CW_CCW1_BIT = 41 - 32;

constexpr gpio_num_t B_CW_CCW1 = GPIO_NUM_45;
constexpr unsigned int B_CW_CCW1_BIT = 45 - 32;

constexpr gpio_num_t A_PWM = GPIO_NUM_42;
constexpr gpio_num_t B_PWM = GPIO_NUM_46;

constexpr gpio_num_t BUZZER = GPIO_NUM_5;

// constexpr gpio_num_t LED1 = GPIO_NUM_13;
// constexpr gpio_num_t LED2 = GPIO_NUM_14;
// constexpr gpio_num_t LED3 = GPIO_NUM_15;
// constexpr gpio_num_t LED4 = GPIO_NUM_10;
// constexpr gpio_num_t LED5 = GPIO_NUM_16;

constexpr gpio_num_t SW1 = GPIO_NUM_38;

constexpr gpio_num_t EN_MISO = GPIO_NUM_1;
constexpr gpio_num_t EN_MOSI = GPIO_NUM_2;
constexpr gpio_num_t EN_CLK = GPIO_NUM_3;
constexpr gpio_num_t EN_GN_SSL = GPIO_NUM_4;

constexpr gpio_num_t ENC_R_CS = GPIO_NUM_6;
constexpr gpio_num_t ENC_L_CS = GPIO_NUM_37;
constexpr gpio_num_t ENC_CLK = GPIO_NUM_7;
constexpr gpio_num_t ENC_MISO = GPIO_NUM_8; // B
constexpr gpio_num_t ENC_MOSI = GPIO_NUM_9; // A

constexpr gpio_num_t SUCTION_PWM = GPIO_NUM_35;

#define SEN_R90 ADC2_CHANNEL_2
#define SEN_R45 ADC2_CHANNEL_4
#define SEN_R45_2 ADC2_CHANNEL_5
#define SEN_L45_2 ADC2_CHANNEL_6
#define SEN_L45 ADC2_CHANNEL_7
#define SEN_L90 ADC2_CHANNEL_8
#define BATTERY ADC2_CHANNEL_9

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

// constexpr adc2_channel_t SEN_R90 = ADC2_CHANNEL_0;
// constexpr adc2_channel_t SEN_R45 = ADC2_CHANNEL_1;
// constexpr adc2_channel_t SEN_F = ADC2_CHANNEL_4;
// constexpr adc2_channel_t SEN_L45 = ADC2_CHANNEL_6;
// constexpr adc2_channel_t SEN_L90 = ADC2_CHANNEL_8;
// constexpr adc2_channel_t BATTERY = ADC2_CHANNEL_9;

constexpr int16_t ENCODER_H_LIM_VAL = 32767;
constexpr int16_t ENCODER_L_LIM_VAL = -32767;

const int16_t ENC_RESOLUTION = 16384 - 1;
constexpr uint8_t READ_FLAG = 0x80;
constexpr uint16_t READ_FLAG2 = 0b01000000;
constexpr uint16_t PARITY_FLAG = 0b10000000;
constexpr uint8_t ESC = 0x1B;
constexpr uint16_t BUF_SIZE = 4096;

constexpr uint16_t MOTION_CHECK_TH = 1000;
constexpr uint16_t ENC_OPE_V_R_TH = 175;

constexpr uint16_t LOG_SIZE = 1300;
constexpr uint16_t LINE_BUF_SIZE = 512;
// constexpr float BATTERY_GAIN = 3.2075; // 2.97324;
constexpr float BATTERY_GAIN = 3.3; // 2.97324;

constexpr uint8_t LEDC_HIGH_SPEED_MODE = 0;
constexpr float LOW_BATTERY_TH = 7.5;

constexpr uint16_t RESET_GYRO_LOOP_CNT = 512;

static const std::initializer_list<std::pair<TurnType, std::string>>
    turn_name_list = {
        {TurnType::None, "straight"},    //
        {TurnType::Normal, "normal"},    //
        {TurnType::Large, "large"},      //
        {TurnType::Orval, "orval"},      //
        {TurnType::Dia45, "dia45"},      //
        {TurnType::Dia135, "dia135"},    //
        {TurnType::Dia90, "dia90"},      //
        {TurnType::Dia45_2, "dia45_2"},  //
        {TurnType::Dia135_2, "dia135_2"} //
};

static const std::initializer_list<std::pair<StraightType, std::string>>
    straight_name_list = {
        {StraightType::Search, "search"},          //
        {StraightType::FastRun, "fast_run"},       //
        {StraightType::FastRunDia, "fast_run_dia"} //
};

static const std::string slalom_log_file("/spiflash/sla.log");
static const std::string sysid_log_file("/spiflash/sysid.log");
static const std::string maze_log_file("/spiflash/maze.log");
static const std::string maze_log_kata_file("/spiflash/maze_kata.log");
static const std::string maze_log_return_file("/spiflash/maze_return.log");

static const std::string
    format1("%d,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,");
static const std::string format2("%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,");
static const std::string format3("%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0."
                                 "3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,"
                                 "%0.3f,%d,");
static const std::string format4("%0.3f,%0.3f,%0.3f,%0.3f,%d\n");
static const std::string
    formatsysid("%d,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f\n");
static char line_buf[LINE_BUF_SIZE];
// static const std::string format1("%d,%d,%d,%d,%d,%d,");
// static const std::string format2("%d,%d,%d,%d,%d,%d,%d,");
// static const std::string format3("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n");

std::shared_ptr<sensing_result_entity_t> get_sensing_entity();

#endif