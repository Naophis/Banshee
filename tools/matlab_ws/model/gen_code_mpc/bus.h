#ifndef BUS_H
#define BUS_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
  float x;
  float y;
  float w;
  float theta;
  float dist;
} t_trajectory_point;

typedef struct {
  float x;
  float y;
  float theta;
  float v;
  float w;
} t_kanayama_tgt_point;

typedef struct {
  float x;
  float y;
  float theta;
} t_trajectory_diff;

typedef struct {
  float k_x;
  float k_y;
  float k_theta;
} t_kanayama_gain;

#define TRAJECTORY_POINT_SIZE 15

typedef struct {
  float limit;
  float n;
} t_accl_param;

typedef struct {
  float v_max;
  float end_v;
  float accl;
  float decel;
  float w_max;
  float end_w;
  float alpha;
  float tgt_dist;
  float tgt_angle;
  // t_trajectory_point trajectory_point[TRAJECTORY_POINT_SIZE];
  int trajectory_point_size;
  t_kanayama_gain kanayama_gain;
  t_accl_param accl_param;
  float slip_gain;
  float limit_accl_ratio_cnt;
  float limit_decel_ratio_cnt;
  float slip_gain_K1;
  float slip_gain_K2;
  float time_step2;
  float axel_degenerate_gain;
} t_tgt;

typedef struct{
  float mass;
  float lm;
  float km;
  float resist;
  float tread;
  float ke;
  float tire;
  float gear_ratio;
} t_dynamics;

typedef struct {
  float base_alpha;
  float base_time;
  float limit_time_count;
  float pow_n;
  int state;
  int counter;
} t_slalom;

typedef struct {
  float x;
  float y;
  float theta;
  float v;
  float w;
  float slip_angle;
} t_point;

typedef struct {
  float beta;
  float vx;
  float vy;
  float v;
  float accl;
} t_slip;

typedef struct {
  float v;
  float accl;
  float w;
  float alpha;
  float alpha2;
  float dist;
  float ang;
  float img_dist;
  float img_ang;
  t_slalom sla_param;
  int state;
  int pivot_state;
  t_point ideal_point;
  t_point slip_point;
  t_kanayama_tgt_point kanayama_point;
  t_trajectory_diff trj_diff;
  float delay_accl;
  float delay_v;
  float cnt_delay_accl_ratio;
  float cnt_delay_decel_ratio;
  t_slip slip;
  float ff_duty_l;
  float ff_duty_r;
  float ff_duty_low_th;
  float ff_duty_low_v_th;
} t_ego;

typedef struct {
  float v;
  float accl;
  float w;
  float alpha;
} t_mpc_out;

#define MPC_SIZE 5
typedef struct {
  t_mpc_out next_state[MPC_SIZE];
} t_mpc_out_list;

typedef enum { START_WAITING = 0, END_WAITING = 1 } WALL_OFF_MODE;

typedef struct {
  int state;
  float Front;
  float L45;
  float R45;
  float L90;
  float R90;
  int turn_mode;
  int is_dia;
  float front_dist;
} t_wall_sensor_input;

typedef struct {
  float th_Front;
  float th_L45;
  float th_R45;
  float th_L90;
  float th_R90;
} t_wall_off_th;

#endif
