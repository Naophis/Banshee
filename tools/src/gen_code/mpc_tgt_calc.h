#ifndef RTW_HEADER_mpc_tgt_calc_h_
#define RTW_HEADER_mpc_tgt_calc_h_
#include <cmath>
#include "rtwtypes.h"
#include "mpc_tgt_calc_types.h"
#include "rt_nonfinite.h"

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#include "bus.h"

class mpc_tgt_calcModelClass {
 public:
  struct P_keep_mpc_tgt_calc_T {
    real_T Constant_Value;
    int32_T Constant2_Value;
  };

  struct P_mpc_tgt_calc_T {
    real_T Constant1_Value;
    real_T Constant1_Value_d;
    real_T Gain_Gain;
    real_T Constant1_Value_l;
    real_T Gain_Gain_j;
    real_T Constant_Value;
    real_T Constant_Value_n;
    real_T Constant1_Value_m;
    real_T Gain_Gain_g;
    real_T Constant_Value_p;
    real_T Constant1_Value_o;
    real_T Gain_Gain_n;
    real_T Gain1_Gain;
    real_T Constant_Value_nl;
    real_T Constant_Value_c;
    real_T Constant1_Value_h;
    real_T Gain_Gain_i;
    real_T Gain1_Gain_c;
    real_T Constant_Value_cx;
    real_T Constant_Value_cf;
    int32_T Constant2_Value;
    int32_T Constant1_Value_p;
    int32_T Constant2_Value_f;
    int32_T Constant2_Value_g;
    int32_T Constant1_Value_c;
    int32_T Constant2_Value_f2;
    int32_T Constant1_Value_oj;
    int32_T Constant2_Value_j;
    int32_T Constant1_Value_e;
    int32_T Constant2_Value_p;
    int32_T Constant1_Value_lj;
    int32_T Constant2_Value_gw;
    int32_T Constant2_Value_i;
    int32_T Constant1_Value_k;
    int32_T Constant2_Value_d;
    real32_T Constant_Value_j;
    real32_T Constant5_Value;
    real32_T Constant1_Value_mn;
    real32_T Constant_Value_ne;
    real32_T Gain_Gain_e;
    real32_T Constant6_Value;
    real32_T Constant4_Value;
    real32_T Constant2_Value_e;
    real32_T Gain1_Gain_l;
    real32_T Gain1_Gain_h;
    real32_T Switch1_Threshold;
    real32_T Constant_Value_ph;
    real32_T Switch2_Threshold;
    real32_T Constant4_Value_i;
    real32_T Constant3_Value;
    real32_T Gain3_Gain;
    real32_T Gain2_Gain;
    real32_T Gain1_Gain_m;
    real32_T Switch_Threshold;
    real32_T Gain_Gain_d;
    real32_T Constant_Value_pr;
    real32_T Gain1_Gain_n;
    real32_T Switch1_Threshold_i;
    real32_T Constant_Value_h;
    real32_T Switch2_Threshold_m;
    real32_T Gain2_Gain_i;
    real32_T Gain1_Gain_hy;
    real32_T Gain_Gain_m;
    real32_T Gain4_Gain;
    real32_T Gain_Gain_c;
    real32_T Constant_Value_m;
    real32_T Constant_Value_e;
    real32_T Gain_Gain_b;
    real32_T Constant3_Value_c;
    real32_T Gain1_Gain_o;
    real32_T Merge_InitialOutput;
    real32_T Gain_Gain_ca;
    real32_T Constant_Value_nex;
    real32_T Constant4_Value_c;
    real32_T Constant3_Value_cn;
    real32_T Gain1_Gain_nz;
    real32_T Merge_InitialOutput_m;
    real32_T Gain_Gain_k;
    real32_T Constant_Value_hs;
    real32_T Constant1_Value_i;
    real32_T Constant_Value_hn;
    real32_T Saturation_UpperSat;
    real32_T Saturation_LowerSat;
    real32_T Constant1_Value_op;
    real32_T Constant_Value_my;
    real32_T Saturation_UpperSat_i;
    real32_T Saturation_LowerSat_e;
    real32_T mms2ms_Gain;
    real32_T Constant1_Value_hj;
    real32_T Constant_Value_g;
    real32_T Constant3_Value_a;
    real32_T Gain1_Gain_e;
    real32_T Gain1_Gain_g;
    real32_T Gain3_Gain_m;
    real32_T Gain4_Gain_o;
    real32_T Gain1_Gain_cq;
    real32_T Gain2_Gain_c;
    real32_T Gain5_Gain;
    real32_T Gain6_Gain;
    real32_T Gain_Gain_kq;
    real32_T DataStoreMemory_InitialValue;
    uint8_T ManualSwitch_CurrentSetting;
    uint8_T ManualSwitch_CurrentSetting_c;
    uint8_T ManualSwitch_CurrentSetting_j;
    uint8_T ManualSwitch_CurrentSetting_i;
    uint8_T ManualSwitch_CurrentSetting_e;
    uint8_T ManualSwitch_CurrentSetting_b;
    P_keep_mpc_tgt_calc_T keep_p;
    P_keep_mpc_tgt_calc_T keep_h;
  };

  struct RT_MODEL_mpc_tgt_calc_T {
    const char_T * volatile errorStatus;
  };

  typedef struct {
    P_keep_mpc_tgt_calc_T* defaultParam;
  } self_keep_mpc_tgt_calc_T;

  void initialize();
  void step(const t_tgt *arg_tgt, const t_ego *arg_ego, int32_T arg_mode,
            int32_T arg_time_step, t_ego *arg_next_ego);
  void terminate();
  mpc_tgt_calcModelClass();
  ~mpc_tgt_calcModelClass();
  mpc_tgt_calcModelClass::RT_MODEL_mpc_tgt_calc_T * getRTM();
 private:
  static P_mpc_tgt_calc_T mpc_tgt_calc_P;
  self_keep_mpc_tgt_calc_T self_keep_p;
  self_keep_mpc_tgt_calc_T self_keep_h;
  RT_MODEL_mpc_tgt_calc_T mpc_tgt_calc_M;
  static void mpc_tgt_calc_keep(self_keep_mpc_tgt_calc_T *mpc_tgt_calc_self_arg,
    real32_T *rty_accl_out, int32_T *rty_state_out);
};

#endif

