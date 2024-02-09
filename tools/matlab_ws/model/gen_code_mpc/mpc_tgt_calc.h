#ifndef RTW_HEADER_mpc_tgt_calc_h_
#define RTW_HEADER_mpc_tgt_calc_h_
#include "rtwtypes.h"
#include "bus.h"
#include "mpc_tgt_calc_types.h"

extern "C"
{

#include "rt_nonfinite.h"

}

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#include "bus.h"

class mpc_tgt_calcModelClass final
{
 public:
  struct B_mpc_tgt_calc_T {
    real32_T Merge_p[2];
    real32_T Merge1[2];
  };

  struct P_IfActionSubsystem_mpc_tgt_c_T {
    real32_T Constant1_Value;
    real32_T Constant2_Value;
  };

  struct P_keep_mpc_tgt_calc_T {
    real_T Constant_Value;
    int32_T Constant2_Value;
  };

  struct P_mpc_tgt_calc_T {
    real32_T dt;
    real_T Constant1_Value;
    real_T Constant1_Value_l;
    real_T Gain_Gain;
    real_T Constant_Value;
    real_T Constant_Value_n;
    real_T Constant1_Value_m;
    real_T Gain_Gain_g;
    real_T Constant_Value_p;
    real_T Constant_Value_g;
    real_T Constant1_Value_k;
    real_T Constant3_Value;
    real_T Gain1_Gain;
    real_T Constant_Value_nl;
    real_T Constant_Value_c;
    real_T Constant3_Value_a;
    real_T Constant1_Value_a;
    real_T Constant2_Value;
    real_T Constant1_Value_h;
    real_T Gain_Gain_i;
    real_T Gain1_Gain_c;
    real_T Constant_Value_cx;
    real_T Constant_Value_cf;
    int32_T Constant2_Value_g;
    int32_T Constant1_Value_p;
    int32_T Constant2_Value_f;
    int32_T Constant2_Value_g3;
    int32_T Constant1_Value_c;
    int32_T Constant2_Value_f2;
    int32_T Constant1_Value_o;
    int32_T Constant2_Value_j;
    int32_T Constant1_Value_e;
    int32_T Constant2_Value_p;
    int32_T Constant1_Value_lj;
    int32_T Constant2_Value_gw;
    int32_T Constant2_Value_i;
    int32_T Constant1_Value_kv;
    int32_T Constant2_Value_d;
    int32_T Constant_Value_k;
    int32_T Constant3_Value_i;
    int32_T Constant1_Value_a0;
    int32_T Constant2_Value_h;
    int32_T DataStoreMemory_InitialValue;
    int32_T DataStoreMemory1_InitialValue;
    real32_T Constant4_Value;
    real32_T Constant6_Value;
    real32_T Constant_Value_ne;
    real32_T Constant1_Value_mn;
    real32_T Constant5_Value;
    real32_T Constant10_Value;
    real32_T Constant2_Value_e;
    real32_T Constant3_Value_j;
    real32_T Constant7_Value;
    real32_T Constant8_Value;
    real32_T Gain1_Gain_l;
    real32_T Constant4_Value_d;
    real32_T Constant6_Value_o;
    real32_T Constant_Value_f;
    real32_T Constant1_Value_i;
    real32_T Constant5_Value_j;
    real32_T Constant10_Value_f;
    real32_T Constant2_Value_m;
    real32_T Constant3_Value_k;
    real32_T Constant7_Value_a;
    real32_T Constant8_Value_k;
    real32_T Gain1_Gain_d;
    real32_T Constant_Value_j;
    real32_T Merge_InitialOutput;
    real32_T Merge1_InitialOutput;
    real32_T Gain1_Gain_h;
    real32_T Switch1_Threshold;
    real32_T Constant_Value_ph;
    real32_T Switch2_Threshold;
    real32_T Constant4_Value_i;
    real32_T Constant3_Value_f;
    real32_T Gain3_Gain;
    real32_T Gain2_Gain;
    real32_T Gain1_Gain_m;
    real32_T Switch_Threshold;
    real32_T Constant_Value_pr;
    real32_T Gain1_Gain_n;
    real32_T Switch1_Threshold_i;
    real32_T Constant_Value_h;
    real32_T Switch2_Threshold_m;
    real32_T Gain6_Gain;
    real32_T Gain7_Gain;
    real32_T Gain4_Gain;
    real32_T Gain2_Gain_i;
    real32_T Gain1_Gain_hy;
    real32_T Gain_Gain_m;
    real32_T Constant_Value_m;
    real32_T Constant_Value_e;
    real32_T Constant_Value_o;
    real32_T Gain3_Gain_d;
    real32_T Constant1_Value_ih;
    real32_T Gain1_Gain_o;
    real32_T Constant_Value_nu;
    real32_T Gain1_Gain_i;
    real32_T Gain_Gain_h;
    real32_T Gain_Gain_n;
    real32_T Constant3_Value_c;
    real32_T Gain1_Gain_o0;
    real32_T Merge_InitialOutput_o;
    real32_T Constant_Value_nex;
    real32_T Constant4_Value_c;
    real32_T Constant3_Value_cn;
    real32_T Gain1_Gain_nz;
    real32_T Merge_InitialOutput_m;
    real32_T Constant_Value_hs;
    real32_T Constant1_Value_e1;
    real32_T Constant4_Value_l;
    real32_T Constant_Value_h5;
    real32_T Gain2_Gain_g;
    real32_T Gain4_Gain_c;
    real32_T Gain1_Gain_b;
    real32_T Gain1_Gain_it;
    real32_T Gain2_Gain_j;
    real32_T Gain4_Gain_m;
    real32_T Gain5_Gain;
    real32_T Gain_Gain_k;
    real32_T Gain2_Gain_d;
    real32_T Gain6_Gain_f;
    real32_T Gain1_Gain_i5;
    real32_T Gain3_Gain_k;
    uint8_T ManualSwitch_CurrentSetting;
    uint8_T ManualSwitch_CurrentSetting_o;
    uint8_T ManualSwitch_CurrentSetting_c;
    uint8_T ManualSwitch_CurrentSetting_i;
    uint8_T ManualSwitch_CurrentSetting_e;
    uint8_T ManualSwitch_CurrentSetting_b;
    P_keep_mpc_tgt_calc_T keep_p;
    P_keep_mpc_tgt_calc_T keep_h;
    P_IfActionSubsystem_mpc_tgt_c_T IfActionSubsystem2;
    P_IfActionSubsystem_mpc_tgt_c_T IfActionSubsystem_g;
  };

  struct RT_MODEL_mpc_tgt_calc_T {
    const char_T * volatile errorStatus;
  };

  mpc_tgt_calcModelClass(mpc_tgt_calcModelClass const&) = delete;
  mpc_tgt_calcModelClass& operator= (mpc_tgt_calcModelClass const&) & = delete;
  mpc_tgt_calcModelClass(mpc_tgt_calcModelClass &&) = delete;
  mpc_tgt_calcModelClass& operator= (mpc_tgt_calcModelClass &&) = delete;
  mpc_tgt_calcModelClass::RT_MODEL_mpc_tgt_calc_T * getRTM();
  void initialize();
  void step(const t_tgt *arg_tgt, const t_ego *arg_ego, int32_T arg_mode,
            int32_T arg_time_step, t_ego *arg_next_ego, t_dynamics *arg_ego1);
  static void terminate();
  mpc_tgt_calcModelClass();
  ~mpc_tgt_calcModelClass();
 private:
  B_mpc_tgt_calc_T mpc_tgt_calc_B;
  static P_mpc_tgt_calc_T mpc_tgt_calc_P;
  static void mpc_tgt_calc_IfActionSubsystem(real32_T rty_Out1[2],
    P_IfActionSubsystem_mpc_tgt_c_T *localP);
  static void mpc_tgt_calc_keep(real32_T *rty_accl_out, int32_T *rty_state_out,
    P_keep_mpc_tgt_calc_T *localP);
  RT_MODEL_mpc_tgt_calc_T mpc_tgt_calc_M;
};

#endif

