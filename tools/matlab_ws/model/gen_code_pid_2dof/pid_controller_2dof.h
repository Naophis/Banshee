#ifndef RTW_HEADER_pid_controller_2dof_h_
#define RTW_HEADER_pid_controller_2dof_h_
#include "rtwtypes.h"
#include "pid_controller_2dof_types.h"
#include "zero_crossing_types.h"

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

class PID_Controller_2dof final
{
 public:
  struct DW_pid_controller_2dof_T {
    real32_T Integrator_DSTATE;
    real32_T UD_DSTATE;
    int8_T Integrator_PrevResetState;
    boolean_T icLoad;
  };

  struct PrevZCX_pid_controller_2dof_T {
    ZCSigState UD_Reset_ZCE;
  };

  struct P_pid_controller_2dof_T {
    real32_T Constant_Value;
    real32_T Integrator_gainval;
  };

  struct RT_MODEL_pid_controller_2dof_T {
    const char_T * volatile errorStatus;
  };

  PID_Controller_2dof(PID_Controller_2dof const&) = delete;
  PID_Controller_2dof& operator= (PID_Controller_2dof const&) & = delete;
  PID_Controller_2dof(PID_Controller_2dof &&) = delete;
  PID_Controller_2dof& operator= (PID_Controller_2dof &&) = delete;
  PID_Controller_2dof::RT_MODEL_pid_controller_2dof_T * getRTM();
  void initialize();
  void step(real32_T arg_tgt, real32_T arg_now, real32_T arg_p_gain, real32_T
            arg_i_gain, real32_T arg_d_gain, real32_T arg_b, real32_T arg_c,
            boolean_T arg_reset_req, real32_T arg_dt, real32_T *arg_out);
  static void terminate();
  PID_Controller_2dof();
  ~PID_Controller_2dof();
 private:
  DW_pid_controller_2dof_T pid_controller_2dof_DW;
  static P_pid_controller_2dof_T pid_controller_2dof_P;
  PrevZCX_pid_controller_2dof_T pid_controller_2dof_PrevZCX;
  RT_MODEL_pid_controller_2dof_T pid_controller_2dof_M;
};

#endif

