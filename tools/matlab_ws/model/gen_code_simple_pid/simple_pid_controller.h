#ifndef RTW_HEADER_simple_pid_controller_h_
#define RTW_HEADER_simple_pid_controller_h_
#include "rtwtypes.h"
#include "simple_pid_controller_types.h"
#include "zero_crossing_types.h"

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

class Simple_PID_Controller final
{
 public:
  struct DW_simple_pid_controller_T {
    real32_T Integrator_DSTATE;
    real32_T UD_DSTATE;
    int8_T Integrator_PrevResetState;
    boolean_T icLoad;
  };

  struct PrevZCX_simple_pid_controller_T {
    ZCSigState UD_Reset_ZCE;
  };

  struct P_simple_pid_controller_T {
    real32_T Constant_Value;
    real32_T Integrator_gainval;
  };

  struct RT_MODEL_simple_pid_controlle_T {
    const char_T * volatile errorStatus;
  };

  Simple_PID_Controller(Simple_PID_Controller const&) = delete;
  Simple_PID_Controller& operator= (Simple_PID_Controller const&) & = delete;
  Simple_PID_Controller(Simple_PID_Controller &&) = delete;
  Simple_PID_Controller& operator= (Simple_PID_Controller &&) = delete;
  Simple_PID_Controller::RT_MODEL_simple_pid_controlle_T * getRTM();
  void initialize();
  void step(const real32_T *arg_diff, const real32_T *arg_p_gain, const real32_T
            *arg_i_gain, const real32_T *arg_d_gain, const boolean_T
            *arg_reset_req, const real32_T *arg_dt, real32_T *arg_out);
  static void terminate();
  Simple_PID_Controller();
  ~Simple_PID_Controller();
 private:
  DW_simple_pid_controller_T simple_pid_controller_DW;
  static P_simple_pid_controller_T simple_pid_controller_P;
  PrevZCX_simple_pid_controller_T simple_pid_controller_PrevZCX;
  RT_MODEL_simple_pid_controlle_T simple_pid_controller_M;
};

#endif

