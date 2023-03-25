#ifndef RTW_HEADER_pid_controller_h_
#define RTW_HEADER_pid_controller_h_
#include "rtwtypes.h"
#include "pid_controller_types.h"
#include "zero_crossing_types.h"

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

class PID_Controller final
{
 public:
  struct DW_pid_controller_T {
    real32_T UD_DSTATE;
    real32_T Integrator_DSTATE;
    int8_T Integrator_PrevResetState;
    boolean_T icLoad;
  };

  struct PrevZCX_pid_controller_T {
    ZCSigState UD_Reset_ZCE;
  };

  struct P_pid_controller_T {
    real32_T Constant_Value;
    real32_T Integrator_gainval;
  };

  struct RT_MODEL_pid_controller_T {
    const char_T * volatile errorStatus;
  };

  PID_Controller(PID_Controller const&) = delete;
  PID_Controller& operator= (PID_Controller const&) & = delete;
  PID_Controller(PID_Controller &&) = delete;
  PID_Controller& operator= (PID_Controller &&) = delete;
  PID_Controller::RT_MODEL_pid_controller_T * getRTM();
  void initialize();
  void step(const real32_T *arg_diff, const real32_T *arg_p_gain, const real32_T
            *arg_i_gain, const real32_T *arg_d_gain, const boolean_T
            *arg_reset_req, const real32_T *arg_dt, real32_T *arg_out);
  static void terminate();
  PID_Controller();
  ~PID_Controller();
 private:
  DW_pid_controller_T pid_controller_DW;
  static P_pid_controller_T pid_controller_P;
  PrevZCX_pid_controller_T pid_controller_PrevZCX;
  RT_MODEL_pid_controller_T pid_controller_M;
};

#endif

