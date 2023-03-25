#include "simple_pid_controller.h"
#include "rtwtypes.h"
#include "zero_crossing_types.h"

void Simple_PID_Controller::step(const real32_T *arg_diff, const real32_T
  *arg_p_gain, const real32_T *arg_i_gain, const real32_T *arg_d_gain, const
  boolean_T *arg_reset_req, const real32_T *arg_dt, real32_T *arg_out)
{
  real32_T rtb_UdiffTsProdOut;
  if (((*arg_reset_req) && (simple_pid_controller_DW.Integrator_PrevResetState <=
        0)) || ((!*arg_reset_req) &&
                (simple_pid_controller_DW.Integrator_PrevResetState == 1))) {
    simple_pid_controller_DW.Integrator_DSTATE =
      simple_pid_controller_P.Constant_Value;
  }

  rtb_UdiffTsProdOut = *arg_diff * *arg_d_gain / *arg_dt;
  simple_pid_controller_DW.icLoad =
    ((((simple_pid_controller_PrevZCX.UD_Reset_ZCE == POS_ZCSIG) !=
       *arg_reset_req) && (simple_pid_controller_PrevZCX.UD_Reset_ZCE !=
       UNINITIALIZED_ZCSIG)) || simple_pid_controller_DW.icLoad);
  simple_pid_controller_PrevZCX.UD_Reset_ZCE = *arg_reset_req;
  if (simple_pid_controller_DW.icLoad) {
    simple_pid_controller_DW.UD_DSTATE = simple_pid_controller_P.Constant_Value;
  }

  *arg_out = (*arg_diff * *arg_p_gain +
              simple_pid_controller_DW.Integrator_DSTATE) + (rtb_UdiffTsProdOut
    - simple_pid_controller_DW.UD_DSTATE);
  simple_pid_controller_DW.Integrator_DSTATE += *arg_diff * *arg_i_gain *
    *arg_dt * simple_pid_controller_P.Integrator_gainval;
  simple_pid_controller_DW.Integrator_PrevResetState = static_cast<int8_T>
    (*arg_reset_req);
  simple_pid_controller_DW.icLoad = false;
  simple_pid_controller_DW.UD_DSTATE = rtb_UdiffTsProdOut;
}

void Simple_PID_Controller::initialize()
{
  {
    real32_T Constant;
    Constant = simple_pid_controller_P.Constant_Value;
    simple_pid_controller_PrevZCX.UD_Reset_ZCE = UNINITIALIZED_ZCSIG;
    simple_pid_controller_DW.Integrator_DSTATE = Constant;
    simple_pid_controller_DW.Integrator_PrevResetState = 2;
    simple_pid_controller_DW.icLoad = true;
  }
}

void Simple_PID_Controller::terminate()
{
}

Simple_PID_Controller::Simple_PID_Controller() :
  simple_pid_controller_DW(),
  simple_pid_controller_PrevZCX(),
  simple_pid_controller_M()
{
}

Simple_PID_Controller::~Simple_PID_Controller()
{
}

Simple_PID_Controller::RT_MODEL_simple_pid_controlle_T * Simple_PID_Controller::
  getRTM()
{
  return (&simple_pid_controller_M);
}
