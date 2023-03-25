#include "pid_controller.h"
#include "rtwtypes.h"
#include "zero_crossing_types.h"

void PID_Controller::step(const real32_T *arg_diff, const real32_T *arg_p_gain,
  const real32_T *arg_i_gain, const real32_T *arg_d_gain, const boolean_T
  *arg_reset_req, const real32_T *arg_dt, real32_T *arg_out)
{
  real32_T rtb_UdiffTsProdOut;
  pid_controller_DW.icLoad = ((((pid_controller_PrevZCX.UD_Reset_ZCE ==
    POS_ZCSIG) != *arg_reset_req) && (pid_controller_PrevZCX.UD_Reset_ZCE !=
    UNINITIALIZED_ZCSIG)) || pid_controller_DW.icLoad);
  pid_controller_PrevZCX.UD_Reset_ZCE = *arg_reset_req;
  if (pid_controller_DW.icLoad) {
    pid_controller_DW.UD_DSTATE = pid_controller_P.Constant_Value;
  }

  rtb_UdiffTsProdOut = *arg_diff * *arg_d_gain / *arg_dt;
  if ((*arg_reset_req) == 0) {
    pid_controller_DW.Integrator_DSTATE = pid_controller_P.Constant_Value;
    pid_controller_DW.UD_DSTATE = pid_controller_P.Constant_Value;
  }

  *arg_out = ((*arg_diff + pid_controller_DW.Integrator_DSTATE) +
              (rtb_UdiffTsProdOut - pid_controller_DW.UD_DSTATE)) * *arg_p_gain;
  pid_controller_DW.icLoad = false;
  pid_controller_DW.UD_DSTATE = rtb_UdiffTsProdOut;
  pid_controller_DW.Integrator_DSTATE += *arg_diff * *arg_i_gain * *arg_dt *
    pid_controller_P.Integrator_gainval;
  pid_controller_DW.Integrator_PrevResetState = static_cast<int8_T>
    (*arg_reset_req);
}

void PID_Controller::initialize()
{
  pid_controller_PrevZCX.UD_Reset_ZCE = UNINITIALIZED_ZCSIG;
  pid_controller_DW.icLoad = true;
  pid_controller_DW.Integrator_DSTATE = pid_controller_P.Constant_Value;
  pid_controller_DW.Integrator_PrevResetState = 2;
}

void PID_Controller::terminate()
{
}

PID_Controller::PID_Controller() :
  pid_controller_DW(),
  pid_controller_PrevZCX(),
  pid_controller_M()
{
}

PID_Controller::~PID_Controller()
{
}

PID_Controller::RT_MODEL_pid_controller_T * PID_Controller::getRTM()
{
  return (&pid_controller_M);
}
