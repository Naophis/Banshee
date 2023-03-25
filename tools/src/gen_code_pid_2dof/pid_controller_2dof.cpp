#include "pid_controller_2dof.h"
#include "rtwtypes.h"
#include "zero_crossing_types.h"

void PID_Controller_2dof::step(real32_T arg_tgt, real32_T arg_now, real32_T
  arg_p_gain, real32_T arg_i_gain, real32_T arg_d_gain, real32_T arg_b, real32_T
  arg_c, boolean_T arg_reset_req, real32_T arg_dt, real32_T *arg_out)
{
  real32_T rtb_UdiffTsProdOut;
  if (arg_reset_req || (pid_controller_2dof_DW.Integrator_PrevResetState != 0))
  {
    pid_controller_2dof_DW.Integrator_DSTATE =
      pid_controller_2dof_P.Constant_Value;
  }

  rtb_UdiffTsProdOut = (arg_tgt * arg_c - arg_now) * arg_d_gain / arg_dt;
  pid_controller_2dof_DW.icLoad = ((((pid_controller_2dof_PrevZCX.UD_Reset_ZCE ==
    POS_ZCSIG) != arg_reset_req) && (pid_controller_2dof_PrevZCX.UD_Reset_ZCE !=
    UNINITIALIZED_ZCSIG)) || arg_reset_req || pid_controller_2dof_DW.icLoad);
  pid_controller_2dof_PrevZCX.UD_Reset_ZCE = arg_reset_req;
  if (pid_controller_2dof_DW.icLoad) {
    pid_controller_2dof_DW.UD_DSTATE = pid_controller_2dof_P.Constant_Value;
  }

  *arg_out = ((arg_tgt * arg_b - arg_now) * arg_p_gain +
              pid_controller_2dof_DW.Integrator_DSTATE) + (rtb_UdiffTsProdOut -
    pid_controller_2dof_DW.UD_DSTATE);
  pid_controller_2dof_DW.Integrator_DSTATE += (arg_tgt - arg_now) * arg_i_gain *
    arg_dt * pid_controller_2dof_P.Integrator_gainval;
  pid_controller_2dof_DW.Integrator_PrevResetState = static_cast<int8_T>
    (arg_reset_req);
  pid_controller_2dof_DW.icLoad = false;
  pid_controller_2dof_DW.UD_DSTATE = rtb_UdiffTsProdOut;
}

void PID_Controller_2dof::initialize()
{
  pid_controller_2dof_PrevZCX.UD_Reset_ZCE = UNINITIALIZED_ZCSIG;
  pid_controller_2dof_DW.Integrator_DSTATE =
    pid_controller_2dof_P.Constant_Value;
  pid_controller_2dof_DW.icLoad = true;
}

void PID_Controller_2dof::terminate()
{
}

PID_Controller_2dof::PID_Controller_2dof() :
  pid_controller_2dof_DW(),
  pid_controller_2dof_PrevZCX(),
  pid_controller_2dof_M()
{
}

PID_Controller_2dof::~PID_Controller_2dof()
{
}

PID_Controller_2dof::RT_MODEL_pid_controller_2dof_T * PID_Controller_2dof::
  getRTM()
{
  return (&pid_controller_2dof_M);
}
