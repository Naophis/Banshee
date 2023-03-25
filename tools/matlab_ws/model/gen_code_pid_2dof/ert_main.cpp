#include <stdio.h>
#include "pid_controller_2dof.h"

static PID_Controller_2dof pid_controller_2dof_Obj;
static real32_T arg_tgt{ 0.0F };

static real32_T arg_now{ 0.0F };

static real32_T arg_p_gain{ 0.0F };

static real32_T arg_i_gain{ 0.0F };

static real32_T arg_d_gain{ 0.0F };

static real32_T arg_b{ 0.0F };

static real32_T arg_c{ 0.0F };

static boolean_T arg_reset_req{ false };

static real32_T arg_dt{ 0.0F };

static real32_T arg_out;
void rt_OneStep(void);
void rt_OneStep(void)
{
  static boolean_T OverrunFlag{ false };

  if (OverrunFlag) {
    rtmSetErrorStatus(pid_controller_2dof_Obj.getRTM(), "Overrun");
    return;
  }

  OverrunFlag = true;
  pid_controller_2dof_Obj.step(arg_tgt, arg_now, arg_p_gain, arg_i_gain,
    arg_d_gain, arg_b, arg_c, arg_reset_req, arg_dt, &arg_out);
  OverrunFlag = false;
}

int_T main(int_T argc, const char *argv[])
{
  (void)(argc);
  (void)(argv);
  pid_controller_2dof_Obj.initialize();
  printf("Warning: The simulation will run forever. "
         "Generated ERT main won't simulate model step behavior. "
         "To change this behavior select the 'MAT-file logging' option.\n");
  fflush((nullptr));
  while (rtmGetErrorStatus(pid_controller_2dof_Obj.getRTM()) == (nullptr)) {
  }

  pid_controller_2dof_Obj.terminate();
  return 0;
}
