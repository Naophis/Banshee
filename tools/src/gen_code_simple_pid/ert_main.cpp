#include <stdio.h>
#include "simple_pid_controller.h"

static Simple_PID_Controller simple_pid_controller_Obj;
static real32_T arg_diff{ 0.0F };

static real32_T arg_p_gain{ 0.0F };

static real32_T arg_i_gain{ 0.0F };

static real32_T arg_d_gain{ 0.0F };

static boolean_T arg_reset_req{ false };

static real32_T arg_dt{ 0.0F };

static real32_T arg_out;
void rt_OneStep(void);
void rt_OneStep(void)
{
  static boolean_T OverrunFlag{ false };

  if (OverrunFlag) {
    rtmSetErrorStatus(simple_pid_controller_Obj.getRTM(), "Overrun");
    return;
  }

  OverrunFlag = true;
  simple_pid_controller_Obj.step(&arg_diff, &arg_p_gain, &arg_i_gain,
    &arg_d_gain, &arg_reset_req, &arg_dt, &arg_out);
  OverrunFlag = false;
}

int_T main(int_T argc, const char *argv[])
{
  (void)(argc);
  (void)(argv);
  simple_pid_controller_Obj.initialize();
  printf("Warning: The simulation will run forever. "
         "Generated ERT main won't simulate model step behavior. "
         "To change this behavior select the 'MAT-file logging' option.\n");
  fflush((nullptr));
  while (rtmGetErrorStatus(simple_pid_controller_Obj.getRTM()) == (nullptr)) {
  }

  simple_pid_controller_Obj.terminate();
  return 0;
}
