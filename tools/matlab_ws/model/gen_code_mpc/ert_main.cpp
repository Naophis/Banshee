#include <stdio.h>
#include "mpc_tgt_calc.h"

static mpc_tgt_calcModelClass mpc_tgt_calc_Obj;
static t_tgt arg_tgt{
  0.0F,
  0.0F,
  0.0F,
  0.0F,
  0.0F,
  0.0F,
  0.0F,
  0.0F,
  0.0F,
  0,

  {
    0.0F,
    0.0F,
    0.0F
  },

  {
    0.0F,
    0.0F
  },
  0.0F,
  0.0F,
  0.0F,
  0.0F,
  0.0F,
  0,
  0.0F
};

static t_ego arg_ego{
  0.0F,
  0.0F,
  0.0F,
  0.0F,
  0.0F,
  0.0F,
  0.0F,
  0.0F,
  0.0F,

  {
    0.0F,
    0.0F,
    0.0F,
    0.0F,
    0,
    0
  },
  0,
  0,

  {
    0.0F,
    0.0F,
    0.0F,
    0.0F,
    0.0F,
    0.0F
  },

  {
    0.0F,
    0.0F,
    0.0F,
    0.0F,
    0.0F,
    0.0F
  },

  {
    0.0F,
    0.0F,
    0.0F,
    0.0F,
    0.0F
  },

  {
    0.0F,
    0.0F,
    0.0F
  },
  0.0F,
  0.0F,
  0.0F,
  0.0F,

  {
    0.0F,
    0.0F,
    0.0F,
    0.0F,
    0.0F
  },
  0.0F,
  0.0F,
  0.0F,
  0.0F
};

static int32_T arg_mode{ 0 };

static int32_T arg_time_step{ 0 };

static t_dynamics arg_ego1{
  0.0F,
  0.0F,
  0.0F,
  0.0F,
  0.0F,
  0.0F,
  0.0F,
  0.0F
};

static t_ego arg_next_ego;
void rt_OneStep(void);
void rt_OneStep(void)
{
  static boolean_T OverrunFlag{ false };

  if (OverrunFlag) {
    rtmSetErrorStatus(mpc_tgt_calc_Obj.getRTM(), "Overrun");
    return;
  }

  OverrunFlag = true;
  mpc_tgt_calc_Obj.step(&arg_tgt, &arg_ego, arg_mode, arg_time_step,
                        &arg_next_ego, &arg_ego1);
  OverrunFlag = false;
}

int_T main(int_T argc, const char *argv[])
{
  (void)(argc);
  (void)(argv);
  mpc_tgt_calc_Obj.initialize();
  printf("Warning: The simulation will run forever. "
         "Generated ERT main won't simulate model step behavior. "
         "To change this behavior select the 'MAT-file logging' option.\n");
  fflush((nullptr));
  while (rtmGetErrorStatus(mpc_tgt_calc_Obj.getRTM()) == (nullptr)) {
  }

  mpc_tgt_calc_Obj.terminate();
  return 0;
}
