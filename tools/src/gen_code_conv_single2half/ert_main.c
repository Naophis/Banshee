#include <stddef.h>
#include <stdio.h>
#include "conv_single2half.h"

void rt_OneStep(void);
void rt_OneStep(void)
{
  static boolean_T OverrunFlag = false;
  if (OverrunFlag) {
    rtmSetErrorStatus(conv_single2half_M, "Overrun");
    return;
  }

  OverrunFlag = true;
  conv_single2half_step();
  OverrunFlag = false;
}

int_T main(int_T argc, const char *argv[])
{
  (void)(argc);
  (void)(argv);
  conv_single2half_initialize();
  printf("Warning: The simulation will run forever. "
         "Generated ERT main won't simulate model step behavior. "
         "To change this behavior select the 'MAT-file logging' option.\n");
  fflush((NULL));
  while (rtmGetErrorStatus(conv_single2half_M) == (NULL)) {
  }

  conv_single2half_terminate();
  return 0;
}
