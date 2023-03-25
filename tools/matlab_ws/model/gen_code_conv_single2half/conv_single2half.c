#include "conv_single2half.h"
#include "half_type.h"
#include "rtwtypes.h"

ExtU_conv_single2half_T conv_single2half_U;
ExtY_conv_single2half_T conv_single2half_Y;
static RT_MODEL_conv_single2half_T conv_single2half_M_;
RT_MODEL_conv_single2half_T *const conv_single2half_M = &conv_single2half_M_;
void conv_single2half_step(void)
{
  conv_single2half_Y.to_half = floatToHalf(conv_single2half_U.from_single);
  conv_single2half_Y.to_single = halfToFloat(conv_single2half_U.from_half);
}

void conv_single2half_initialize(void)
{
}

void conv_single2half_terminate(void)
{
}
