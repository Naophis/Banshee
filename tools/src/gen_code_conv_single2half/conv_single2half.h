#ifndef RTW_HEADER_conv_single2half_h_
#define RTW_HEADER_conv_single2half_h_
#ifndef conv_single2half_COMMON_INCLUDES_
#define conv_single2half_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif

#include "conv_single2half_types.h"
#include "half_type.h"

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

typedef struct {
  real32_T from_single;
  real16_T from_half;
} ExtU_conv_single2half_T;

typedef struct {
  real16_T to_half;
  real32_T to_single;
} ExtY_conv_single2half_T;

struct tag_RTM_conv_single2half_T {
  const char_T * volatile errorStatus;
};

extern ExtU_conv_single2half_T conv_single2half_U;
extern ExtY_conv_single2half_T conv_single2half_Y;
extern void conv_single2half_initialize(void);
extern void conv_single2half_step(void);
extern void conv_single2half_terminate(void);
extern RT_MODEL_conv_single2half_T *const conv_single2half_M;

#endif

