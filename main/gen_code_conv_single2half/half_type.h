#ifndef HALF_TYPE_H
#define HALF_TYPE_H
#include "rtwtypes.h"

extern uint32_T getBitfieldFromFloat(real32_T a);
extern real32_T getFloatFromBitfield(uint32_T a);
typedef struct {
  uint16_T bitPattern;
} half_t;

typedef half_t real16_T;
typedef struct {
  real16_T re;
  real16_T im;
} creal16_T;
extern real32_T halfToFloat(real16_T a);
extern real64_T halfToDouble(real16_T a);
extern real16_T floatToHalf(real32_T a);
extern real16_T doubleToHalf(real64_T a);

#endif
