#ifndef HALF_TYPE_H
#define HALF_TYPE_H
#include "rtwtypes.h"

uint32_T getBitfieldFromFloat(real32_T a);
real32_T getFloatFromBitfield(uint32_T a);
typedef struct {
  uint16_T bitPattern;
} half_t;

typedef half_t real16_T;
typedef struct {
  real16_T re;
  real16_T im;
} creal16_T;

uint16_T getBitfieldFromHalf(real16_T a);
real16_T getHalfFromBitfield(uint16_T a);
real32_T halfToFloat(real16_T a);
real64_T halfToDouble(real16_T a);
real16_T floatToHalf(real32_T a);
real16_T doubleToHalf(real64_T a);

#endif
