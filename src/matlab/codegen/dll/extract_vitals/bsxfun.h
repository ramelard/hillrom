/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: bsxfun.h
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 01-Mar-2018 16:31:19
 */

#ifndef BSXFUN_H
#define BSXFUN_H

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "extract_vitals_types.h"

/* Function Declarations */
#ifdef __cplusplus

extern "C" {

#endif

  extern void b_bsxfun(const emxArray_real_T *a, const emxArray_real_T *b,
                       emxArray_real_T *c);
  extern void bsxfun(const emxArray_int32_T *a, const emxArray_int32_T *b,
                     emxArray_int32_T *c);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for bsxfun.h
 *
 * [EOF]
 */
