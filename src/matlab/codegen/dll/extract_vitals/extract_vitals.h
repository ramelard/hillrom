/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: extract_vitals.h
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 01-Mar-2018 16:31:19
 */

#ifndef EXTRACT_VITALS_H
#define EXTRACT_VITALS_H

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

  extern void extract_vitals(const emxArray_real_T *frames, double block_size,
    double *hr, double *rr);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for extract_vitals.h
 *
 * [EOF]
 */
