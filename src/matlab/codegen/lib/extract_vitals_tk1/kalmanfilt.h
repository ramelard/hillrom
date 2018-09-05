/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: kalmanfilt.h
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 05-Sep-2018 12:32:31
 */

#ifndef KALMANFILT_H
#define KALMANFILT_H

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "extract_vitals_tk1_types.h"

/* Function Declarations */
extern void b_kalmanfilt(const emxArray_real_T *z, emxArray_real_T *xest);
extern void kalmanfilt(const emxArray_creal_T *z, emxArray_real_T *xest);

#endif

/*
 * File trailer for kalmanfilt.h
 *
 * [EOF]
 */
