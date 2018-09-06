/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: fft.h
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 06-Sep-2018 14:56:40
 */

#ifndef FFT_H
#define FFT_H

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
extern void b_fft(const emxArray_real_T *x, emxArray_creal_T *y);
extern void c_fft(const emxArray_real_T *x, double varargin_1, emxArray_creal_T *
                  y);
extern void fft(const emxArray_real_T *x, double varargin_1, emxArray_creal_T *y);

#endif

/*
 * File trailer for fft.h
 *
 * [EOF]
 */
