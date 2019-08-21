/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: fft1.h
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 08-Aug-2019 11:00:09
 */

#ifndef FFT1_H
#define FFT1_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "extract_vitals_tk1_types.h"

/* Function Declarations */
extern void bluestein(const emxArray_real_T *x, int xoffInit, int nfft, int
                      nRows, const emxArray_real_T *costab, const
                      emxArray_real_T *sintab, const emxArray_real_T *costabinv,
                      const emxArray_real_T *sintabinv, const emxArray_creal_T
                      *wwc, emxArray_creal_T *y);
extern void generate_twiddle_tables(int nRows, boolean_T useRadix2,
  emxArray_real_T *costab, emxArray_real_T *sintab, emxArray_real_T *sintabinv);

#endif

/*
 * File trailer for fft1.h
 *
 * [EOF]
 */
