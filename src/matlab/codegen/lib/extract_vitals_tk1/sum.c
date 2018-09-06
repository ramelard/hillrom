/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: sum.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 06-Sep-2018 09:43:33
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "extract_vitals_tk1.h"
#include "sum.h"

/* Function Definitions */

/*
 * Arguments    : const emxArray_creal_T *x
 * Return Type  : creal_T
 */
creal_T sum(const emxArray_creal_T *x)
{
  creal_T y;
  int k;
  if (x->size[1] == 0) {
    y.re = 0.0;
    y.im = 0.0;
  } else {
    y = x->data[0];
    for (k = 2; k <= x->size[1]; k++) {
      y.re += x->data[k - 1].re;
      y.im += x->data[k - 1].im;
    }
  }

  return y;
}

/*
 * File trailer for sum.c
 *
 * [EOF]
 */
