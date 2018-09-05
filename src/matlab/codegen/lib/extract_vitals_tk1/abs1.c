/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: abs1.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 05-Sep-2018 12:32:31
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "extract_vitals_tk1.h"
#include "abs1.h"
#include "relop.h"
#include "extract_vitals_tk1_emxutil.h"
#include "extract_vitals_tk1_rtwutil.h"

/* Function Definitions */

/*
 * Arguments    : const emxArray_creal_T *x
 *                emxArray_real_T *y
 * Return Type  : void
 */
void b_abs(const emxArray_creal_T *x, emxArray_real_T *y)
{
  unsigned int uv1[2];
  int n;
  int k;
  for (n = 0; n < 2; n++) {
    uv1[n] = (unsigned int)x->size[n];
  }

  n = y->size[0] * y->size[1];
  y->size[0] = (int)uv1[0];
  y->size[1] = (int)uv1[1];
  emxEnsureCapacity((emxArray__common *)y, n, (int)sizeof(double));
  n = x->size[0] * x->size[1];
  for (k = 0; k + 1 <= n; k++) {
    y->data[k] = rt_hypotd_snf(x->data[k].re, x->data[k].im);
  }
}

/*
 * File trailer for abs1.c
 *
 * [EOF]
 */
