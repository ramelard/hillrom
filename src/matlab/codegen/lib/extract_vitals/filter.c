/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: filter.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 16-Apr-2018 17:05:50
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "extract_vitals.h"
#include "filter.h"
#include "extract_vitals_emxutil.h"

/* Function Definitions */

/*
 * Arguments    : const emxArray_real_T *x
 *                emxArray_real_T *y
 * Return Type  : void
 */
void filter(const emxArray_real_T *x, emxArray_real_T *y)
{
  unsigned int x_idx_0;
  int k;
  int nx;
  int naxpy;
  int j;
  static const double dv0[7] = { 0.013384535450421321, -0.0,
    -0.040153606351263971, -0.0, 0.040153606351263971, -0.0,
    -0.013384535450421321 };

  double as;
  static const double dv1[7] = { 1.0, -4.8149689085711938, 9.7461126552643478,
    -10.655494898831453, 6.6553628300631358, -2.2530885903511111,
    0.32209686282177208 };

  x_idx_0 = (unsigned int)x->size[0];
  k = y->size[0];
  y->size[0] = (int)x_idx_0;
  emxEnsureCapacity((emxArray__common *)y, k, (int)sizeof(double));
  nx = x->size[0];
  naxpy = y->size[0];
  k = y->size[0];
  y->size[0] = naxpy;
  emxEnsureCapacity((emxArray__common *)y, k, (int)sizeof(double));
  for (k = 0; k < naxpy; k++) {
    y->data[k] = 0.0;
  }

  for (k = 0; k + 1 <= nx; k++) {
    naxpy = nx - k;
    if (!(naxpy <= 7)) {
      naxpy = 7;
    }

    for (j = 0; j + 1 <= naxpy; j++) {
      y->data[k + j] += x->data[k] * dv0[j];
    }

    naxpy = (nx - k) - 1;
    if (!(naxpy <= 6)) {
      naxpy = 6;
    }

    as = -y->data[k];
    for (j = 1; j <= naxpy; j++) {
      y->data[k + j] += as * dv1[j];
    }
  }
}

/*
 * File trailer for filter.c
 *
 * [EOF]
 */
