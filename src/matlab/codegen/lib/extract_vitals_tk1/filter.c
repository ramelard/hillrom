/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: filter.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 18-Apr-2018 07:51:09
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "extract_vitals_tk1.h"
#include "filter.h"
#include "extract_vitals_tk1_emxutil.h"

/* Function Definitions */

/*
 * Arguments    : const double b[7]
 *                const double a[7]
 *                const emxArray_real_T *x
 *                emxArray_real_T *y
 * Return Type  : void
 */
void filter(const double b[7], const double a[7], const emxArray_real_T *x,
            emxArray_real_T *y)
{
  unsigned int x_idx_0;
  int k;
  int nx;
  int naxpy;
  int j;
  double as;
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

  for (k = 1; k <= nx; k++) {
    naxpy = (nx - k) + 1;
    if (!(naxpy <= 7)) {
      naxpy = 7;
    }

    for (j = -1; j + 2 <= naxpy; j++) {
      y->data[k + j] += x->data[k - 1] * b[j + 1];
    }

    naxpy = nx - k;
    if (!(naxpy <= 6)) {
      naxpy = 6;
    }

    as = -y->data[k - 1];
    for (j = 1; j <= naxpy; j++) {
      y->data[(k + j) - 1] += as * a[j];
    }
  }
}

/*
 * File trailer for filter.c
 *
 * [EOF]
 */
