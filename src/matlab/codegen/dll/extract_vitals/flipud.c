/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: flipud.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 01-Mar-2018 16:31:19
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "extract_vitals.h"
#include "flipud.h"

/* Function Definitions */

/*
 * Arguments    : emxArray_real_T *x
 * Return Type  : void
 */
void flipud(emxArray_real_T *x)
{
  int m;
  int n;
  int md2;
  int j;
  int i;
  int x_idx_0;
  double xtmp;
  int b_x_idx_0;
  m = x->size[0];
  n = x->size[1];
  md2 = x->size[0] >> 1;
  for (j = 0; j + 1 <= n; j++) {
    for (i = 1; i <= md2; i++) {
      x_idx_0 = x->size[0];
      xtmp = x->data[(i + x_idx_0 * j) - 1];
      x_idx_0 = x->size[0];
      b_x_idx_0 = x->size[0];
      x->data[(i + b_x_idx_0 * j) - 1] = x->data[(m - i) + x_idx_0 * j];
      x_idx_0 = x->size[0];
      x->data[(m - i) + x_idx_0 * j] = xtmp;
    }
  }
}

/*
 * File trailer for flipud.c
 *
 * [EOF]
 */
