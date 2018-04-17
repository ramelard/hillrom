/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: log.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 16-Apr-2018 17:05:50
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "extract_vitals.h"
#include "log.h"
#include "extract_vitals_emxutil.h"

/* Function Definitions */

/*
 * Arguments    : const emxArray_real_T *x
 *                emxArray_real_T *b_x
 * Return Type  : void
 */
void b_log(const emxArray_real_T *x, emxArray_real_T *b_x)
{
  int i11;
  int loop_ub;
  i11 = b_x->size[0] * b_x->size[1];
  b_x->size[0] = 1;
  b_x->size[1] = x->size[1];
  emxEnsureCapacity((emxArray__common *)b_x, i11, (int)sizeof(double));
  loop_ub = x->size[0] * x->size[1];
  for (i11 = 0; i11 < loop_ub; i11++) {
    b_x->data[i11] = x->data[i11];
  }

  d_log(b_x);
}

/*
 * Arguments    : emxArray_real_T *x
 * Return Type  : void
 */
void c_log(emxArray_real_T *x)
{
  int nx;
  int k;
  nx = x->size[0] * x->size[1];
  for (k = 0; k + 1 <= nx; k++) {
    x->data[k] = log(x->data[k]);
  }
}

/*
 * Arguments    : emxArray_real_T *x
 * Return Type  : void
 */
void d_log(emxArray_real_T *x)
{
  int nx;
  int k;
  nx = x->size[1];
  for (k = 0; k + 1 <= nx; k++) {
    x->data[k] = log(x->data[k]);
  }
}

/*
 * File trailer for log.c
 *
 * [EOF]
 */
