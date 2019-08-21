/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: combineVectorElements.c
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 08-Aug-2019 11:00:09
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "extract_vitals_tk1.h"
#include "combineVectorElements.h"
#include "extract_vitals_tk1_emxutil.h"

/* Function Definitions */

/*
 * Arguments    : const emxArray_creal_T *x
 *                int vlen
 *                emxArray_creal_T *y
 * Return Type  : void
 */
void colMajorFlatIter(const emxArray_creal_T *x, int vlen, emxArray_creal_T *y)
{
  int i;
  int xpageoffset;
  int k;
  i = y->size[0] * y->size[1];
  y->size[0] = 1;
  y->size[1] = x->size[1];
  emxEnsureCapacity_creal_T(y, i);
  for (i = 0; i < x->size[1]; i++) {
    xpageoffset = i * x->size[0];
    y->data[i] = x->data[xpageoffset];
    for (k = 2; k <= vlen; k++) {
      y->data[i].re += x->data[(xpageoffset + k) - 1].re;
      y->data[i].im += x->data[(xpageoffset + k) - 1].im;
    }
  }
}

/*
 * File trailer for combineVectorElements.c
 *
 * [EOF]
 */
