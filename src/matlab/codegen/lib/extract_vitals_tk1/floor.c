/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: floor.c
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 08-Aug-2019 11:00:09
 */

/* Include Files */
#include <math.h>
#include "rt_nonfinite.h"
#include "extract_vitals_tk1.h"
#include "floor.h"

/* Function Definitions */

/*
 * Arguments    : emxArray_real_T *x
 * Return Type  : void
 */
void b_floor(emxArray_real_T *x)
{
  int nx;
  int k;
  nx = x->size[1];
  for (k = 0; k < nx; k++) {
    x->data[k] = floor(x->data[k]);
  }
}

/*
 * File trailer for floor.c
 *
 * [EOF]
 */
