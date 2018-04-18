/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: sort1.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 18-Apr-2018 07:51:09
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "extract_vitals_tk1.h"
#include "sort1.h"

/* Function Definitions */

/*
 * Arguments    : double x[2]
 *                int idx[2]
 * Return Type  : void
 */
void sort(double x[2], int idx[2])
{
  double b_x[2];
  int i10;
  boolean_T p;
  for (i10 = 0; i10 < 2; i10++) {
    b_x[i10] = x[i10];
  }

  if ((x[0] <= x[1]) || rtIsNaN(x[1])) {
    p = true;
  } else {
    p = false;
  }

  if (p) {
    idx[0] = 1;
    idx[1] = 2;
  } else {
    idx[0] = 2;
    idx[1] = 1;
    b_x[0] = x[1];
    b_x[1] = x[0];
  }

  for (i10 = 0; i10 < 2; i10++) {
    x[i10] = b_x[i10];
  }
}

/*
 * File trailer for sort1.c
 *
 * [EOF]
 */
