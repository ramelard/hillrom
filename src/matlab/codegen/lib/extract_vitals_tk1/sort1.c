/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: sort1.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 06-Sep-2018 09:33:28
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
  int i12;
  boolean_T p;
  for (i12 = 0; i12 < 2; i12++) {
    b_x[i12] = x[i12];
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

  for (i12 = 0; i12 < 2; i12++) {
    x[i12] = b_x[i12];
  }
}

/*
 * File trailer for sort1.c
 *
 * [EOF]
 */
