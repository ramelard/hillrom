/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: log21.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 18-Apr-2018 07:51:09
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "extract_vitals_tk1.h"
#include "log21.h"

/* Function Definitions */

/*
 * Arguments    : double x
 * Return Type  : double
 */
double scalar_real_log2(double x)
{
  double y;
  double t;
  int eint;
  if (x == 0.0) {
    y = rtMinusInf;
  } else if ((!rtIsInf(x)) && (!rtIsNaN(x))) {
    t = frexp(x, &eint);
    if (t == 0.5) {
      y = (double)eint - 1.0;
    } else {
      y = log(t) / 0.69314718055994529 + (double)eint;
    }
  } else {
    y = x;
  }

  return y;
}

/*
 * File trailer for log21.c
 *
 * [EOF]
 */
