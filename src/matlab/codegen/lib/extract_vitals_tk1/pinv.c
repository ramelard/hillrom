/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: pinv.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 06-Sep-2018 16:44:03
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "extract_vitals_tk1.h"
#include "pinv.h"

/* Function Definitions */

/*
 * Arguments    : double A
 * Return Type  : double
 */
double pinv(double A)
{
  double X;
  double U;
  int m;
  double rt;
  double Vf;
  X = 0.0;
  if ((!rtIsInf(A)) && (!rtIsNaN(A))) {
    m = 1;
    rt = A;
    U = 1.0;
    Vf = 1.0;
    if (A != 0.0) {
      rt = fabs(A);
      U = A / rt;
    }

    while (m > 0) {
      if (rt < 0.0) {
        rt = -rt;
        Vf = -Vf;
      }

      m = 0;
    }
  } else {
    U = rtNaN;
    rt = rtNaN;
    Vf = rtNaN;
  }

  m = 0;
  if (rt > rt * 2.2204460492503131E-16) {
    m = 1;
  }

  if (m > 0) {
    rt = 1.0 / rt;
    m = 1;
    while (m <= 1) {
      Vf *= rt;
      m = 2;
    }

    if (U != 0.0) {
      X = U * Vf;
    }
  }

  return X;
}

/*
 * File trailer for pinv.c
 *
 * [EOF]
 */
