/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: filter.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 01-Mar-2018 16:31:19
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "extract_vitals.h"
#include "filter.h"

/* Function Definitions */

/*
 * Arguments    : const double x_data[]
 *                double y_data[]
 *                int y_size[1]
 * Return Type  : void
 */
void filter(const double x_data[], double y_data[], int y_size[1])
{
  int k;
  int j;
  double as;
  static const double dv0[7] = { 0.013384535450421321, -0.0,
    -0.040153606351263971, -0.0, 0.040153606351263971, -0.0,
    -0.013384535450421321 };

  static const double dv1[7] = { 1.0, -4.8149689085711938, 9.7461126552643478,
    -10.655494898831453, 6.6553628300631358, -2.2530885903511111,
    0.32209686282177208 };

  y_size[0] = 3;
  for (k = 0; k < 3; k++) {
    y_data[k] = 0.0;
  }

  for (k = 0; k < 3; k++) {
    for (j = 0; j + 1 <= 3 - k; j++) {
      y_data[k + j] += x_data[k] * dv0[j];
    }

    as = -y_data[k];
    for (j = 1; j <= 2 - k; j++) {
      y_data[k + j] += as * dv1[j];
    }
  }
}

/*
 * File trailer for filter.c
 *
 * [EOF]
 */
