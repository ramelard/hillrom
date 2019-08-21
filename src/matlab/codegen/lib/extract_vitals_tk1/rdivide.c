/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: rdivide.c
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 08-Aug-2019 11:00:09
 */

/* Include Files */
#include <math.h>
#include "rt_nonfinite.h"
#include "extract_vitals_tk1.h"
#include "rdivide.h"
#include "extract_vitals_tk1_emxutil.h"

/* Function Definitions */

/*
 * Arguments    : const emxArray_creal_T *x
 *                const creal_T y
 *                emxArray_creal_T *z
 * Return Type  : void
 */
void rdivide(const emxArray_creal_T *x, const creal_T y, emxArray_creal_T *z)
{
  int i1;
  int loop_ub;
  double x_re;
  double x_im;
  double brm;
  double bim;
  double d;
  i1 = z->size[0] * z->size[1];
  z->size[0] = 1;
  z->size[1] = x->size[1];
  emxEnsureCapacity_creal_T(z, i1);
  loop_ub = x->size[0] * x->size[1];
  for (i1 = 0; i1 < loop_ub; i1++) {
    x_re = x->data[i1].re;
    x_im = x->data[i1].im;
    if (y.im == 0.0) {
      if (x_im == 0.0) {
        z->data[i1].re = x_re / y.re;
        z->data[i1].im = 0.0;
      } else if (x_re == 0.0) {
        z->data[i1].re = 0.0;
        z->data[i1].im = x_im / y.re;
      } else {
        z->data[i1].re = x_re / y.re;
        z->data[i1].im = x_im / y.re;
      }
    } else if (y.re == 0.0) {
      if (x_re == 0.0) {
        z->data[i1].re = x_im / y.im;
        z->data[i1].im = 0.0;
      } else if (x_im == 0.0) {
        z->data[i1].re = 0.0;
        z->data[i1].im = -(x_re / y.im);
      } else {
        z->data[i1].re = x_im / y.im;
        z->data[i1].im = -(x_re / y.im);
      }
    } else {
      brm = fabs(y.re);
      bim = fabs(y.im);
      if (brm > bim) {
        bim = y.im / y.re;
        d = y.re + bim * y.im;
        z->data[i1].re = (x_re + bim * x_im) / d;
        z->data[i1].im = (x_im - bim * x_re) / d;
      } else if (bim == brm) {
        if (y.re > 0.0) {
          bim = 0.5;
        } else {
          bim = -0.5;
        }

        if (y.im > 0.0) {
          d = 0.5;
        } else {
          d = -0.5;
        }

        z->data[i1].re = (x_re * bim + x_im * d) / brm;
        z->data[i1].im = (x_im * bim - x_re * d) / brm;
      } else {
        bim = y.re / y.im;
        d = y.im + bim * y.re;
        z->data[i1].re = (bim * x_re + x_im) / d;
        z->data[i1].im = (bim * x_im - x_re) / d;
      }
    }
  }
}

/*
 * File trailer for rdivide.c
 *
 * [EOF]
 */
