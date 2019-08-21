/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: fft.c
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 08-Aug-2019 11:00:09
 */

/* Include Files */
#include <math.h>
#include "rt_nonfinite.h"
#include "extract_vitals_tk1.h"
#include "fft.h"
#include "extract_vitals_tk1_emxutil.h"
#include "fft1.h"

/* Function Definitions */

/*
 * Arguments    : const emxArray_real_T *x
 *                double varargin_1
 *                emxArray_creal_T *y
 * Return Type  : void
 */
void fft(const emxArray_real_T *x, double varargin_1, emxArray_creal_T *y)
{
  emxArray_real_T *costab;
  emxArray_real_T *sintab;
  emxArray_real_T *sintabinv;
  emxArray_creal_T *wwc;
  emxArray_creal_T *rwork;
  boolean_T useRadix2;
  int istart;
  int ihi;
  int sz[2];
  int nn1m1;
  int pmax;
  int pmin;
  boolean_T exitg1;
  int pow2p;
  unsigned int sx[2];
  int k;
  int nRowsD2;
  double nt_im;
  int nRowsD4;
  double nt_re;
  int i;
  double temp_re;
  double temp_im;
  emxInit_real_T(&costab, 2);
  emxInit_real_T(&sintab, 2);
  emxInit_real_T(&sintabinv, 2);
  emxInit_creal_T1(&wwc, 1);
  emxInit_creal_T1(&rwork, 1);
  if ((x->size[0] == 0) || (x->size[1] == 0) || ((int)varargin_1 == 0)) {
    for (istart = 0; istart < 2; istart++) {
      sz[istart] = x->size[istart];
    }

    istart = y->size[0] * y->size[1];
    y->size[0] = (int)varargin_1;
    y->size[1] = sz[1];
    emxEnsureCapacity_creal_T(y, istart);
    if ((int)varargin_1 > x->size[0]) {
      istart = y->size[0] * y->size[1];
      emxEnsureCapacity_creal_T(y, istart);
      pmin = y->size[1];
      for (istart = 0; istart < pmin; istart++) {
        nn1m1 = y->size[0];
        for (pmax = 0; pmax < nn1m1; pmax++) {
          y->data[pmax + y->size[0] * istart].re = 0.0;
          y->data[pmax + y->size[0] * istart].im = 0.0;
        }
      }
    }
  } else {
    useRadix2 = (((int)varargin_1 & ((int)varargin_1 - 1)) == 0);
    ihi = 1;
    if (useRadix2) {
      nn1m1 = (int)varargin_1;
    } else {
      nn1m1 = ((int)varargin_1 + (int)varargin_1) - 1;
      pmax = 31;
      if (nn1m1 <= 1) {
        pmax = 0;
      } else {
        pmin = 0;
        exitg1 = false;
        while ((!exitg1) && (pmax - pmin > 1)) {
          istart = (pmin + pmax) >> 1;
          pow2p = 1 << istart;
          if (pow2p == nn1m1) {
            pmax = istart;
            exitg1 = true;
          } else if (pow2p > nn1m1) {
            pmax = istart;
          } else {
            pmin = istart;
          }
        }
      }

      ihi = 1 << pmax;
      nn1m1 = ihi;
    }

    generate_twiddle_tables(nn1m1, useRadix2, costab, sintab, sintabinv);
    if (useRadix2) {
      for (istart = 0; istart < 2; istart++) {
        sx[istart] = (unsigned int)x->size[istart];
      }

      for (istart = 0; istart < 2; istart++) {
        sz[istart] = x->size[istart];
      }

      istart = y->size[0] * y->size[1];
      y->size[0] = (int)varargin_1;
      y->size[1] = sz[1];
      emxEnsureCapacity_creal_T(y, istart);
      if ((int)varargin_1 > x->size[0]) {
        istart = y->size[0] * y->size[1];
        emxEnsureCapacity_creal_T(y, istart);
        pmin = y->size[1];
        for (istart = 0; istart < pmin; istart++) {
          nn1m1 = y->size[0];
          for (pmax = 0; pmax < nn1m1; pmax++) {
            y->data[pmax + y->size[0] * istart].re = 0.0;
            y->data[pmax + y->size[0] * istart].im = 0.0;
          }
        }
      }

      for (k = 0; k < (int)sx[1]; k++) {
        nn1m1 = x->size[0];
        pow2p = (int)varargin_1;
        if (nn1m1 < pow2p) {
          pow2p = nn1m1;
        }

        nRowsD2 = (int)varargin_1 / 2;
        nRowsD4 = nRowsD2 / 2;
        istart = rwork->size[0];
        rwork->size[0] = (int)varargin_1;
        emxEnsureCapacity_creal_T1(rwork, istart);
        if ((int)varargin_1 > x->size[0]) {
          pmin = rwork->size[0];
          istart = rwork->size[0];
          rwork->size[0] = pmin;
          emxEnsureCapacity_creal_T1(rwork, istart);
          for (istart = 0; istart < pmin; istart++) {
            rwork->data[istart].re = 0.0;
            rwork->data[istart].im = 0.0;
          }
        }

        pmax = k * x->size[0];
        pmin = 0;
        nn1m1 = 0;
        for (i = 1; i < pow2p; i++) {
          rwork->data[nn1m1].re = x->data[pmax];
          rwork->data[nn1m1].im = 0.0;
          nn1m1 = (int)varargin_1;
          useRadix2 = true;
          while (useRadix2) {
            nn1m1 >>= 1;
            pmin ^= nn1m1;
            useRadix2 = ((pmin & nn1m1) == 0);
          }

          nn1m1 = pmin;
          pmax++;
        }

        rwork->data[nn1m1].re = x->data[pmax];
        rwork->data[nn1m1].im = 0.0;
        if ((int)varargin_1 > 1) {
          for (i = 0; i <= (int)varargin_1 - 2; i += 2) {
            temp_re = rwork->data[i + 1].re;
            temp_im = rwork->data[i + 1].im;
            rwork->data[i + 1].re = rwork->data[i].re - rwork->data[i + 1].re;
            rwork->data[i + 1].im = rwork->data[i].im - rwork->data[i + 1].im;
            rwork->data[i].re += temp_re;
            rwork->data[i].im += temp_im;
          }
        }

        nn1m1 = 2;
        pmax = 4;
        pmin = 1 + ((nRowsD4 - 1) << 2);
        while (nRowsD4 > 0) {
          for (i = 0; i < pmin; i += pmax) {
            temp_re = rwork->data[i + nn1m1].re;
            temp_im = rwork->data[i + nn1m1].im;
            rwork->data[i + nn1m1].re = rwork->data[i].re - temp_re;
            rwork->data[i + nn1m1].im = rwork->data[i].im - temp_im;
            rwork->data[i].re += temp_re;
            rwork->data[i].im += temp_im;
          }

          istart = 1;
          for (pow2p = nRowsD4; pow2p < nRowsD2; pow2p += nRowsD4) {
            nt_re = costab->data[pow2p];
            nt_im = sintab->data[pow2p];
            i = istart;
            ihi = istart + pmin;
            while (i < ihi) {
              temp_re = nt_re * rwork->data[i + nn1m1].re - nt_im * rwork->
                data[i + nn1m1].im;
              temp_im = nt_re * rwork->data[i + nn1m1].im + nt_im * rwork->
                data[i + nn1m1].re;
              rwork->data[i + nn1m1].re = rwork->data[i].re - temp_re;
              rwork->data[i + nn1m1].im = rwork->data[i].im - temp_im;
              rwork->data[i].re += temp_re;
              rwork->data[i].im += temp_im;
              i += pmax;
            }

            istart++;
          }

          nRowsD4 /= 2;
          nn1m1 = pmax;
          pmax += pmax;
          pmin -= nn1m1;
        }

        for (nn1m1 = 0; nn1m1 < (int)varargin_1; nn1m1++) {
          y->data[nn1m1 + y->size[0] * k] = rwork->data[nn1m1];
        }
      }
    } else {
      for (istart = 0; istart < 2; istart++) {
        sx[istart] = (unsigned int)x->size[istart];
      }

      pow2p = ((int)varargin_1 + (int)varargin_1) - 1;
      istart = wwc->size[0];
      wwc->size[0] = pow2p;
      emxEnsureCapacity_creal_T1(wwc, istart);
      nn1m1 = (int)varargin_1;
      pmax = 0;
      wwc->data[(int)varargin_1 - 1].re = 1.0;
      wwc->data[(int)varargin_1 - 1].im = 0.0;
      pmin = (int)varargin_1 << 1;
      for (k = 1; k < (int)varargin_1; k++) {
        istart = (k << 1) - 1;
        if (pmin - pmax <= istart) {
          pmax += istart - pmin;
        } else {
          pmax += istart;
        }

        nt_im = -3.1415926535897931 * (double)pmax / (double)(int)varargin_1;
        if (nt_im == 0.0) {
          nt_re = 1.0;
          nt_im = 0.0;
        } else {
          nt_re = cos(nt_im);
          nt_im = sin(nt_im);
        }

        wwc->data[nn1m1 - 2].re = nt_re;
        wwc->data[nn1m1 - 2].im = -nt_im;
        nn1m1--;
      }

      nn1m1 = 0;
      for (k = pow2p - 1; k >= (int)varargin_1; k--) {
        wwc->data[k] = wwc->data[nn1m1];
        nn1m1++;
      }

      for (istart = 0; istart < 2; istart++) {
        sz[istart] = x->size[istart];
      }

      istart = y->size[0] * y->size[1];
      y->size[0] = (int)varargin_1;
      y->size[1] = sz[1];
      emxEnsureCapacity_creal_T(y, istart);
      if ((int)varargin_1 > x->size[0]) {
        istart = y->size[0] * y->size[1];
        emxEnsureCapacity_creal_T(y, istart);
        pmin = y->size[1];
        for (istart = 0; istart < pmin; istart++) {
          nn1m1 = y->size[0];
          for (pmax = 0; pmax < nn1m1; pmax++) {
            y->data[pmax + y->size[0] * istart].re = 0.0;
            y->data[pmax + y->size[0] * istart].im = 0.0;
          }
        }
      }

      for (k = 0; k < (int)sx[1]; k++) {
        bluestein(x, k * x->size[0], ihi, (int)varargin_1, costab, sintab,
                  costab, sintabinv, wwc, rwork);
        for (nn1m1 = 0; nn1m1 < (int)varargin_1; nn1m1++) {
          y->data[nn1m1 + y->size[0] * k] = rwork->data[nn1m1];
        }
      }
    }
  }

  emxFree_creal_T(&rwork);
  emxFree_creal_T(&wwc);
  emxFree_real_T(&sintabinv);
  emxFree_real_T(&sintab);
  emxFree_real_T(&costab);
}

/*
 * File trailer for fft.c
 *
 * [EOF]
 */
