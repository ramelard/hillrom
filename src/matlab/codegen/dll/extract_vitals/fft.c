/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: fft.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 01-Mar-2018 16:31:19
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "extract_vitals.h"
#include "fft.h"
#include "extract_vitals_emxutil.h"
#include "plot_power_spectrum.h"
#include "bluestein_setup.h"

/* Function Declarations */
static void dobluesteinfft(const emxArray_real_T *x, int N2, int n1, const
  emxArray_real_T *costab, const emxArray_real_T *sintab, const emxArray_real_T *
  sintabinv, emxArray_creal_T *y);
static void generate_twiddle_tables(int nRows, boolean_T useRadix2,
  emxArray_real_T *costab, emxArray_real_T *sintab, emxArray_real_T *sintabinv);

/* Function Definitions */

/*
 * Arguments    : const emxArray_real_T *x
 *                int N2
 *                int n1
 *                const emxArray_real_T *costab
 *                const emxArray_real_T *sintab
 *                const emxArray_real_T *sintabinv
 *                emxArray_creal_T *y
 * Return Type  : void
 */
static void dobluesteinfft(const emxArray_real_T *x, int N2, int n1, const
  emxArray_real_T *costab, const emxArray_real_T *sintab, const emxArray_real_T *
  sintabinv, emxArray_creal_T *y)
{
  unsigned int sx[2];
  int i9;
  emxArray_creal_T *wwc;
  int sz[2];
  int k;
  emxArray_creal_T *rwork;
  int minNrowsNx;
  emxArray_creal_T *fy;
  emxArray_creal_T *fv;
  int xidx;
  int b_k;
  double wwc_re;
  double wwc_im;
  double fv_re;
  double fv_im;
  double b_wwc_re;
  double b_fv_im;
  double b_wwc_im;
  double b_fv_re;
  for (i9 = 0; i9 < 2; i9++) {
    sx[i9] = (unsigned int)x->size[i9];
  }

  emxInit_creal_T1(&wwc, 1);
  bluestein_setup(n1, wwc);
  for (i9 = 0; i9 < 2; i9++) {
    sz[i9] = x->size[i9];
  }

  i9 = y->size[0] * y->size[1];
  y->size[0] = n1;
  y->size[1] = sz[1];
  emxEnsureCapacity((emxArray__common *)y, i9, (int)sizeof(creal_T));
  if (n1 > 3) {
    i9 = y->size[0] * y->size[1];
    emxEnsureCapacity((emxArray__common *)y, i9, (int)sizeof(creal_T));
    minNrowsNx = y->size[1];
    for (i9 = 0; i9 < minNrowsNx; i9++) {
      xidx = y->size[0];
      for (b_k = 0; b_k < xidx; b_k++) {
        y->data[b_k + y->size[0] * i9].re = 0.0;
        y->data[b_k + y->size[0] * i9].im = 0.0;
      }
    }
  }

  k = 0;
  emxInit_creal_T1(&rwork, 1);
  emxInit_creal_T1(&fy, 1);
  emxInit_creal_T1(&fv, 1);
  while (k + 1 <= (int)sx[1]) {
    if (n1 <= 3) {
      minNrowsNx = n1;
    } else {
      minNrowsNx = 3;
    }

    i9 = rwork->size[0];
    rwork->size[0] = n1;
    emxEnsureCapacity((emxArray__common *)rwork, i9, (int)sizeof(creal_T));
    if (n1 > 3) {
      xidx = rwork->size[0];
      i9 = rwork->size[0];
      rwork->size[0] = xidx;
      emxEnsureCapacity((emxArray__common *)rwork, i9, (int)sizeof(creal_T));
      for (i9 = 0; i9 < xidx; i9++) {
        rwork->data[i9].re = 0.0;
        rwork->data[i9].im = 0.0;
      }
    }

    xidx = k * 3;
    for (b_k = 0; b_k + 1 <= minNrowsNx; b_k++) {
      wwc_re = wwc->data[(n1 + b_k) - 1].re;
      wwc_im = wwc->data[(n1 + b_k) - 1].im;
      rwork->data[b_k].re = wwc_re * x->data[xidx];
      rwork->data[b_k].im = wwc_im * -x->data[xidx];
      xidx++;
    }

    while (minNrowsNx + 1 <= n1) {
      rwork->data[minNrowsNx].re = 0.0;
      rwork->data[minNrowsNx].im = 0.0;
      minNrowsNx++;
    }

    r2br_r2dit_trig_impl(rwork, N2, costab, sintab, fy);
    r2br_r2dit_trig(wwc, N2, costab, sintab, fv);
    i9 = fy->size[0];
    emxEnsureCapacity((emxArray__common *)fy, i9, (int)sizeof(creal_T));
    minNrowsNx = fy->size[0];
    for (i9 = 0; i9 < minNrowsNx; i9++) {
      wwc_re = fy->data[i9].re;
      wwc_im = fy->data[i9].im;
      fv_re = fv->data[i9].re;
      fv_im = fv->data[i9].im;
      fy->data[i9].re = wwc_re * fv_re - wwc_im * fv_im;
      fy->data[i9].im = wwc_re * fv_im + wwc_im * fv_re;
    }

    b_r2br_r2dit_trig(fy, N2, costab, sintabinv, fv);
    xidx = 0;
    for (b_k = n1 - 1; b_k + 1 <= wwc->size[0]; b_k++) {
      wwc_re = wwc->data[b_k].re;
      fv_re = fv->data[b_k].re;
      wwc_im = wwc->data[b_k].im;
      fv_im = fv->data[b_k].im;
      b_wwc_re = wwc->data[b_k].re;
      b_fv_im = fv->data[b_k].im;
      b_wwc_im = wwc->data[b_k].im;
      b_fv_re = fv->data[b_k].re;
      rwork->data[xidx].re = wwc_re * fv_re + wwc_im * fv_im;
      rwork->data[xidx].im = b_wwc_re * b_fv_im - b_wwc_im * b_fv_re;
      xidx++;
    }

    for (xidx = 0; xidx + 1 <= n1; xidx++) {
      y->data[xidx + y->size[0] * k] = rwork->data[xidx];
    }

    k++;
  }

  emxFree_creal_T(&fv);
  emxFree_creal_T(&fy);
  emxFree_creal_T(&rwork);
  emxFree_creal_T(&wwc);
}

/*
 * Arguments    : int nRows
 *                boolean_T useRadix2
 *                emxArray_real_T *costab
 *                emxArray_real_T *sintab
 *                emxArray_real_T *sintabinv
 * Return Type  : void
 */
static void generate_twiddle_tables(int nRows, boolean_T useRadix2,
  emxArray_real_T *costab, emxArray_real_T *sintab, emxArray_real_T *sintabinv)
{
  emxArray_real_T *costab1q;
  double e;
  int nRowsD4;
  int nd2;
  int k;
  int n2;
  emxInit_real_T(&costab1q, 2);
  e = 6.2831853071795862 / (double)nRows;
  nRowsD4 = nRows / 2 / 2;
  nd2 = costab1q->size[0] * costab1q->size[1];
  costab1q->size[0] = 1;
  costab1q->size[1] = nRowsD4 + 1;
  emxEnsureCapacity((emxArray__common *)costab1q, nd2, (int)sizeof(double));
  costab1q->data[0] = 1.0;
  nd2 = nRowsD4 / 2;
  for (k = 1; k <= nd2; k++) {
    costab1q->data[k] = cos(e * (double)k);
  }

  for (k = nd2 + 1; k < nRowsD4; k++) {
    costab1q->data[k] = sin(e * (double)(nRowsD4 - k));
  }

  costab1q->data[nRowsD4] = 0.0;
  if (!useRadix2) {
    nRowsD4 = costab1q->size[1] - 1;
    n2 = (costab1q->size[1] - 1) << 1;
    nd2 = costab->size[0] * costab->size[1];
    costab->size[0] = 1;
    costab->size[1] = n2 + 1;
    emxEnsureCapacity((emxArray__common *)costab, nd2, (int)sizeof(double));
    nd2 = sintab->size[0] * sintab->size[1];
    sintab->size[0] = 1;
    sintab->size[1] = n2 + 1;
    emxEnsureCapacity((emxArray__common *)sintab, nd2, (int)sizeof(double));
    costab->data[0] = 1.0;
    sintab->data[0] = 0.0;
    nd2 = sintabinv->size[0] * sintabinv->size[1];
    sintabinv->size[0] = 1;
    sintabinv->size[1] = n2 + 1;
    emxEnsureCapacity((emxArray__common *)sintabinv, nd2, (int)sizeof(double));
    for (k = 1; k <= nRowsD4; k++) {
      sintabinv->data[k] = costab1q->data[nRowsD4 - k];
    }

    for (k = costab1q->size[1]; k <= n2; k++) {
      sintabinv->data[k] = costab1q->data[k - nRowsD4];
    }

    for (k = 1; k <= nRowsD4; k++) {
      costab->data[k] = costab1q->data[k];
      sintab->data[k] = -costab1q->data[nRowsD4 - k];
    }

    for (k = costab1q->size[1]; k <= n2; k++) {
      costab->data[k] = -costab1q->data[n2 - k];
      sintab->data[k] = -costab1q->data[k - nRowsD4];
    }
  } else {
    nRowsD4 = costab1q->size[1] - 1;
    n2 = (costab1q->size[1] - 1) << 1;
    nd2 = costab->size[0] * costab->size[1];
    costab->size[0] = 1;
    costab->size[1] = n2 + 1;
    emxEnsureCapacity((emxArray__common *)costab, nd2, (int)sizeof(double));
    nd2 = sintab->size[0] * sintab->size[1];
    sintab->size[0] = 1;
    sintab->size[1] = n2 + 1;
    emxEnsureCapacity((emxArray__common *)sintab, nd2, (int)sizeof(double));
    costab->data[0] = 1.0;
    sintab->data[0] = 0.0;
    for (k = 1; k <= nRowsD4; k++) {
      costab->data[k] = costab1q->data[k];
      sintab->data[k] = -costab1q->data[nRowsD4 - k];
    }

    for (k = costab1q->size[1]; k <= n2; k++) {
      costab->data[k] = -costab1q->data[n2 - k];
      sintab->data[k] = -costab1q->data[k - nRowsD4];
    }

    nd2 = sintabinv->size[0] * sintabinv->size[1];
    sintabinv->size[0] = 1;
    sintabinv->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)sintabinv, nd2, (int)sizeof(double));
  }

  emxFree_real_T(&costab1q);
}

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
  emxArray_creal_T *rwork;
  boolean_T useRadix2;
  int sz[2];
  int iy;
  int iDelta;
  int ju;
  unsigned int sx[2];
  int ix;
  int k;
  int j;
  int nRowsD2;
  int nRowsD4;
  int i;
  double temp_re;
  double temp_im;
  double twid_re;
  double twid_im;
  int ihi;
  emxInit_real_T(&costab, 2);
  emxInit_real_T(&sintab, 2);
  emxInit_real_T(&sintabinv, 2);
  emxInit_creal_T1(&rwork, 1);
  if ((x->size[1] == 0) || ((int)varargin_1 == 0)) {
    for (ju = 0; ju < 2; ju++) {
      sz[ju] = x->size[ju];
    }

    ju = y->size[0] * y->size[1];
    y->size[0] = (int)varargin_1;
    y->size[1] = sz[1];
    emxEnsureCapacity((emxArray__common *)y, ju, (int)sizeof(creal_T));
    if ((int)varargin_1 > 3) {
      ju = y->size[0] * y->size[1];
      emxEnsureCapacity((emxArray__common *)y, ju, (int)sizeof(creal_T));
      iy = y->size[1];
      for (ju = 0; ju < iy; ju++) {
        iDelta = y->size[0];
        for (ix = 0; ix < iDelta; ix++) {
          y->data[ix + y->size[0] * ju].re = 0.0;
          y->data[ix + y->size[0] * ju].im = 0.0;
        }
      }
    }
  } else {
    useRadix2 = (((int)varargin_1 & ((int)varargin_1 - 1)) == 0);
    get_algo_sizes((int)varargin_1, useRadix2, &iy, &iDelta);
    generate_twiddle_tables(iDelta, useRadix2, costab, sintab, sintabinv);
    if (useRadix2) {
      for (ju = 0; ju < 2; ju++) {
        sx[ju] = (unsigned int)x->size[ju];
      }

      for (ju = 0; ju < 2; ju++) {
        sz[ju] = x->size[ju];
      }

      ju = y->size[0] * y->size[1];
      y->size[0] = (int)varargin_1;
      y->size[1] = sz[1];
      emxEnsureCapacity((emxArray__common *)y, ju, (int)sizeof(creal_T));
      if ((int)varargin_1 > 3) {
        ju = y->size[0] * y->size[1];
        emxEnsureCapacity((emxArray__common *)y, ju, (int)sizeof(creal_T));
        iy = y->size[1];
        for (ju = 0; ju < iy; ju++) {
          iDelta = y->size[0];
          for (ix = 0; ix < iDelta; ix++) {
            y->data[ix + y->size[0] * ju].re = 0.0;
            y->data[ix + y->size[0] * ju].im = 0.0;
          }
        }
      }

      for (k = 0; k + 1 <= (int)sx[1]; k++) {
        if (3 <= (int)varargin_1) {
          j = 3;
        } else {
          j = (int)varargin_1;
        }

        nRowsD2 = (int)varargin_1 / 2;
        nRowsD4 = nRowsD2 / 2;
        ju = rwork->size[0];
        rwork->size[0] = (int)varargin_1;
        emxEnsureCapacity((emxArray__common *)rwork, ju, (int)sizeof(creal_T));
        if ((int)varargin_1 > 3) {
          iy = rwork->size[0];
          ju = rwork->size[0];
          rwork->size[0] = iy;
          emxEnsureCapacity((emxArray__common *)rwork, ju, (int)sizeof(creal_T));
          for (ju = 0; ju < iy; ju++) {
            rwork->data[ju].re = 0.0;
            rwork->data[ju].im = 0.0;
          }
        }

        ix = k * 3;
        ju = 0;
        iy = 0;
        for (i = 1; i < j; i++) {
          rwork->data[iy].re = x->data[ix];
          rwork->data[iy].im = 0.0;
          iDelta = (int)varargin_1;
          useRadix2 = true;
          while (useRadix2) {
            iDelta >>= 1;
            ju ^= iDelta;
            useRadix2 = ((ju & iDelta) == 0);
          }

          iy = ju;
          ix++;
        }

        rwork->data[iy].re = x->data[ix];
        rwork->data[iy].im = 0.0;
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

        iDelta = 2;
        iy = 4;
        ix = 1 + ((nRowsD4 - 1) << 2);
        while (nRowsD4 > 0) {
          for (i = 0; i < ix; i += iy) {
            temp_re = rwork->data[i + iDelta].re;
            temp_im = rwork->data[i + iDelta].im;
            rwork->data[i + iDelta].re = rwork->data[i].re - temp_re;
            rwork->data[i + iDelta].im = rwork->data[i].im - temp_im;
            rwork->data[i].re += temp_re;
            rwork->data[i].im += temp_im;
          }

          ju = 1;
          for (j = nRowsD4; j < nRowsD2; j += nRowsD4) {
            twid_re = costab->data[j];
            twid_im = sintab->data[j];
            i = ju;
            ihi = ju + ix;
            while (i < ihi) {
              temp_re = twid_re * rwork->data[i + iDelta].re - twid_im *
                rwork->data[i + iDelta].im;
              temp_im = twid_re * rwork->data[i + iDelta].im + twid_im *
                rwork->data[i + iDelta].re;
              rwork->data[i + iDelta].re = rwork->data[i].re - temp_re;
              rwork->data[i + iDelta].im = rwork->data[i].im - temp_im;
              rwork->data[i].re += temp_re;
              rwork->data[i].im += temp_im;
              i += iy;
            }

            ju++;
          }

          nRowsD4 /= 2;
          iDelta = iy;
          iy <<= 1;
          ix -= iDelta;
        }

        for (iy = 0; iy + 1 <= (int)varargin_1; iy++) {
          y->data[iy + y->size[0] * k] = rwork->data[iy];
        }
      }
    } else {
      dobluesteinfft(x, iy, (int)varargin_1, costab, sintab, sintabinv, y);
    }
  }

  emxFree_creal_T(&rwork);
  emxFree_real_T(&sintabinv);
  emxFree_real_T(&sintab);
  emxFree_real_T(&costab);
}

/*
 * File trailer for fft.c
 *
 * [EOF]
 */
