/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: fft1.c
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 08-Aug-2019 11:00:09
 */

/* Include Files */
#include <math.h>
#include "rt_nonfinite.h"
#include "extract_vitals_tk1.h"
#include "fft1.h"
#include "extract_vitals_tk1_emxutil.h"

/* Function Declarations */
static void r2br_r2dit_trig_impl(const emxArray_creal_T *x, int unsigned_nRows,
  const emxArray_real_T *costab, const emxArray_real_T *sintab, emxArray_creal_T
  *y);

/* Function Definitions */

/*
 * Arguments    : const emxArray_creal_T *x
 *                int unsigned_nRows
 *                const emxArray_real_T *costab
 *                const emxArray_real_T *sintab
 *                emxArray_creal_T *y
 * Return Type  : void
 */
static void r2br_r2dit_trig_impl(const emxArray_creal_T *x, int unsigned_nRows,
  const emxArray_real_T *costab, const emxArray_real_T *sintab, emxArray_creal_T
  *y)
{
  int j;
  int nRowsD2;
  int nRowsD4;
  int iy;
  int iDelta;
  int ix;
  int ju;
  int i;
  boolean_T tst;
  double temp_re;
  double temp_im;
  double twid_re;
  double twid_im;
  int ihi;
  j = x->size[0];
  if (!(j < unsigned_nRows)) {
    j = unsigned_nRows;
  }

  nRowsD2 = unsigned_nRows / 2;
  nRowsD4 = nRowsD2 / 2;
  iy = y->size[0];
  y->size[0] = unsigned_nRows;
  emxEnsureCapacity_creal_T1(y, iy);
  if (unsigned_nRows > x->size[0]) {
    iDelta = y->size[0];
    iy = y->size[0];
    y->size[0] = iDelta;
    emxEnsureCapacity_creal_T1(y, iy);
    for (iy = 0; iy < iDelta; iy++) {
      y->data[iy].re = 0.0;
      y->data[iy].im = 0.0;
    }
  }

  ix = 0;
  ju = 0;
  iy = 0;
  for (i = 1; i < j; i++) {
    y->data[iy] = x->data[ix];
    iDelta = unsigned_nRows;
    tst = true;
    while (tst) {
      iDelta >>= 1;
      ju ^= iDelta;
      tst = ((ju & iDelta) == 0);
    }

    iy = ju;
    ix++;
  }

  y->data[iy] = x->data[ix];
  if (unsigned_nRows > 1) {
    for (i = 0; i <= unsigned_nRows - 2; i += 2) {
      temp_re = y->data[i + 1].re;
      temp_im = y->data[i + 1].im;
      y->data[i + 1].re = y->data[i].re - y->data[i + 1].re;
      y->data[i + 1].im = y->data[i].im - y->data[i + 1].im;
      y->data[i].re += temp_re;
      y->data[i].im += temp_im;
    }
  }

  iDelta = 2;
  iy = 4;
  ix = 1 + ((nRowsD4 - 1) << 2);
  while (nRowsD4 > 0) {
    for (i = 0; i < ix; i += iy) {
      temp_re = y->data[i + iDelta].re;
      temp_im = y->data[i + iDelta].im;
      y->data[i + iDelta].re = y->data[i].re - temp_re;
      y->data[i + iDelta].im = y->data[i].im - temp_im;
      y->data[i].re += temp_re;
      y->data[i].im += temp_im;
    }

    ju = 1;
    for (j = nRowsD4; j < nRowsD2; j += nRowsD4) {
      twid_re = costab->data[j];
      twid_im = sintab->data[j];
      i = ju;
      ihi = ju + ix;
      while (i < ihi) {
        temp_re = twid_re * y->data[i + iDelta].re - twid_im * y->data[i +
          iDelta].im;
        temp_im = twid_re * y->data[i + iDelta].im + twid_im * y->data[i +
          iDelta].re;
        y->data[i + iDelta].re = y->data[i].re - temp_re;
        y->data[i + iDelta].im = y->data[i].im - temp_im;
        y->data[i].re += temp_re;
        y->data[i].im += temp_im;
        i += iy;
      }

      ju++;
    }

    nRowsD4 /= 2;
    iDelta = iy;
    iy += iy;
    ix -= iDelta;
  }
}

/*
 * Arguments    : const emxArray_real_T *x
 *                int xoffInit
 *                int nfft
 *                int nRows
 *                const emxArray_real_T *costab
 *                const emxArray_real_T *sintab
 *                const emxArray_real_T *costabinv
 *                const emxArray_real_T *sintabinv
 *                const emxArray_creal_T *wwc
 *                emxArray_creal_T *y
 * Return Type  : void
 */
void bluestein(const emxArray_real_T *x, int xoffInit, int nfft, int nRows,
               const emxArray_real_T *costab, const emxArray_real_T *sintab,
               const emxArray_real_T *costabinv, const emxArray_real_T
               *sintabinv, const emxArray_creal_T *wwc, emxArray_creal_T *y)
{
  int minNrowsNx;
  int xidx;
  int ju;
  double r;
  double twid_im;
  int istart;
  emxArray_creal_T *fy;
  int nRowsD2;
  int nRowsD4;
  int i;
  boolean_T tst;
  double temp_re;
  double temp_im;
  emxArray_creal_T *fv;
  int j;
  double fv_re;
  double fv_im;
  int ihi;
  double wwc_im;
  double b_fv_re;
  minNrowsNx = x->size[0];
  if (nRows < minNrowsNx) {
    minNrowsNx = nRows;
  }

  xidx = y->size[0];
  y->size[0] = nRows;
  emxEnsureCapacity_creal_T1(y, xidx);
  if (nRows > x->size[0]) {
    ju = y->size[0];
    xidx = y->size[0];
    y->size[0] = ju;
    emxEnsureCapacity_creal_T1(y, xidx);
    for (xidx = 0; xidx < ju; xidx++) {
      y->data[xidx].re = 0.0;
      y->data[xidx].im = 0.0;
    }
  }

  xidx = xoffInit;
  for (ju = 0; ju < minNrowsNx; ju++) {
    r = wwc->data[(nRows + ju) - 1].re;
    twid_im = wwc->data[(nRows + ju) - 1].im;
    y->data[ju].re = r * x->data[xidx];
    y->data[ju].im = twid_im * -x->data[xidx];
    xidx++;
  }

  while (minNrowsNx + 1 <= nRows) {
    y->data[minNrowsNx].re = 0.0;
    y->data[minNrowsNx].im = 0.0;
    minNrowsNx++;
  }

  istart = y->size[0];
  if (!(istart < nfft)) {
    istart = nfft;
  }

  emxInit_creal_T1(&fy, 1);
  nRowsD2 = nfft / 2;
  nRowsD4 = nRowsD2 / 2;
  xidx = fy->size[0];
  fy->size[0] = nfft;
  emxEnsureCapacity_creal_T1(fy, xidx);
  if (nfft > y->size[0]) {
    ju = fy->size[0];
    xidx = fy->size[0];
    fy->size[0] = ju;
    emxEnsureCapacity_creal_T1(fy, xidx);
    for (xidx = 0; xidx < ju; xidx++) {
      fy->data[xidx].re = 0.0;
      fy->data[xidx].im = 0.0;
    }
  }

  minNrowsNx = 0;
  ju = 0;
  xidx = 0;
  for (i = 1; i < istart; i++) {
    fy->data[xidx] = y->data[minNrowsNx];
    xidx = nfft;
    tst = true;
    while (tst) {
      xidx >>= 1;
      ju ^= xidx;
      tst = ((ju & xidx) == 0);
    }

    xidx = ju;
    minNrowsNx++;
  }

  fy->data[xidx] = y->data[minNrowsNx];
  if (nfft > 1) {
    for (i = 0; i <= nfft - 2; i += 2) {
      temp_re = fy->data[i + 1].re;
      temp_im = fy->data[i + 1].im;
      fy->data[i + 1].re = fy->data[i].re - fy->data[i + 1].re;
      fy->data[i + 1].im = fy->data[i].im - fy->data[i + 1].im;
      fy->data[i].re += temp_re;
      fy->data[i].im += temp_im;
    }
  }

  xidx = 2;
  minNrowsNx = 4;
  ju = 1 + ((nRowsD4 - 1) << 2);
  while (nRowsD4 > 0) {
    for (i = 0; i < ju; i += minNrowsNx) {
      temp_re = fy->data[i + xidx].re;
      temp_im = fy->data[i + xidx].im;
      fy->data[i + xidx].re = fy->data[i].re - temp_re;
      fy->data[i + xidx].im = fy->data[i].im - temp_im;
      fy->data[i].re += temp_re;
      fy->data[i].im += temp_im;
    }

    istart = 1;
    for (j = nRowsD4; j < nRowsD2; j += nRowsD4) {
      r = costab->data[j];
      twid_im = sintab->data[j];
      i = istart;
      ihi = istart + ju;
      while (i < ihi) {
        temp_re = r * fy->data[i + xidx].re - twid_im * fy->data[i + xidx].im;
        temp_im = r * fy->data[i + xidx].im + twid_im * fy->data[i + xidx].re;
        fy->data[i + xidx].re = fy->data[i].re - temp_re;
        fy->data[i + xidx].im = fy->data[i].im - temp_im;
        fy->data[i].re += temp_re;
        fy->data[i].im += temp_im;
        i += minNrowsNx;
      }

      istart++;
    }

    nRowsD4 /= 2;
    xidx = minNrowsNx;
    minNrowsNx += minNrowsNx;
    ju -= xidx;
  }

  emxInit_creal_T1(&fv, 1);
  r2br_r2dit_trig_impl(wwc, nfft, costab, sintab, fv);
  xidx = fy->size[0];
  emxEnsureCapacity_creal_T1(fy, xidx);
  ju = fy->size[0];
  for (xidx = 0; xidx < ju; xidx++) {
    r = fy->data[xidx].re;
    twid_im = fy->data[xidx].im;
    fv_re = fv->data[xidx].re;
    fv_im = fv->data[xidx].im;
    fy->data[xidx].re = r * fv_re - twid_im * fv_im;
    fy->data[xidx].im = r * fv_im + twid_im * fv_re;
  }

  r2br_r2dit_trig_impl(fy, nfft, costabinv, sintabinv, fv);
  emxFree_creal_T(&fy);
  if (fv->size[0] > 1) {
    r = 1.0 / (double)fv->size[0];
    xidx = fv->size[0];
    emxEnsureCapacity_creal_T1(fv, xidx);
    ju = fv->size[0];
    for (xidx = 0; xidx < ju; xidx++) {
      fv->data[xidx].re *= r;
      fv->data[xidx].im *= r;
    }
  }

  xidx = 0;
  for (ju = nRows - 1; ju < wwc->size[0]; ju++) {
    r = wwc->data[ju].re;
    fv_re = fv->data[ju].re;
    twid_im = wwc->data[ju].im;
    fv_im = fv->data[ju].im;
    temp_re = wwc->data[ju].re;
    temp_im = fv->data[ju].im;
    wwc_im = wwc->data[ju].im;
    b_fv_re = fv->data[ju].re;
    y->data[xidx].re = r * fv_re + twid_im * fv_im;
    y->data[xidx].im = temp_re * temp_im - wwc_im * b_fv_re;
    xidx++;
  }

  emxFree_creal_T(&fv);
}

/*
 * Arguments    : int nRows
 *                boolean_T useRadix2
 *                emxArray_real_T *costab
 *                emxArray_real_T *sintab
 *                emxArray_real_T *sintabinv
 * Return Type  : void
 */
void generate_twiddle_tables(int nRows, boolean_T useRadix2, emxArray_real_T
  *costab, emxArray_real_T *sintab, emxArray_real_T *sintabinv)
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
  emxEnsureCapacity_real_T(costab1q, nd2);
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
    emxEnsureCapacity_real_T(costab, nd2);
    nd2 = sintab->size[0] * sintab->size[1];
    sintab->size[0] = 1;
    sintab->size[1] = n2 + 1;
    emxEnsureCapacity_real_T(sintab, nd2);
    costab->data[0] = 1.0;
    sintab->data[0] = 0.0;
    nd2 = sintabinv->size[0] * sintabinv->size[1];
    sintabinv->size[0] = 1;
    sintabinv->size[1] = n2 + 1;
    emxEnsureCapacity_real_T(sintabinv, nd2);
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
    emxEnsureCapacity_real_T(costab, nd2);
    nd2 = sintab->size[0] * sintab->size[1];
    sintab->size[0] = 1;
    sintab->size[1] = n2 + 1;
    emxEnsureCapacity_real_T(sintab, nd2);
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
    emxEnsureCapacity_real_T(sintabinv, nd2);
  }

  emxFree_real_T(&costab1q);
}

/*
 * File trailer for fft1.c
 *
 * [EOF]
 */
