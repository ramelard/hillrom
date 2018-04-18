/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: fft.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 18-Apr-2018 17:29:29
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "extract_vitals_tk1.h"
#include "fft.h"
#include "extract_vitals_tk1_emxutil.h"
#include "bluestein_setup.h"

/* Function Declarations */
static void dobluesteinfft(const emxArray_real_T *x, int N2, int n1, const
  emxArray_real_T *costab, const emxArray_real_T *sintab, const emxArray_real_T *
  sintabinv, emxArray_creal_T *y);
static void generate_twiddle_tables(int nRows, boolean_T useRadix2,
  emxArray_real_T *costab, emxArray_real_T *sintab, emxArray_real_T *sintabinv);
static void r2br_r2dit_trig(const emxArray_real_T *x, int n1_unsigned, const
  emxArray_real_T *costab, const emxArray_real_T *sintab, emxArray_creal_T *y);

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
  int i6;
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
  for (i6 = 0; i6 < 2; i6++) {
    sx[i6] = (unsigned int)x->size[i6];
  }

  emxInit_creal_T1(&wwc, 1);
  bluestein_setup(n1, wwc);
  for (i6 = 0; i6 < 2; i6++) {
    sz[i6] = x->size[i6];
  }

  i6 = y->size[0] * y->size[1];
  y->size[0] = n1;
  y->size[1] = sz[1];
  emxEnsureCapacity((emxArray__common *)y, i6, (int)sizeof(creal_T));
  if (n1 > x->size[0]) {
    i6 = y->size[0] * y->size[1];
    emxEnsureCapacity((emxArray__common *)y, i6, (int)sizeof(creal_T));
    minNrowsNx = y->size[1];
    for (i6 = 0; i6 < minNrowsNx; i6++) {
      xidx = y->size[0];
      for (b_k = 0; b_k < xidx; b_k++) {
        y->data[b_k + y->size[0] * i6].re = 0.0;
        y->data[b_k + y->size[0] * i6].im = 0.0;
      }
    }
  }

  k = 0;
  emxInit_creal_T1(&rwork, 1);
  emxInit_creal_T1(&fy, 1);
  emxInit_creal_T1(&fv, 1);
  while (k + 1 <= (int)sx[1]) {
    if (n1 <= x->size[0]) {
      minNrowsNx = n1;
    } else {
      minNrowsNx = x->size[0];
    }

    i6 = rwork->size[0];
    rwork->size[0] = n1;
    emxEnsureCapacity((emxArray__common *)rwork, i6, (int)sizeof(creal_T));
    xidx = x->size[0];
    if (n1 > xidx) {
      xidx = rwork->size[0];
      i6 = rwork->size[0];
      rwork->size[0] = xidx;
      emxEnsureCapacity((emxArray__common *)rwork, i6, (int)sizeof(creal_T));
      for (i6 = 0; i6 < xidx; i6++) {
        rwork->data[i6].re = 0.0;
        rwork->data[i6].im = 0.0;
      }
    }

    xidx = k * x->size[0];
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
    b_r2br_r2dit_trig(wwc, N2, costab, sintab, fv);
    i6 = fy->size[0];
    emxEnsureCapacity((emxArray__common *)fy, i6, (int)sizeof(creal_T));
    minNrowsNx = fy->size[0];
    for (i6 = 0; i6 < minNrowsNx; i6++) {
      wwc_re = fy->data[i6].re;
      wwc_im = fy->data[i6].im;
      fv_re = fv->data[i6].re;
      fv_im = fv->data[i6].im;
      fy->data[i6].re = wwc_re * fv_re - wwc_im * fv_im;
      fy->data[i6].im = wwc_re * fv_im + wwc_im * fv_re;
    }

    c_r2br_r2dit_trig(fy, N2, costab, sintabinv, fv);
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
  emxInit_real_T1(&costab1q, 2);
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
 *                int n1_unsigned
 *                const emxArray_real_T *costab
 *                const emxArray_real_T *sintab
 *                emxArray_creal_T *y
 * Return Type  : void
 */
static void r2br_r2dit_trig(const emxArray_real_T *x, int n1_unsigned, const
  emxArray_real_T *costab, const emxArray_real_T *sintab, emxArray_creal_T *y)
{
  unsigned int sx[2];
  int ju;
  int sz[2];
  int k;
  emxArray_creal_T *rwork;
  int iy;
  int iDelta2;
  int j;
  int ix;
  int nRowsD2;
  int nRowsD4;
  int i;
  boolean_T tst;
  double temp_re;
  double temp_im;
  double twid_re;
  double twid_im;
  int ihi;
  for (ju = 0; ju < 2; ju++) {
    sx[ju] = (unsigned int)x->size[ju];
  }

  for (ju = 0; ju < 2; ju++) {
    sz[ju] = x->size[ju];
  }

  ju = y->size[0] * y->size[1];
  y->size[0] = n1_unsigned;
  y->size[1] = sz[1];
  emxEnsureCapacity((emxArray__common *)y, ju, (int)sizeof(creal_T));
  if (n1_unsigned > x->size[0]) {
    ju = y->size[0] * y->size[1];
    emxEnsureCapacity((emxArray__common *)y, ju, (int)sizeof(creal_T));
    iy = y->size[1];
    for (ju = 0; ju < iy; ju++) {
      iDelta2 = y->size[0];
      for (ix = 0; ix < iDelta2; ix++) {
        y->data[ix + y->size[0] * ju].re = 0.0;
        y->data[ix + y->size[0] * ju].im = 0.0;
      }
    }
  }

  k = 0;
  emxInit_creal_T1(&rwork, 1);
  while (k + 1 <= (int)sx[1]) {
    if (x->size[0] <= n1_unsigned) {
      j = x->size[0];
    } else {
      j = n1_unsigned;
    }

    nRowsD2 = n1_unsigned / 2;
    nRowsD4 = nRowsD2 / 2;
    ju = rwork->size[0];
    rwork->size[0] = n1_unsigned;
    emxEnsureCapacity((emxArray__common *)rwork, ju, (int)sizeof(creal_T));
    iy = x->size[0];
    if (n1_unsigned > iy) {
      iy = rwork->size[0];
      ju = rwork->size[0];
      rwork->size[0] = iy;
      emxEnsureCapacity((emxArray__common *)rwork, ju, (int)sizeof(creal_T));
      for (ju = 0; ju < iy; ju++) {
        rwork->data[ju].re = 0.0;
        rwork->data[ju].im = 0.0;
      }
    }

    ix = k * x->size[0];
    ju = 0;
    iy = 0;
    for (i = 1; i < j; i++) {
      rwork->data[iy].re = x->data[ix];
      rwork->data[iy].im = 0.0;
      iDelta2 = n1_unsigned;
      tst = true;
      while (tst) {
        iDelta2 >>= 1;
        ju ^= iDelta2;
        tst = ((ju & iDelta2) == 0);
      }

      iy = ju;
      ix++;
    }

    rwork->data[iy].re = x->data[ix];
    rwork->data[iy].im = 0.0;
    if (n1_unsigned > 1) {
      for (i = 0; i <= n1_unsigned - 2; i += 2) {
        temp_re = rwork->data[i + 1].re;
        temp_im = rwork->data[i + 1].im;
        rwork->data[i + 1].re = rwork->data[i].re - rwork->data[i + 1].re;
        rwork->data[i + 1].im = rwork->data[i].im - rwork->data[i + 1].im;
        rwork->data[i].re += temp_re;
        rwork->data[i].im += temp_im;
      }
    }

    iy = 2;
    iDelta2 = 4;
    ix = 1 + ((nRowsD4 - 1) << 2);
    while (nRowsD4 > 0) {
      for (i = 0; i < ix; i += iDelta2) {
        temp_re = rwork->data[i + iy].re;
        temp_im = rwork->data[i + iy].im;
        rwork->data[i + iy].re = rwork->data[i].re - temp_re;
        rwork->data[i + iy].im = rwork->data[i].im - temp_im;
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
          temp_re = twid_re * rwork->data[i + iy].re - twid_im * rwork->data[i +
            iy].im;
          temp_im = twid_re * rwork->data[i + iy].im + twid_im * rwork->data[i +
            iy].re;
          rwork->data[i + iy].re = rwork->data[i].re - temp_re;
          rwork->data[i + iy].im = rwork->data[i].im - temp_im;
          rwork->data[i].re += temp_re;
          rwork->data[i].im += temp_im;
          i += iDelta2;
        }

        ju++;
      }

      nRowsD4 /= 2;
      iy = iDelta2;
      iDelta2 <<= 1;
      ix -= iy;
    }

    for (iy = 0; iy + 1 <= n1_unsigned; iy++) {
      y->data[iy + y->size[0] * k] = rwork->data[iy];
    }

    k++;
  }

  emxFree_creal_T(&rwork);
}

/*
 * Arguments    : const emxArray_real_T *x
 *                emxArray_creal_T *y
 * Return Type  : void
 */
void b_fft(const emxArray_real_T *x, emxArray_creal_T *y)
{
  emxArray_real_T *costab;
  int sz[2];
  emxArray_real_T *sintab;
  int N2blue;
  emxArray_real_T *sintabinv;
  boolean_T useRadix2;
  int nRows;
  if ((x->size[0] == 0) || (x->size[1] == 0)) {
    for (N2blue = 0; N2blue < 2; N2blue++) {
      sz[N2blue] = x->size[N2blue];
    }

    sz[0] = x->size[0];
    N2blue = y->size[0] * y->size[1];
    y->size[0] = sz[0];
    y->size[1] = sz[1];
    emxEnsureCapacity((emxArray__common *)y, N2blue, (int)sizeof(creal_T));
  } else {
    emxInit_real_T1(&costab, 2);
    emxInit_real_T1(&sintab, 2);
    emxInit_real_T1(&sintabinv, 2);
    useRadix2 = ((x->size[0] & (x->size[0] - 1)) == 0);
    get_algo_sizes(x->size[0], useRadix2, &N2blue, &nRows);
    generate_twiddle_tables(nRows, useRadix2, costab, sintab, sintabinv);
    if (useRadix2) {
      r2br_r2dit_trig(x, x->size[0], costab, sintab, y);
    } else {
      dobluesteinfft(x, N2blue, x->size[0], costab, sintab, sintabinv, y);
    }

    emxFree_real_T(&sintabinv);
    emxFree_real_T(&sintab);
    emxFree_real_T(&costab);
  }
}

/*
 * Arguments    : const emxArray_creal_T *x
 *                int n1_unsigned
 *                const emxArray_real_T *costab
 *                const emxArray_real_T *sintab
 *                emxArray_creal_T *y
 * Return Type  : void
 */
void b_r2br_r2dit_trig(const emxArray_creal_T *x, int n1_unsigned, const
  emxArray_real_T *costab, const emxArray_real_T *sintab, emxArray_creal_T *y)
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
  if (x->size[0] <= n1_unsigned) {
    j = x->size[0];
  } else {
    j = n1_unsigned;
  }

  nRowsD2 = n1_unsigned / 2;
  nRowsD4 = nRowsD2 / 2;
  iy = y->size[0];
  y->size[0] = n1_unsigned;
  emxEnsureCapacity((emxArray__common *)y, iy, (int)sizeof(creal_T));
  if (n1_unsigned > x->size[0]) {
    iDelta = y->size[0];
    iy = y->size[0];
    y->size[0] = iDelta;
    emxEnsureCapacity((emxArray__common *)y, iy, (int)sizeof(creal_T));
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
    iDelta = n1_unsigned;
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
  if (n1_unsigned > 1) {
    for (i = 0; i <= n1_unsigned - 2; i += 2) {
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
    iy <<= 1;
    ix -= iDelta;
  }
}

/*
 * Arguments    : const emxArray_creal_T *x
 *                double varargin_1
 *                emxArray_creal_T *y
 * Return Type  : void
 */
void c_fft(const emxArray_creal_T *x, double varargin_1, emxArray_creal_T *y)
{
  emxArray_real_T *costab1q;
  emxArray_real_T *costab;
  emxArray_real_T *sintab;
  emxArray_real_T *sintabinv;
  emxArray_creal_T *wwc;
  emxArray_creal_T *fy;
  emxArray_creal_T *fv;
  boolean_T useRadix2;
  int nInt2;
  int N2blue;
  int nd2;
  double e;
  int nRowsD4;
  int b_y;
  int k;
  int nInt2m1;
  double nt_im;
  double wwc_re;
  double fv_im;
  double wwc_im;
  double fv_re;
  double b_fv_re;
  double b_fv_im;
  emxInit_real_T1(&costab1q, 2);
  emxInit_real_T1(&costab, 2);
  emxInit_real_T1(&sintab, 2);
  emxInit_real_T1(&sintabinv, 2);
  emxInit_creal_T1(&wwc, 1);
  emxInit_creal_T1(&fy, 1);
  emxInit_creal_T1(&fv, 1);
  if ((x->size[0] == 0) || ((int)varargin_1 == 0)) {
    nInt2 = y->size[0];
    y->size[0] = (int)varargin_1;
    emxEnsureCapacity((emxArray__common *)y, nInt2, (int)sizeof(creal_T));
    if ((int)varargin_1 > x->size[0]) {
      b_y = y->size[0];
      nInt2 = y->size[0];
      y->size[0] = b_y;
      emxEnsureCapacity((emxArray__common *)y, nInt2, (int)sizeof(creal_T));
      for (nInt2 = 0; nInt2 < b_y; nInt2++) {
        y->data[nInt2].re = 0.0;
        y->data[nInt2].im = 0.0;
      }
    }
  } else {
    useRadix2 = (((int)varargin_1 & ((int)varargin_1 - 1)) == 0);
    get_algo_sizes((int)varargin_1, useRadix2, &N2blue, &nd2);
    e = 6.2831853071795862 / (double)nd2;
    nRowsD4 = nd2 / 2 / 2;
    nInt2 = costab1q->size[0] * costab1q->size[1];
    costab1q->size[0] = 1;
    costab1q->size[1] = nRowsD4 + 1;
    emxEnsureCapacity((emxArray__common *)costab1q, nInt2, (int)sizeof(double));
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
      nd2 = costab1q->size[1] - 1;
      nRowsD4 = (costab1q->size[1] - 1) << 1;
      nInt2 = costab->size[0] * costab->size[1];
      costab->size[0] = 1;
      costab->size[1] = nRowsD4 + 1;
      emxEnsureCapacity((emxArray__common *)costab, nInt2, (int)sizeof(double));
      nInt2 = sintab->size[0] * sintab->size[1];
      sintab->size[0] = 1;
      sintab->size[1] = nRowsD4 + 1;
      emxEnsureCapacity((emxArray__common *)sintab, nInt2, (int)sizeof(double));
      costab->data[0] = 1.0;
      sintab->data[0] = 0.0;
      nInt2 = sintabinv->size[0] * sintabinv->size[1];
      sintabinv->size[0] = 1;
      sintabinv->size[1] = nRowsD4 + 1;
      emxEnsureCapacity((emxArray__common *)sintabinv, nInt2, (int)sizeof(double));
      for (k = 1; k <= nd2; k++) {
        sintabinv->data[k] = costab1q->data[nd2 - k];
      }

      for (k = costab1q->size[1]; k <= nRowsD4; k++) {
        sintabinv->data[k] = costab1q->data[k - nd2];
      }

      for (k = 1; k <= nd2; k++) {
        costab->data[k] = costab1q->data[k];
        sintab->data[k] = -costab1q->data[nd2 - k];
      }

      for (k = costab1q->size[1]; k <= nRowsD4; k++) {
        costab->data[k] = -costab1q->data[nRowsD4 - k];
        sintab->data[k] = -costab1q->data[k - nd2];
      }
    } else {
      nd2 = costab1q->size[1] - 1;
      nRowsD4 = (costab1q->size[1] - 1) << 1;
      nInt2 = costab->size[0] * costab->size[1];
      costab->size[0] = 1;
      costab->size[1] = nRowsD4 + 1;
      emxEnsureCapacity((emxArray__common *)costab, nInt2, (int)sizeof(double));
      nInt2 = sintab->size[0] * sintab->size[1];
      sintab->size[0] = 1;
      sintab->size[1] = nRowsD4 + 1;
      emxEnsureCapacity((emxArray__common *)sintab, nInt2, (int)sizeof(double));
      costab->data[0] = 1.0;
      sintab->data[0] = 0.0;
      for (k = 1; k <= nd2; k++) {
        costab->data[k] = costab1q->data[k];
        sintab->data[k] = -costab1q->data[nd2 - k];
      }

      for (k = costab1q->size[1]; k <= nRowsD4; k++) {
        costab->data[k] = -costab1q->data[nRowsD4 - k];
        sintab->data[k] = -costab1q->data[k - nd2];
      }

      nInt2 = sintabinv->size[0] * sintabinv->size[1];
      sintabinv->size[0] = 1;
      sintabinv->size[1] = 0;
      emxEnsureCapacity((emxArray__common *)sintabinv, nInt2, (int)sizeof(double));
    }

    if (useRadix2) {
      b_r2br_r2dit_trig(x, (int)varargin_1, costab, sintab, y);
    } else {
      nInt2m1 = ((int)varargin_1 + (int)varargin_1) - 1;
      nInt2 = wwc->size[0];
      wwc->size[0] = nInt2m1;
      emxEnsureCapacity((emxArray__common *)wwc, nInt2, (int)sizeof(creal_T));
      nd2 = (int)varargin_1;
      nRowsD4 = 0;
      wwc->data[(int)varargin_1 - 1].re = 1.0;
      wwc->data[(int)varargin_1 - 1].im = 0.0;
      nInt2 = (int)varargin_1 << 1;
      for (k = 1; k < (int)varargin_1; k++) {
        b_y = (k << 1) - 1;
        if (nInt2 - nRowsD4 <= b_y) {
          nRowsD4 += b_y - nInt2;
        } else {
          nRowsD4 += b_y;
        }

        nt_im = -3.1415926535897931 * (double)nRowsD4 / (double)(int)varargin_1;
        if (nt_im == 0.0) {
          e = 1.0;
          nt_im = 0.0;
        } else {
          e = cos(nt_im);
          nt_im = sin(nt_im);
        }

        wwc->data[nd2 - 2].re = e;
        wwc->data[nd2 - 2].im = -nt_im;
        nd2--;
      }

      nd2 = 0;
      for (k = nInt2m1 - 1; k >= (int)varargin_1; k--) {
        wwc->data[k] = wwc->data[nd2];
        nd2++;
      }

      if ((int)varargin_1 <= x->size[0]) {
        nRowsD4 = (int)varargin_1;
      } else {
        nRowsD4 = x->size[0];
      }

      nInt2 = y->size[0];
      y->size[0] = (int)varargin_1;
      emxEnsureCapacity((emxArray__common *)y, nInt2, (int)sizeof(creal_T));
      if ((int)varargin_1 > x->size[0]) {
        b_y = y->size[0];
        nInt2 = y->size[0];
        y->size[0] = b_y;
        emxEnsureCapacity((emxArray__common *)y, nInt2, (int)sizeof(creal_T));
        for (nInt2 = 0; nInt2 < b_y; nInt2++) {
          y->data[nInt2].re = 0.0;
          y->data[nInt2].im = 0.0;
        }
      }

      nd2 = 0;
      for (k = 0; k + 1 <= nRowsD4; k++) {
        e = wwc->data[((int)varargin_1 + k) - 1].re;
        nt_im = wwc->data[((int)varargin_1 + k) - 1].im;
        wwc_re = x->data[nd2].re;
        fv_im = x->data[nd2].im;
        wwc_im = x->data[nd2].im;
        fv_re = x->data[nd2].re;
        y->data[k].re = e * wwc_re + nt_im * fv_im;
        y->data[k].im = e * wwc_im - nt_im * fv_re;
        nd2++;
      }

      while (nRowsD4 + 1 <= (int)varargin_1) {
        y->data[nRowsD4].re = 0.0;
        y->data[nRowsD4].im = 0.0;
        nRowsD4++;
      }

      r2br_r2dit_trig_impl(y, N2blue, costab, sintab, fy);
      b_r2br_r2dit_trig(wwc, N2blue, costab, sintab, fv);
      nInt2 = fy->size[0];
      emxEnsureCapacity((emxArray__common *)fy, nInt2, (int)sizeof(creal_T));
      nd2 = fy->size[0];
      for (nInt2 = 0; nInt2 < nd2; nInt2++) {
        e = fy->data[nInt2].re;
        nt_im = fy->data[nInt2].im;
        b_fv_re = fv->data[nInt2].re;
        b_fv_im = fv->data[nInt2].im;
        fy->data[nInt2].re = e * b_fv_re - nt_im * b_fv_im;
        fy->data[nInt2].im = e * b_fv_im + nt_im * b_fv_re;
      }

      c_r2br_r2dit_trig(fy, N2blue, costab, sintabinv, fv);
      nd2 = 0;
      for (k = (int)varargin_1 - 1; k + 1 <= wwc->size[0]; k++) {
        e = wwc->data[k].re;
        b_fv_re = fv->data[k].re;
        nt_im = wwc->data[k].im;
        b_fv_im = fv->data[k].im;
        wwc_re = wwc->data[k].re;
        fv_im = fv->data[k].im;
        wwc_im = wwc->data[k].im;
        fv_re = fv->data[k].re;
        y->data[nd2].re = e * b_fv_re + nt_im * b_fv_im;
        y->data[nd2].im = wwc_re * fv_im - wwc_im * fv_re;
        nd2++;
      }
    }
  }

  emxFree_creal_T(&fv);
  emxFree_creal_T(&fy);
  emxFree_creal_T(&wwc);
  emxFree_real_T(&sintabinv);
  emxFree_real_T(&sintab);
  emxFree_real_T(&costab);
  emxFree_real_T(&costab1q);
}

/*
 * Arguments    : const emxArray_creal_T *x
 *                int n1_unsigned
 *                const emxArray_real_T *costab
 *                const emxArray_real_T *sintab
 *                emxArray_creal_T *y
 * Return Type  : void
 */
void c_r2br_r2dit_trig(const emxArray_creal_T *x, int n1_unsigned, const
  emxArray_real_T *costab, const emxArray_real_T *sintab, emxArray_creal_T *y)
{
  int j;
  int nRowsD2;
  int nRowsD4;
  int iDelta2;
  int iy;
  int ix;
  int ju;
  int i;
  boolean_T tst;
  double temp_re;
  double temp_im;
  double r;
  double twid_im;
  int ihi;
  if (x->size[0] <= n1_unsigned) {
    j = x->size[0];
  } else {
    j = n1_unsigned;
  }

  nRowsD2 = n1_unsigned / 2;
  nRowsD4 = nRowsD2 / 2;
  iDelta2 = y->size[0];
  y->size[0] = n1_unsigned;
  emxEnsureCapacity((emxArray__common *)y, iDelta2, (int)sizeof(creal_T));
  if (n1_unsigned > x->size[0]) {
    iy = y->size[0];
    iDelta2 = y->size[0];
    y->size[0] = iy;
    emxEnsureCapacity((emxArray__common *)y, iDelta2, (int)sizeof(creal_T));
    for (iDelta2 = 0; iDelta2 < iy; iDelta2++) {
      y->data[iDelta2].re = 0.0;
      y->data[iDelta2].im = 0.0;
    }
  }

  ix = 0;
  ju = 0;
  iy = 0;
  for (i = 1; i < j; i++) {
    y->data[iy] = x->data[ix];
    iDelta2 = n1_unsigned;
    tst = true;
    while (tst) {
      iDelta2 >>= 1;
      ju ^= iDelta2;
      tst = ((ju & iDelta2) == 0);
    }

    iy = ju;
    ix++;
  }

  y->data[iy] = x->data[ix];
  if (n1_unsigned > 1) {
    for (i = 0; i <= n1_unsigned - 2; i += 2) {
      temp_re = y->data[i + 1].re;
      temp_im = y->data[i + 1].im;
      y->data[i + 1].re = y->data[i].re - y->data[i + 1].re;
      y->data[i + 1].im = y->data[i].im - y->data[i + 1].im;
      y->data[i].re += temp_re;
      y->data[i].im += temp_im;
    }
  }

  iy = 2;
  iDelta2 = 4;
  ix = 1 + ((nRowsD4 - 1) << 2);
  while (nRowsD4 > 0) {
    for (i = 0; i < ix; i += iDelta2) {
      temp_re = y->data[i + iy].re;
      temp_im = y->data[i + iy].im;
      y->data[i + iy].re = y->data[i].re - temp_re;
      y->data[i + iy].im = y->data[i].im - temp_im;
      y->data[i].re += temp_re;
      y->data[i].im += temp_im;
    }

    ju = 1;
    for (j = nRowsD4; j < nRowsD2; j += nRowsD4) {
      r = costab->data[j];
      twid_im = sintab->data[j];
      i = ju;
      ihi = ju + ix;
      while (i < ihi) {
        temp_re = r * y->data[i + iy].re - twid_im * y->data[i + iy].im;
        temp_im = r * y->data[i + iy].im + twid_im * y->data[i + iy].re;
        y->data[i + iy].re = y->data[i].re - temp_re;
        y->data[i + iy].im = y->data[i].im - temp_im;
        y->data[i].re += temp_re;
        y->data[i].im += temp_im;
        i += iDelta2;
      }

      ju++;
    }

    nRowsD4 /= 2;
    iy = iDelta2;
    iDelta2 <<= 1;
    ix -= iy;
  }

  if (y->size[0] > 1) {
    r = 1.0 / (double)y->size[0];
    iDelta2 = y->size[0];
    emxEnsureCapacity((emxArray__common *)y, iDelta2, (int)sizeof(creal_T));
    iy = y->size[0];
    for (iDelta2 = 0; iDelta2 < iy; iDelta2++) {
      y->data[iDelta2].re *= r;
      y->data[iDelta2].im *= r;
    }
  }
}

/*
 * Arguments    : const emxArray_real_T *x
 *                double varargin_1
 *                emxArray_creal_T *y
 * Return Type  : void
 */
void d_fft(const emxArray_real_T *x, double varargin_1, emxArray_creal_T *y)
{
  emxArray_real_T *costab1q;
  emxArray_real_T *costab;
  emxArray_real_T *sintab;
  emxArray_real_T *sintabinv;
  emxArray_creal_T *wwc;
  emxArray_creal_T *fy;
  emxArray_creal_T *fv;
  boolean_T useRadix2;
  int ju;
  int istart;
  int nd2;
  double e;
  int nRowsD4;
  int n;
  int minNrowsNx;
  int nRowsD2;
  int i;
  double twid_im;
  double temp_re;
  double temp_im;
  double fv_re;
  double fv_im;
  double wwc_im;
  double b_fv_re;
  int ihi;
  emxInit_real_T1(&costab1q, 2);
  emxInit_real_T1(&costab, 2);
  emxInit_real_T1(&sintab, 2);
  emxInit_real_T1(&sintabinv, 2);
  emxInit_creal_T1(&wwc, 1);
  emxInit_creal_T1(&fy, 1);
  emxInit_creal_T1(&fv, 1);
  if ((x->size[0] == 0) || ((int)varargin_1 == 0)) {
    ju = y->size[0];
    y->size[0] = (int)varargin_1;
    emxEnsureCapacity((emxArray__common *)y, ju, (int)sizeof(creal_T));
    if ((int)varargin_1 > x->size[0]) {
      nd2 = y->size[0];
      ju = y->size[0];
      y->size[0] = nd2;
      emxEnsureCapacity((emxArray__common *)y, ju, (int)sizeof(creal_T));
      for (ju = 0; ju < nd2; ju++) {
        y->data[ju].re = 0.0;
        y->data[ju].im = 0.0;
      }
    }
  } else {
    useRadix2 = (((int)varargin_1 & ((int)varargin_1 - 1)) == 0);
    get_algo_sizes((int)varargin_1, useRadix2, &istart, &nd2);
    e = 6.2831853071795862 / (double)nd2;
    nRowsD4 = nd2 / 2 / 2;
    ju = costab1q->size[0] * costab1q->size[1];
    costab1q->size[0] = 1;
    costab1q->size[1] = nRowsD4 + 1;
    emxEnsureCapacity((emxArray__common *)costab1q, ju, (int)sizeof(double));
    costab1q->data[0] = 1.0;
    nd2 = nRowsD4 / 2;
    for (ju = 1; ju <= nd2; ju++) {
      costab1q->data[ju] = cos(e * (double)ju);
    }

    for (ju = nd2 + 1; ju < nRowsD4; ju++) {
      costab1q->data[ju] = sin(e * (double)(nRowsD4 - ju));
    }

    costab1q->data[nRowsD4] = 0.0;
    if (!useRadix2) {
      n = costab1q->size[1] - 1;
      nd2 = (costab1q->size[1] - 1) << 1;
      ju = costab->size[0] * costab->size[1];
      costab->size[0] = 1;
      costab->size[1] = nd2 + 1;
      emxEnsureCapacity((emxArray__common *)costab, ju, (int)sizeof(double));
      ju = sintab->size[0] * sintab->size[1];
      sintab->size[0] = 1;
      sintab->size[1] = nd2 + 1;
      emxEnsureCapacity((emxArray__common *)sintab, ju, (int)sizeof(double));
      costab->data[0] = 1.0;
      sintab->data[0] = 0.0;
      ju = sintabinv->size[0] * sintabinv->size[1];
      sintabinv->size[0] = 1;
      sintabinv->size[1] = nd2 + 1;
      emxEnsureCapacity((emxArray__common *)sintabinv, ju, (int)sizeof(double));
      for (ju = 1; ju <= n; ju++) {
        sintabinv->data[ju] = costab1q->data[n - ju];
      }

      for (ju = costab1q->size[1]; ju <= nd2; ju++) {
        sintabinv->data[ju] = costab1q->data[ju - n];
      }

      for (ju = 1; ju <= n; ju++) {
        costab->data[ju] = costab1q->data[ju];
        sintab->data[ju] = -costab1q->data[n - ju];
      }

      for (ju = costab1q->size[1]; ju <= nd2; ju++) {
        costab->data[ju] = -costab1q->data[nd2 - ju];
        sintab->data[ju] = -costab1q->data[ju - n];
      }
    } else {
      n = costab1q->size[1] - 1;
      nd2 = (costab1q->size[1] - 1) << 1;
      ju = costab->size[0] * costab->size[1];
      costab->size[0] = 1;
      costab->size[1] = nd2 + 1;
      emxEnsureCapacity((emxArray__common *)costab, ju, (int)sizeof(double));
      ju = sintab->size[0] * sintab->size[1];
      sintab->size[0] = 1;
      sintab->size[1] = nd2 + 1;
      emxEnsureCapacity((emxArray__common *)sintab, ju, (int)sizeof(double));
      costab->data[0] = 1.0;
      sintab->data[0] = 0.0;
      for (ju = 1; ju <= n; ju++) {
        costab->data[ju] = costab1q->data[ju];
        sintab->data[ju] = -costab1q->data[n - ju];
      }

      for (ju = costab1q->size[1]; ju <= nd2; ju++) {
        costab->data[ju] = -costab1q->data[nd2 - ju];
        sintab->data[ju] = -costab1q->data[ju - n];
      }

      ju = sintabinv->size[0] * sintabinv->size[1];
      sintabinv->size[0] = 1;
      sintabinv->size[1] = 0;
      emxEnsureCapacity((emxArray__common *)sintabinv, ju, (int)sizeof(double));
    }

    if (useRadix2) {
      if (x->size[0] <= (int)varargin_1) {
        istart = x->size[0];
      } else {
        istart = (int)varargin_1;
      }

      nRowsD2 = (int)varargin_1 / 2;
      nRowsD4 = nRowsD2 / 2;
      ju = y->size[0];
      y->size[0] = (int)varargin_1;
      emxEnsureCapacity((emxArray__common *)y, ju, (int)sizeof(creal_T));
      if ((int)varargin_1 > x->size[0]) {
        nd2 = y->size[0];
        ju = y->size[0];
        y->size[0] = nd2;
        emxEnsureCapacity((emxArray__common *)y, ju, (int)sizeof(creal_T));
        for (ju = 0; ju < nd2; ju++) {
          y->data[ju].re = 0.0;
          y->data[ju].im = 0.0;
        }
      }

      minNrowsNx = 0;
      ju = 0;
      nd2 = 0;
      for (i = 1; i < istart; i++) {
        y->data[nd2].re = x->data[minNrowsNx];
        y->data[nd2].im = 0.0;
        n = (int)varargin_1;
        useRadix2 = true;
        while (useRadix2) {
          n >>= 1;
          ju ^= n;
          useRadix2 = ((ju & n) == 0);
        }

        nd2 = ju;
        minNrowsNx++;
      }

      y->data[nd2].re = x->data[minNrowsNx];
      y->data[nd2].im = 0.0;
      if ((int)varargin_1 > 1) {
        for (i = 0; i <= (int)varargin_1 - 2; i += 2) {
          temp_re = y->data[i + 1].re;
          temp_im = y->data[i + 1].im;
          y->data[i + 1].re = y->data[i].re - y->data[i + 1].re;
          y->data[i + 1].im = y->data[i].im - y->data[i + 1].im;
          y->data[i].re += temp_re;
          y->data[i].im += temp_im;
        }
      }

      nd2 = 2;
      minNrowsNx = 4;
      ju = 1 + ((nRowsD4 - 1) << 2);
      while (nRowsD4 > 0) {
        for (i = 0; i < ju; i += minNrowsNx) {
          temp_re = y->data[i + nd2].re;
          temp_im = y->data[i + nd2].im;
          y->data[i + nd2].re = y->data[i].re - temp_re;
          y->data[i + nd2].im = y->data[i].im - temp_im;
          y->data[i].re += temp_re;
          y->data[i].im += temp_im;
        }

        istart = 1;
        for (n = nRowsD4; n < nRowsD2; n += nRowsD4) {
          e = costab->data[n];
          twid_im = sintab->data[n];
          i = istart;
          ihi = istart + ju;
          while (i < ihi) {
            temp_re = e * y->data[i + nd2].re - twid_im * y->data[i + nd2].im;
            temp_im = e * y->data[i + nd2].im + twid_im * y->data[i + nd2].re;
            y->data[i + nd2].re = y->data[i].re - temp_re;
            y->data[i + nd2].im = y->data[i].im - temp_im;
            y->data[i].re += temp_re;
            y->data[i].im += temp_im;
            i += minNrowsNx;
          }

          istart++;
        }

        nRowsD4 /= 2;
        nd2 = minNrowsNx;
        minNrowsNx <<= 1;
        ju -= nd2;
      }
    } else {
      bluestein_setup((int)varargin_1, wwc);
      if ((int)varargin_1 <= x->size[0]) {
        minNrowsNx = (int)varargin_1;
      } else {
        minNrowsNx = x->size[0];
      }

      ju = y->size[0];
      y->size[0] = (int)varargin_1;
      emxEnsureCapacity((emxArray__common *)y, ju, (int)sizeof(creal_T));
      if ((int)varargin_1 > x->size[0]) {
        nd2 = y->size[0];
        ju = y->size[0];
        y->size[0] = nd2;
        emxEnsureCapacity((emxArray__common *)y, ju, (int)sizeof(creal_T));
        for (ju = 0; ju < nd2; ju++) {
          y->data[ju].re = 0.0;
          y->data[ju].im = 0.0;
        }
      }

      nd2 = 0;
      for (ju = 0; ju + 1 <= minNrowsNx; ju++) {
        e = wwc->data[((int)varargin_1 + ju) - 1].re;
        twid_im = wwc->data[((int)varargin_1 + ju) - 1].im;
        y->data[ju].re = e * x->data[nd2];
        y->data[ju].im = twid_im * -x->data[nd2];
        nd2++;
      }

      while (minNrowsNx + 1 <= (int)varargin_1) {
        y->data[minNrowsNx].re = 0.0;
        y->data[minNrowsNx].im = 0.0;
        minNrowsNx++;
      }

      r2br_r2dit_trig_impl(y, istart, costab, sintab, fy);
      b_r2br_r2dit_trig(wwc, istart, costab, sintab, fv);
      ju = fy->size[0];
      emxEnsureCapacity((emxArray__common *)fy, ju, (int)sizeof(creal_T));
      nd2 = fy->size[0];
      for (ju = 0; ju < nd2; ju++) {
        e = fy->data[ju].re;
        twid_im = fy->data[ju].im;
        fv_re = fv->data[ju].re;
        fv_im = fv->data[ju].im;
        fy->data[ju].re = e * fv_re - twid_im * fv_im;
        fy->data[ju].im = e * fv_im + twid_im * fv_re;
      }

      c_r2br_r2dit_trig(fy, istart, costab, sintabinv, fv);
      nd2 = 0;
      for (ju = (int)varargin_1 - 1; ju + 1 <= wwc->size[0]; ju++) {
        e = wwc->data[ju].re;
        fv_re = fv->data[ju].re;
        twid_im = wwc->data[ju].im;
        fv_im = fv->data[ju].im;
        temp_re = wwc->data[ju].re;
        temp_im = fv->data[ju].im;
        wwc_im = wwc->data[ju].im;
        b_fv_re = fv->data[ju].re;
        y->data[nd2].re = e * fv_re + twid_im * fv_im;
        y->data[nd2].im = temp_re * temp_im - wwc_im * b_fv_re;
        nd2++;
      }
    }
  }

  emxFree_creal_T(&fv);
  emxFree_creal_T(&fy);
  emxFree_creal_T(&wwc);
  emxFree_real_T(&sintabinv);
  emxFree_real_T(&sintab);
  emxFree_real_T(&costab);
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
  boolean_T useRadix2;
  int sz[2];
  int N2blue;
  int nRows;
  int loop_ub;
  int i5;
  emxInit_real_T1(&costab, 2);
  emxInit_real_T1(&sintab, 2);
  emxInit_real_T1(&sintabinv, 2);
  if ((x->size[0] == 0) || (x->size[1] == 0) || ((int)varargin_1 == 0)) {
    for (N2blue = 0; N2blue < 2; N2blue++) {
      sz[N2blue] = x->size[N2blue];
    }

    N2blue = y->size[0] * y->size[1];
    y->size[0] = (int)varargin_1;
    y->size[1] = sz[1];
    emxEnsureCapacity((emxArray__common *)y, N2blue, (int)sizeof(creal_T));
    if ((int)varargin_1 > x->size[0]) {
      N2blue = y->size[0] * y->size[1];
      emxEnsureCapacity((emxArray__common *)y, N2blue, (int)sizeof(creal_T));
      nRows = y->size[1];
      for (N2blue = 0; N2blue < nRows; N2blue++) {
        loop_ub = y->size[0];
        for (i5 = 0; i5 < loop_ub; i5++) {
          y->data[i5 + y->size[0] * N2blue].re = 0.0;
          y->data[i5 + y->size[0] * N2blue].im = 0.0;
        }
      }
    }
  } else {
    useRadix2 = (((int)varargin_1 & ((int)varargin_1 - 1)) == 0);
    get_algo_sizes((int)varargin_1, useRadix2, &N2blue, &nRows);
    generate_twiddle_tables(nRows, useRadix2, costab, sintab, sintabinv);
    if (useRadix2) {
      r2br_r2dit_trig(x, (int)varargin_1, costab, sintab, y);
    } else {
      dobluesteinfft(x, N2blue, (int)varargin_1, costab, sintab, sintabinv, y);
    }
  }

  emxFree_real_T(&sintabinv);
  emxFree_real_T(&sintab);
  emxFree_real_T(&costab);
}

/*
 * Arguments    : int n1
 *                boolean_T useRadix2
 *                int *N2blue
 *                int *nRows
 * Return Type  : void
 */
void get_algo_sizes(int n1, boolean_T useRadix2, int *N2blue, int *nRows)
{
  int nn1m1;
  int pmax;
  int pmin;
  boolean_T exitg1;
  int p;
  int pow2p;
  *N2blue = 1;
  if (useRadix2) {
    *nRows = n1;
  } else {
    nn1m1 = (n1 + n1) - 1;
    pmax = 31;
    if (nn1m1 <= 1) {
      pmax = 0;
    } else {
      pmin = 0;
      exitg1 = false;
      while ((!exitg1) && (pmax - pmin > 1)) {
        p = (pmin + pmax) >> 1;
        pow2p = 1 << p;
        if (pow2p == nn1m1) {
          pmax = p;
          exitg1 = true;
        } else if (pow2p > nn1m1) {
          pmax = p;
        } else {
          pmin = p;
        }
      }
    }

    *N2blue = 1 << pmax;
    *nRows = *N2blue;
  }
}

/*
 * Arguments    : const emxArray_creal_T *x
 *                int unsigned_nRows
 *                const emxArray_real_T *costab
 *                const emxArray_real_T *sintab
 *                emxArray_creal_T *y
 * Return Type  : void
 */
void r2br_r2dit_trig_impl(const emxArray_creal_T *x, int unsigned_nRows, const
  emxArray_real_T *costab, const emxArray_real_T *sintab, emxArray_creal_T *y)
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
  if (x->size[0] <= unsigned_nRows) {
    j = x->size[0];
  } else {
    j = unsigned_nRows;
  }

  nRowsD2 = unsigned_nRows / 2;
  nRowsD4 = nRowsD2 / 2;
  iy = y->size[0];
  y->size[0] = unsigned_nRows;
  emxEnsureCapacity((emxArray__common *)y, iy, (int)sizeof(creal_T));
  iy = x->size[0];
  if (unsigned_nRows > iy) {
    iDelta = y->size[0];
    iy = y->size[0];
    y->size[0] = iDelta;
    emxEnsureCapacity((emxArray__common *)y, iy, (int)sizeof(creal_T));
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
    iy <<= 1;
    ix -= iDelta;
  }
}

/*
 * File trailer for fft.c
 *
 * [EOF]
 */
