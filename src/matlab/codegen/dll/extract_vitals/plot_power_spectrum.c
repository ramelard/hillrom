/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: plot_power_spectrum.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 01-Mar-2018 16:31:19
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "extract_vitals.h"
#include "plot_power_spectrum.h"
#include "extract_vitals_emxutil.h"
#include "bluestein_setup.h"

/* Function Definitions */

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
 * Arguments    : const double y_data[]
 *                double freq_data[]
 *                int freq_size[2]
 *                creal_T power_data[]
 *                int power_size[1]
 * Return Type  : void
 */
void plot_power_spectrum(const double y_data[], double freq_data[], int
  freq_size[2], creal_T power_data[], int power_size[1])
{
  emxArray_real_T *costab1q;
  int N2blue;
  int nd2;
  double e;
  int nRowsD4;
  int k;
  emxArray_real_T *costab;
  emxArray_real_T *sintab;
  emxArray_real_T *sintabinv;
  emxArray_creal_T *y;
  int wwc_size[1];
  creal_T wwc_data[5];
  int Y_size[1];
  creal_T Y_data[3];
  emxArray_creal_T b_Y_data;
  int fy_size[1];
  emxArray_creal_T b_wwc_data;
  creal_T fy_data[12];
  creal_T fv_data[12];
  emxArray_creal_T b_fy_data;
  double Y_data_im;
  creal_T c_Y_data[3];
  double Y_data_re;
  emxInit_real_T(&costab1q, 2);

  /* PLOT_POWER_SPECTRUM Plot frequency power spectrum */
  /*  */
  /*  [freq,fpower] = plot_power_spectrum(y, sampling_rate) */
  /*  */
  /*  From http://www.mathworks.com/products/matlab/examples.html?file=/products/demos/shipping/matlab/fftdemo.html */
  /*  y - signal (columns are observations) */
  /*  if size(y,2) > size(y,1) */
  /*    warning(sprintf('Are you sure columns are observations? (r%u,c%u)', size(y,1), size(y,2))) */
  /*  end */
  /*  npts = numel(y); */
  get_algo_sizes(3, false, &N2blue, &nd2);
  e = 6.2831853071795862 / (double)nd2;
  nRowsD4 = nd2 / 2 / 2;
  k = costab1q->size[0] * costab1q->size[1];
  costab1q->size[0] = 1;
  costab1q->size[1] = nRowsD4 + 1;
  emxEnsureCapacity((emxArray__common *)costab1q, k, (int)sizeof(double));
  costab1q->data[0] = 1.0;
  nd2 = nRowsD4 / 2;
  for (k = 1; k <= nd2; k++) {
    costab1q->data[k] = cos(e * (double)k);
  }

  for (k = nd2 + 1; k < nRowsD4; k++) {
    costab1q->data[k] = sin(e * (double)(nRowsD4 - k));
  }

  emxInit_real_T(&costab, 2);
  emxInit_real_T(&sintab, 2);
  emxInit_real_T(&sintabinv, 2);
  costab1q->data[nRowsD4] = 0.0;
  nd2 = costab1q->size[1] - 1;
  nRowsD4 = (costab1q->size[1] - 1) << 1;
  k = costab->size[0] * costab->size[1];
  costab->size[0] = 1;
  costab->size[1] = nRowsD4 + 1;
  emxEnsureCapacity((emxArray__common *)costab, k, (int)sizeof(double));
  k = sintab->size[0] * sintab->size[1];
  sintab->size[0] = 1;
  sintab->size[1] = nRowsD4 + 1;
  emxEnsureCapacity((emxArray__common *)sintab, k, (int)sizeof(double));
  costab->data[0] = 1.0;
  sintab->data[0] = 0.0;
  k = sintabinv->size[0] * sintabinv->size[1];
  sintabinv->size[0] = 1;
  sintabinv->size[1] = nRowsD4 + 1;
  emxEnsureCapacity((emxArray__common *)sintabinv, k, (int)sizeof(double));
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

  emxFree_real_T(&costab1q);
  emxInit_creal_T1(&y, 1);
  bluestein_setup(3, y);
  wwc_size[0] = y->size[0];
  nd2 = y->size[0];
  for (k = 0; k < nd2; k++) {
    wwc_data[k] = y->data[k];
  }

  k = y->size[0];
  y->size[0] = 3;
  emxEnsureCapacity((emxArray__common *)y, k, (int)sizeof(creal_T));
  Y_size[0] = y->size[0];
  nd2 = y->size[0];
  for (k = 0; k < nd2; k++) {
    Y_data[k] = y->data[k];
  }

  nd2 = 0;
  Y_size[0] = 3;
  for (k = 0; k < 3; k++) {
    Y_data[k].re = wwc_data[k + 2].re * y_data[nd2];
    Y_data[k].im = wwc_data[k + 2].im * -y_data[nd2];
    nd2++;
  }

  b_Y_data.data = (creal_T *)&Y_data;
  b_Y_data.size = (int *)&Y_size;
  b_Y_data.allocatedSize = 3;
  b_Y_data.numDimensions = 1;
  b_Y_data.canFreeData = false;
  r2br_r2dit_trig_impl(&b_Y_data, N2blue, costab, sintab, y);
  fy_size[0] = y->size[0];
  nd2 = y->size[0];
  for (k = 0; k < nd2; k++) {
    fy_data[k] = y->data[k];
  }

  b_wwc_data.data = (creal_T *)&wwc_data;
  b_wwc_data.size = (int *)&wwc_size;
  b_wwc_data.allocatedSize = 5;
  b_wwc_data.numDimensions = 1;
  b_wwc_data.canFreeData = false;
  r2br_r2dit_trig(&b_wwc_data, N2blue, costab, sintab, y);
  nd2 = y->size[0];
  emxFree_real_T(&sintab);
  for (k = 0; k < nd2; k++) {
    fv_data[k] = y->data[k];
  }

  nd2 = fy_size[0];
  for (k = 0; k < nd2; k++) {
    e = fy_data[k].re;
    Y_data_im = fy_data[k].im;
    fy_data[k].re = e * fv_data[k].re - Y_data_im * fv_data[k].im;
    fy_data[k].im = e * fv_data[k].im + Y_data_im * fv_data[k].re;
  }

  b_fy_data.data = (creal_T *)&fy_data;
  b_fy_data.size = (int *)&fy_size;
  b_fy_data.allocatedSize = 12;
  b_fy_data.numDimensions = 1;
  b_fy_data.canFreeData = false;
  b_r2br_r2dit_trig(&b_fy_data, N2blue, costab, sintabinv, y);
  nd2 = y->size[0];
  emxFree_real_T(&sintabinv);
  emxFree_real_T(&costab);
  for (k = 0; k < nd2; k++) {
    fv_data[k] = y->data[k];
  }

  emxFree_creal_T(&y);
  nd2 = 0;
  for (k = 0; k < 3; k++) {
    Y_data[nd2].re = wwc_data[k + 2].re * fv_data[k + 2].re + wwc_data[k + 2].im
      * fv_data[k + 2].im;
    Y_data[nd2].im = wwc_data[k + 2].re * fv_data[k + 2].im - wwc_data[k + 2].im
      * fv_data[k + 2].re;
    nd2++;
  }

  /*  power = (a^2+b^2)/N */
  /*  Should be same as 1/N*abs(Y)^2 */
  /*  nth bin is n*Fs/N, +1 for index offset */
  freq_size[0] = 1;
  freq_size[1] = 1;
  freq_data[0] = 0.0;
  for (k = 0; k < 3; k++) {
    e = Y_data[k].re;
    Y_data_im = -Y_data[k].im;
    Y_data_re = Y_data[k].re * e - Y_data[k].im * Y_data_im;
    Y_data_im = Y_data[k].re * Y_data_im + Y_data[k].im * e;
    if (Y_data_im == 0.0) {
      c_Y_data[k].re = Y_data_re / 3.0;
      c_Y_data[k].im = 0.0;
    } else if (Y_data_re == 0.0) {
      c_Y_data[k].re = 0.0;
      c_Y_data[k].im = Y_data_im / 3.0;
    } else {
      c_Y_data[k].re = Y_data_re / 3.0;
      c_Y_data[k].im = Y_data_im / 3.0;
    }
  }

  power_size[0] = 1;
  power_data[0] = c_Y_data[0];
}

/*
 * Arguments    : const emxArray_creal_T *x
 *                int n1_unsigned
 *                const emxArray_real_T *costab
 *                const emxArray_real_T *sintab
 *                emxArray_creal_T *y
 * Return Type  : void
 */
void r2br_r2dit_trig(const emxArray_creal_T *x, int n1_unsigned, const
                     emxArray_real_T *costab, const emxArray_real_T *sintab,
                     emxArray_creal_T *y)
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
 * File trailer for plot_power_spectrum.c
 *
 * [EOF]
 */
