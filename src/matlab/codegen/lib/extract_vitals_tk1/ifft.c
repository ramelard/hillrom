/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: ifft.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 18-Apr-2018 17:29:29
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "extract_vitals_tk1.h"
#include "ifft.h"
#include "extract_vitals_tk1_emxutil.h"
#include "fft.h"

/* Function Definitions */

/*
 * Arguments    : const emxArray_creal_T *x
 *                emxArray_creal_T *y
 * Return Type  : void
 */
void ifft(const emxArray_creal_T *x, emxArray_creal_T *y)
{
  int n1;
  emxArray_real_T *costab1q;
  int nInt2;
  boolean_T useRadix2;
  int N2blue;
  int nd2;
  double e;
  int nRowsD4;
  int k;
  emxArray_real_T *costab;
  emxArray_real_T *sintab;
  emxArray_real_T *sintabinv;
  emxArray_creal_T *wwc;
  int nInt2m1;
  int b_y;
  double denom_re;
  emxArray_creal_T *fy;
  double wwc_im;
  emxArray_creal_T *fv;
  double wwc_re;
  double fv_im;
  double b_wwc_im;
  double fv_re;
  double b_fv_im;
  double b_fv_re;
  n1 = x->size[0];
  if (x->size[0] == 0) {
    nInt2 = y->size[0];
    y->size[0] = 0;
    emxEnsureCapacity((emxArray__common *)y, nInt2, (int)sizeof(creal_T));
  } else {
    emxInit_real_T1(&costab1q, 2);
    useRadix2 = ((x->size[0] & (x->size[0] - 1)) == 0);
    get_algo_sizes(x->size[0], useRadix2, &N2blue, &nd2);
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
    emxInit_real_T1(&costab, 2);
    emxInit_real_T1(&sintab, 2);
    emxInit_real_T1(&sintabinv, 2);
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
        sintab->data[k] = costab1q->data[nd2 - k];
      }

      for (k = costab1q->size[1]; k <= nRowsD4; k++) {
        costab->data[k] = -costab1q->data[nRowsD4 - k];
        sintab->data[k] = costab1q->data[k - nd2];
      }

      nInt2 = sintabinv->size[0] * sintabinv->size[1];
      sintabinv->size[0] = 1;
      sintabinv->size[1] = 0;
      emxEnsureCapacity((emxArray__common *)sintabinv, nInt2, (int)sizeof(double));
    }

    emxFree_real_T(&costab1q);
    if (useRadix2) {
      c_r2br_r2dit_trig(x, x->size[0], costab, sintab, y);
    } else {
      emxInit_creal_T1(&wwc, 1);
      nInt2m1 = (x->size[0] + x->size[0]) - 1;
      nInt2 = wwc->size[0];
      wwc->size[0] = nInt2m1;
      emxEnsureCapacity((emxArray__common *)wwc, nInt2, (int)sizeof(creal_T));
      nd2 = x->size[0];
      nRowsD4 = 0;
      wwc->data[x->size[0] - 1].re = 1.0;
      wwc->data[x->size[0] - 1].im = 0.0;
      nInt2 = x->size[0] << 1;
      for (k = 1; k < n1; k++) {
        b_y = (k << 1) - 1;
        if (nInt2 - nRowsD4 <= b_y) {
          nRowsD4 += b_y - nInt2;
        } else {
          nRowsD4 += b_y;
        }

        e = 3.1415926535897931 * (double)nRowsD4 / (double)x->size[0];
        if (e == 0.0) {
          denom_re = 1.0;
          e = 0.0;
        } else {
          denom_re = cos(e);
          e = sin(e);
        }

        wwc->data[nd2 - 2].re = denom_re;
        wwc->data[nd2 - 2].im = -e;
        nd2--;
      }

      nd2 = 0;
      for (k = nInt2m1 - 1; k >= n1; k--) {
        wwc->data[k] = wwc->data[nd2];
        nd2++;
      }

      nRowsD4 = x->size[0];
      nd2 = x->size[0];
      nInt2 = y->size[0];
      y->size[0] = nd2;
      emxEnsureCapacity((emxArray__common *)y, nInt2, (int)sizeof(creal_T));
      nd2 = 0;
      for (k = 0; k + 1 <= nRowsD4; k++) {
        denom_re = wwc->data[(n1 + k) - 1].re;
        e = wwc->data[(n1 + k) - 1].im;
        wwc_im = x->data[nd2].re;
        wwc_re = x->data[nd2].im;
        fv_im = x->data[nd2].im;
        b_wwc_im = x->data[nd2].re;
        y->data[k].re = denom_re * wwc_im + e * wwc_re;
        y->data[k].im = denom_re * fv_im - e * b_wwc_im;
        nd2++;
      }

      while (nRowsD4 + 1 <= n1) {
        y->data[nRowsD4].re = 0.0;
        y->data[nRowsD4].im = 0.0;
        nRowsD4++;
      }

      emxInit_creal_T1(&fy, 1);
      emxInit_creal_T1(&fv, 1);
      r2br_r2dit_trig_impl(y, N2blue, costab, sintab, fy);
      b_r2br_r2dit_trig(wwc, N2blue, costab, sintab, fv);
      nInt2 = fy->size[0];
      emxEnsureCapacity((emxArray__common *)fy, nInt2, (int)sizeof(creal_T));
      nd2 = fy->size[0];
      for (nInt2 = 0; nInt2 < nd2; nInt2++) {
        e = fy->data[nInt2].re;
        wwc_im = fy->data[nInt2].im;
        fv_re = fv->data[nInt2].re;
        b_fv_im = fv->data[nInt2].im;
        fy->data[nInt2].re = e * fv_re - wwc_im * b_fv_im;
        fy->data[nInt2].im = e * b_fv_im + wwc_im * fv_re;
      }

      c_r2br_r2dit_trig(fy, N2blue, costab, sintabinv, fv);
      nd2 = 0;
      denom_re = x->size[0];
      k = x->size[0] - 1;
      emxFree_creal_T(&fy);
      while (k + 1 <= wwc->size[0]) {
        e = wwc->data[k].re;
        fv_re = fv->data[k].re;
        wwc_im = wwc->data[k].im;
        b_fv_im = fv->data[k].im;
        wwc_re = wwc->data[k].re;
        fv_im = fv->data[k].im;
        b_wwc_im = wwc->data[k].im;
        b_fv_re = fv->data[k].re;
        y->data[nd2].re = e * fv_re + wwc_im * b_fv_im;
        y->data[nd2].im = wwc_re * fv_im - b_wwc_im * b_fv_re;
        e = wwc->data[k].re;
        fv_re = fv->data[k].re;
        wwc_im = wwc->data[k].im;
        b_fv_im = fv->data[k].im;
        wwc_re = wwc->data[k].re;
        fv_im = fv->data[k].im;
        b_wwc_im = wwc->data[k].im;
        b_fv_re = fv->data[k].re;
        y->data[nd2].re = e * fv_re + wwc_im * b_fv_im;
        y->data[nd2].im = wwc_re * fv_im - b_wwc_im * b_fv_re;
        e = y->data[nd2].re;
        wwc_im = y->data[nd2].im;
        if (wwc_im == 0.0) {
          y->data[nd2].re = e / denom_re;
          y->data[nd2].im = 0.0;
        } else if (e == 0.0) {
          y->data[nd2].re = 0.0;
          y->data[nd2].im = wwc_im / denom_re;
        } else {
          y->data[nd2].re = e / denom_re;
          y->data[nd2].im = wwc_im / denom_re;
        }

        nd2++;
        k++;
      }

      emxFree_creal_T(&fv);
      emxFree_creal_T(&wwc);
    }

    emxFree_real_T(&sintabinv);
    emxFree_real_T(&sintab);
    emxFree_real_T(&costab);
  }
}

/*
 * File trailer for ifft.c
 *
 * [EOF]
 */
