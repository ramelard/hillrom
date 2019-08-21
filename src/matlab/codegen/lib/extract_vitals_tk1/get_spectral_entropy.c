/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: get_spectral_entropy.c
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 08-Aug-2019 11:00:09
 */

/* Include Files */
#include <math.h>
#include "rt_nonfinite.h"
#include "extract_vitals_tk1.h"
#include "get_spectral_entropy.h"
#include "log21.h"
#include "extract_vitals_tk1_emxutil.h"
#include "combineVectorElements.h"
#include "extract_vitals_tk1_rtwutil.h"

/* Function Definitions */

/*
 * Arguments    : emxArray_creal_T *power_range
 *                emxArray_creal_T *entr
 * Return Type  : void
 */
void get_spectral_entropy(emxArray_creal_T *power_range, emxArray_creal_T *entr)
{
  int sck;
  int ib;
  emxArray_creal_T *y;
  emxArray_creal_T *a;
  unsigned int sz[2];
  int csz_idx_0;
  int acoef;
  emxArray_boolean_T *r1;
  int k;
  int ia;
  int szc;
  int b_acoef;
  int b_k;
  emxArray_boolean_T *r2;
  double a_re;
  double a_im;
  double y_re;
  double y_im;
  double brm;
  double bim;
  emxArray_int32_T *r3;

  /*  Get normalized entropy from spectral power range. */
  if (power_range->size[0] == 1) {
    sck = power_range->size[1];
    ib = power_range->size[0] * power_range->size[1];
    power_range->size[0] = sck;
    power_range->size[1] = 1;
    emxEnsureCapacity_creal_T(power_range, ib);
  }

  /*    A = sqrt(power_range); */
  /*  Make spectrum a PDF */
  emxInit_creal_T(&y, 2);
  if ((power_range->size[0] == 0) || (power_range->size[1] == 0)) {
    for (ib = 0; ib < 2; ib++) {
      sz[ib] = (unsigned int)power_range->size[ib];
    }

    ib = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = (int)sz[1];
    emxEnsureCapacity_creal_T(y, ib);
    sck = (int)sz[1];
    for (ib = 0; ib < sck; ib++) {
      y->data[ib].re = 0.0;
      y->data[ib].im = 0.0;
    }
  } else {
    colMajorFlatIter(power_range, power_range->size[0], y);
  }

  emxInit_creal_T(&a, 2);
  ib = a->size[0] * a->size[1];
  a->size[0] = power_range->size[0];
  a->size[1] = power_range->size[1];
  emxEnsureCapacity_creal_T(a, ib);
  sck = power_range->size[0] * power_range->size[1];
  for (ib = 0; ib < sck; ib++) {
    a->data[ib] = power_range->data[ib];
  }

  csz_idx_0 = power_range->size[0];
  sck = y->size[1];
  acoef = power_range->size[1];
  if (sck < acoef) {
    acoef = sck;
  }

  if (y->size[1] == 1) {
    sck = power_range->size[1];
  } else if (power_range->size[1] == 1) {
    sck = y->size[1];
  } else if (power_range->size[1] == y->size[1]) {
    sck = power_range->size[1];
  } else {
    sck = acoef;
  }

  ib = power_range->size[0] * power_range->size[1];
  power_range->size[0] = csz_idx_0;
  power_range->size[1] = sck;
  emxEnsureCapacity_creal_T(power_range, ib);
  if (!((power_range->size[0] == 0) || (power_range->size[1] == 0))) {
    sck = power_range->size[1];
    acoef = (a->size[1] != 1);
    csz_idx_0 = (y->size[1] != 1);
    for (k = 0; k < sck; k++) {
      ia = acoef * k;
      ib = csz_idx_0 * k;
      szc = power_range->size[0];
      b_acoef = (a->size[0] != 1);
      for (b_k = 0; b_k < szc; b_k++) {
        a_re = a->data[b_acoef * b_k + a->size[0] * ia].re;
        a_im = a->data[b_acoef * b_k + a->size[0] * ia].im;
        y_re = y->data[y->size[0] * ib].re;
        y_im = y->data[y->size[0] * ib].im;
        if (y_im == 0.0) {
          if (a_im == 0.0) {
            power_range->data[b_k + power_range->size[0] * k].re = a_re / y_re;
            power_range->data[b_k + power_range->size[0] * k].im = 0.0;
          } else if (a_re == 0.0) {
            power_range->data[b_k + power_range->size[0] * k].re = 0.0;
            power_range->data[b_k + power_range->size[0] * k].im = a_im / y_re;
          } else {
            power_range->data[b_k + power_range->size[0] * k].re = a_re / y_re;
            power_range->data[b_k + power_range->size[0] * k].im = a_im / y_re;
          }
        } else if (y_re == 0.0) {
          if (a_re == 0.0) {
            power_range->data[b_k + power_range->size[0] * k].re = a_im / y_im;
            power_range->data[b_k + power_range->size[0] * k].im = 0.0;
          } else if (a_im == 0.0) {
            power_range->data[b_k + power_range->size[0] * k].re = 0.0;
            power_range->data[b_k + power_range->size[0] * k].im = -(a_re / y_im);
          } else {
            power_range->data[b_k + power_range->size[0] * k].re = a_im / y_im;
            power_range->data[b_k + power_range->size[0] * k].im = -(a_re / y_im);
          }
        } else {
          brm = fabs(y_re);
          bim = fabs(y_im);
          if (brm > bim) {
            brm = y_im / y_re;
            bim = y_re + brm * y_im;
            power_range->data[b_k + power_range->size[0] * k].re = (a_re + brm *
              a_im) / bim;
            power_range->data[b_k + power_range->size[0] * k].im = (a_im - brm *
              a_re) / bim;
          } else if (bim == brm) {
            if (y_re > 0.0) {
              y_re = 0.5;
            } else {
              y_re = -0.5;
            }

            if (y_im > 0.0) {
              bim = 0.5;
            } else {
              bim = -0.5;
            }

            power_range->data[b_k + power_range->size[0] * k].re = (a_re * y_re
              + a_im * bim) / brm;
            power_range->data[b_k + power_range->size[0] * k].im = (a_im * y_re
              - a_re * bim) / brm;
          } else {
            brm = y_re / y_im;
            bim = y_im + brm * y_re;
            power_range->data[b_k + power_range->size[0] * k].re = (brm * a_re +
              a_im) / bim;
            power_range->data[b_k + power_range->size[0] * k].im = (brm * a_im -
              a_re) / bim;
          }
        }
      }
    }
  }

  emxFree_creal_T(&y);
  emxInit_boolean_T(&r1, 2);

  /*  Happens when whole spectrum range has 0 power. */
  ib = r1->size[0] * r1->size[1];
  r1->size[0] = power_range->size[0];
  r1->size[1] = power_range->size[1];
  emxEnsureCapacity_boolean_T(r1, ib);
  sck = power_range->size[0] * power_range->size[1];
  for (ib = 0; ib < sck; ib++) {
    r1->data[ib] = rtIsNaN(power_range->data[ib].re);
  }

  emxInit_boolean_T(&r2, 2);
  ib = r2->size[0] * r2->size[1];
  r2->size[0] = power_range->size[0];
  r2->size[1] = power_range->size[1];
  emxEnsureCapacity_boolean_T(r2, ib);
  sck = power_range->size[0] * power_range->size[1];
  for (ib = 0; ib < sck; ib++) {
    r2->data[ib] = rtIsNaN(power_range->data[ib].im);
  }

  sck = r1->size[0] * r1->size[1] - 1;
  ib = r1->size[0] * r1->size[1];
  emxEnsureCapacity_boolean_T(r1, ib);
  for (ib = 0; ib <= sck; ib++) {
    r1->data[ib] = (r1->data[ib] || r2->data[ib]);
  }

  ib = r2->size[0] * r2->size[1];
  r2->size[0] = power_range->size[0];
  r2->size[1] = power_range->size[1];
  emxEnsureCapacity_boolean_T(r2, ib);
  sck = power_range->size[0] * power_range->size[1];
  for (ib = 0; ib < sck; ib++) {
    r2->data[ib] = ((power_range->data[ib].re == 0.0) && (power_range->data[ib].
      im == 0.0));
  }

  ia = r1->size[0] * r1->size[1] - 1;
  sck = 0;
  for (csz_idx_0 = 0; csz_idx_0 <= ia; csz_idx_0++) {
    if (r1->data[csz_idx_0] || r2->data[csz_idx_0]) {
      sck++;
    }
  }

  emxInit_int32_T(&r3, 1);
  ib = r3->size[0];
  r3->size[0] = sck;
  emxEnsureCapacity_int32_T(r3, ib);
  acoef = 0;
  for (csz_idx_0 = 0; csz_idx_0 <= ia; csz_idx_0++) {
    if (r1->data[csz_idx_0] || r2->data[csz_idx_0]) {
      r3->data[acoef] = csz_idx_0 + 1;
      acoef++;
    }
  }

  emxFree_boolean_T(&r2);
  emxFree_boolean_T(&r1);
  sck = r3->size[0] - 1;
  for (ib = 0; ib <= sck; ib++) {
    power_range->data[r3->data[ib] - 1].re = 1.0E-100;
    power_range->data[r3->data[ib] - 1].im = 0.0;
  }

  emxFree_int32_T(&r3);

  /*  Use normalized entropy. */
  acoef = power_range->size[0] * power_range->size[1];
  for (ib = 0; ib < 2; ib++) {
    sz[ib] = (unsigned int)power_range->size[ib];
  }

  ib = a->size[0] * a->size[1];
  a->size[0] = (int)sz[0];
  a->size[1] = (int)sz[1];
  emxEnsureCapacity_creal_T(a, ib);
  for (k = 0; k < acoef; k++) {
    if (power_range->data[k].im == 0.0) {
      if (power_range->data[k].re < 0.0) {
        brm = scalar_real_log2(fabs(power_range->data[k].re));
        bim = 4.5323601418271942;
      } else {
        brm = scalar_real_log2(fabs(power_range->data[k].re));
        bim = 0.0;
      }
    } else if ((fabs(power_range->data[k].re) > 8.9884656743115785E+307) ||
               (fabs(power_range->data[k].im) > 8.9884656743115785E+307)) {
      bim = scalar_real_log2(rt_hypotd_snf(power_range->data[k].re / 2.0,
        power_range->data[k].im / 2.0));
      brm = bim + 1.0;
      bim = rt_atan2d_snf(power_range->data[k].im, power_range->data[k].re) /
        0.69314718055994529;
    } else {
      brm = scalar_real_log2(rt_hypotd_snf(power_range->data[k].re,
        power_range->data[k].im));
      bim = rt_atan2d_snf(power_range->data[k].im, power_range->data[k].re) /
        0.69314718055994529;
    }

    a->data[k].re = brm;
    a->data[k].im = bim;
  }

  sck = power_range->size[0] * power_range->size[1] - 1;
  ib = a->size[0] * a->size[1];
  a->size[0] = power_range->size[0];
  a->size[1] = power_range->size[1];
  emxEnsureCapacity_creal_T(a, ib);
  for (ib = 0; ib <= sck; ib++) {
    brm = power_range->data[ib].re;
    y_re = power_range->data[ib].im;
    a_re = a->data[ib].re;
    a_im = a->data[ib].im;
    a->data[ib].re = brm * a_re - y_re * a_im;
    a->data[ib].im = brm * a_im + y_re * a_re;
  }

  if ((a->size[0] == 0) || (a->size[1] == 0)) {
    for (ib = 0; ib < 2; ib++) {
      sz[ib] = (unsigned int)a->size[ib];
    }

    ib = entr->size[0] * entr->size[1];
    entr->size[0] = 1;
    entr->size[1] = (int)sz[1];
    emxEnsureCapacity_creal_T(entr, ib);
    sck = (int)sz[1];
    for (ib = 0; ib < sck; ib++) {
      entr->data[ib].re = 0.0;
      entr->data[ib].im = 0.0;
    }
  } else {
    colMajorFlatIter(a, a->size[0], entr);
  }

  emxFree_creal_T(&a);
  bim = scalar_real_log2(power_range->size[0]);
  sck = entr->size[0] * entr->size[1] - 1;
  ib = entr->size[0] * entr->size[1];
  entr->size[0] = 1;
  emxEnsureCapacity_creal_T(entr, ib);
  for (ib = 0; ib <= sck; ib++) {
    brm = -entr->data[ib].re;
    y_re = -entr->data[ib].im;
    if (y_re == 0.0) {
      entr->data[ib].re = brm / bim;
      entr->data[ib].im = 0.0;
    } else if (brm == 0.0) {
      entr->data[ib].re = 0.0;
      entr->data[ib].im = y_re / bim;
    } else {
      entr->data[ib].re = brm / bim;
      entr->data[ib].im = y_re / bim;
    }
  }

  /*        f30bpm = 5; */
  /*        f200bpm = 21; */
  /*         */
  /*        Acam_pow_norm = bsxfun(@rdivide, power_range, sum(power_range,1)); */
  /*        pct_in_hr = sum(Acam_pow_norm(f30bpm:f200bpm,:),1); */
  /*        max_over_mean = max(Acam_pow_norm(f30bpm:f200bpm,:),[],1)./mean(Acam_pow_norm(f30bpm:f200bpm,:),1); */
  /*        entr = entr./max(Acam_pow_norm(f30bpm:f200bpm,:),[],1); */
  /*        entr = 1./entr; */
  /*        entr = max(Acam_pow_norm(f30bpm:f200bpm,:),[],1)./entr; */
  /*        entr = entr.*(1-pct_in_hr); */
  /*        entr = entr.*(pct_in_hr<.2); */
  /*        entr = pct_in_hr>.5; */
  /*        entr = pct_in_hr; */
  /*        a = .7165; b = 14.9; */
  /*        x = 1-pct_in_hr; */
  /*        entr = entr.*(a*log(b*x./(1-x))); */
  /*        entr = pct_in_hr; */
}

/*
 * File trailer for get_spectral_entropy.c
 *
 * [EOF]
 */
