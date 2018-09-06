/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: get_spectral_entropy.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 06-Sep-2018 16:44:03
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "extract_vitals_tk1.h"
#include "get_spectral_entropy.h"
#include "extract_vitals_tk1_emxutil.h"
#include "relop.h"
#include "log21.h"
#include "extract_vitals_tk1_rtwutil.h"

/* Function Definitions */

/*
 * Arguments    : emxArray_creal_T *power_range
 *                emxArray_creal_T *entr
 * Return Type  : void
 */
void get_spectral_entropy(emxArray_creal_T *power_range, emxArray_creal_T *entr)
{
  int vlen;
  emxArray_creal_T *y;
  int i7;
  int asub;
  int xoffset;
  emxArray_creal_T *a;
  double s_re;
  double s_im;
  int k;
  int na1;
  emxArray_creal_T *av;
  emxArray_boolean_T *r2;
  int bsub;
  int ak;
  int bk;
  int nc1;
  int ck;
  emxArray_boolean_T *r3;
  emxArray_creal_T *cv;
  double y_re;
  double y_im;
  double av_re;
  double av_im;
  double brm;
  emxArray_int32_T *r4;
  unsigned int uv0[2];

  /*  Get normalized entropy from spectral power range. */
  if (power_range->size[0] == 1) {
    vlen = power_range->size[1];
    i7 = power_range->size[0] * power_range->size[1];
    power_range->size[0] = vlen;
    power_range->size[1] = 1;
    emxEnsureCapacity((emxArray__common *)power_range, i7, (int)sizeof(creal_T));
  }

  emxInit_creal_T(&y, 2);

  /*    A = sqrt(power_range); */
  /*  Make spectrum a PDF */
  i7 = y->size[0] * y->size[1];
  y->size[0] = 1;
  y->size[1] = power_range->size[1];
  emxEnsureCapacity((emxArray__common *)y, i7, (int)sizeof(creal_T));
  if ((power_range->size[0] == 0) || (power_range->size[1] == 0)) {
    i7 = y->size[0] * y->size[1];
    y->size[0] = 1;
    emxEnsureCapacity((emxArray__common *)y, i7, (int)sizeof(creal_T));
    vlen = y->size[1];
    for (i7 = 0; i7 < vlen; i7++) {
      y->data[y->size[0] * i7].re = 0.0;
      y->data[y->size[0] * i7].im = 0.0;
    }
  } else {
    vlen = power_range->size[0];
    for (asub = 0; asub + 1 <= power_range->size[1]; asub++) {
      xoffset = asub * vlen;
      s_re = power_range->data[xoffset].re;
      s_im = power_range->data[xoffset].im;
      for (k = 2; k <= vlen; k++) {
        s_re += power_range->data[(xoffset + k) - 1].re;
        s_im += power_range->data[(xoffset + k) - 1].im;
      }

      y->data[asub].re = s_re;
      y->data[asub].im = s_im;
    }
  }

  emxInit_creal_T(&a, 2);
  i7 = a->size[0] * a->size[1];
  a->size[0] = power_range->size[0];
  a->size[1] = power_range->size[1];
  emxEnsureCapacity((emxArray__common *)a, i7, (int)sizeof(creal_T));
  vlen = power_range->size[0] * power_range->size[1];
  for (i7 = 0; i7 < vlen; i7++) {
    a->data[i7] = power_range->data[i7];
  }

  na1 = power_range->size[0];
  vlen = power_range->size[0];
  if (power_range->size[1] <= y->size[1]) {
    xoffset = power_range->size[1];
  } else {
    xoffset = y->size[1];
  }

  i7 = power_range->size[0] * power_range->size[1];
  power_range->size[0] = vlen;
  power_range->size[1] = xoffset;
  emxEnsureCapacity((emxArray__common *)power_range, i7, (int)sizeof(creal_T));
  if (!((power_range->size[0] == 0) || (power_range->size[1] == 0))) {
    emxInit_creal_T1(&av, 1);
    i7 = av->size[0];
    av->size[0] = na1;
    emxEnsureCapacity((emxArray__common *)av, i7, (int)sizeof(creal_T));
    asub = 1;
    bsub = 1;
    ak = -1;
    bk = 0;
    nc1 = power_range->size[0];
    i7 = power_range->size[0] * power_range->size[1] - power_range->size[0];
    ck = 0;
    emxInit_creal_T1(&cv, 1);
    while (ck <= i7) {
      for (k = 1; k <= na1; k++) {
        av->data[k - 1] = a->data[ak + k];
      }

      y_re = y->data[bk].re;
      y_im = y->data[bk].im;
      xoffset = cv->size[0];
      cv->size[0] = av->size[0];
      emxEnsureCapacity((emxArray__common *)cv, xoffset, (int)sizeof(creal_T));
      vlen = av->size[0];
      for (xoffset = 0; xoffset < vlen; xoffset++) {
        av_re = av->data[xoffset].re;
        av_im = av->data[xoffset].im;
        if (y_im == 0.0) {
          if (av_im == 0.0) {
            cv->data[xoffset].re = av_re / y_re;
            cv->data[xoffset].im = 0.0;
          } else if (av_re == 0.0) {
            cv->data[xoffset].re = 0.0;
            cv->data[xoffset].im = av_im / y_re;
          } else {
            cv->data[xoffset].re = av_re / y_re;
            cv->data[xoffset].im = av_im / y_re;
          }
        } else if (y_re == 0.0) {
          if (av_re == 0.0) {
            cv->data[xoffset].re = av_im / y_im;
            cv->data[xoffset].im = 0.0;
          } else if (av_im == 0.0) {
            cv->data[xoffset].re = 0.0;
            cv->data[xoffset].im = -(av_re / y_im);
          } else {
            cv->data[xoffset].re = av_im / y_im;
            cv->data[xoffset].im = -(av_re / y_im);
          }
        } else {
          brm = fabs(y_re);
          s_re = fabs(y_im);
          if (brm > s_re) {
            s_re = y_im / y_re;
            s_im = y_re + s_re * y_im;
            cv->data[xoffset].re = (av_re + s_re * av_im) / s_im;
            cv->data[xoffset].im = (av_im - s_re * av_re) / s_im;
          } else if (s_re == brm) {
            if (y_re > 0.0) {
              s_re = 0.5;
            } else {
              s_re = -0.5;
            }

            if (y_im > 0.0) {
              s_im = 0.5;
            } else {
              s_im = -0.5;
            }

            cv->data[xoffset].re = (av_re * s_re + av_im * s_im) / brm;
            cv->data[xoffset].im = (av_im * s_re - av_re * s_im) / brm;
          } else {
            s_re = y_re / y_im;
            s_im = y_im + s_re * y_re;
            cv->data[xoffset].re = (s_re * av_re + av_im) / s_im;
            cv->data[xoffset].im = (s_re * av_im - av_re) / s_im;
          }
        }
      }

      for (k = 1; k <= nc1; k++) {
        power_range->data[(ck + k) - 1] = cv->data[k - 1];
      }

      if (asub < a->size[1]) {
        ak += na1;
        bk++;
        bsub++;
        asub++;
      } else if (bsub < y->size[1]) {
        bk++;
        bsub++;
      } else {
        asub = 1;
        bsub = 1;
      }

      ck += nc1;
    }

    emxFree_creal_T(&cv);
    emxFree_creal_T(&av);
  }

  emxFree_creal_T(&y);
  emxInit_boolean_T(&r2, 2);

  /*  Happens when whole spectrum range has 0 power. */
  i7 = r2->size[0] * r2->size[1];
  r2->size[0] = power_range->size[0];
  r2->size[1] = power_range->size[1];
  emxEnsureCapacity((emxArray__common *)r2, i7, (int)sizeof(boolean_T));
  vlen = power_range->size[0] * power_range->size[1];
  for (i7 = 0; i7 < vlen; i7++) {
    r2->data[i7] = rtIsNaN(power_range->data[i7].re);
  }

  emxInit_boolean_T(&r3, 2);
  i7 = r3->size[0] * r3->size[1];
  r3->size[0] = power_range->size[0];
  r3->size[1] = power_range->size[1];
  emxEnsureCapacity((emxArray__common *)r3, i7, (int)sizeof(boolean_T));
  vlen = power_range->size[0] * power_range->size[1];
  for (i7 = 0; i7 < vlen; i7++) {
    r3->data[i7] = rtIsNaN(power_range->data[i7].im);
  }

  i7 = r2->size[0] * r2->size[1];
  emxEnsureCapacity((emxArray__common *)r2, i7, (int)sizeof(boolean_T));
  i7 = r2->size[0];
  xoffset = r2->size[1];
  vlen = i7 * xoffset;
  for (i7 = 0; i7 < vlen; i7++) {
    r2->data[i7] = (r2->data[i7] || r3->data[i7]);
  }

  i7 = r3->size[0] * r3->size[1];
  r3->size[0] = power_range->size[0];
  r3->size[1] = power_range->size[1];
  emxEnsureCapacity((emxArray__common *)r3, i7, (int)sizeof(boolean_T));
  vlen = power_range->size[0] * power_range->size[1];
  for (i7 = 0; i7 < vlen; i7++) {
    r3->data[i7] = ((power_range->data[i7].re == 0.0) && (power_range->data[i7].
      im == 0.0));
  }

  emxInit_int32_T(&r4, 1);
  xoffset = r2->size[0] * r2->size[1] - 1;
  vlen = 0;
  for (asub = 0; asub <= xoffset; asub++) {
    if (r2->data[asub] || r3->data[asub]) {
      vlen++;
    }
  }

  i7 = r4->size[0];
  r4->size[0] = vlen;
  emxEnsureCapacity((emxArray__common *)r4, i7, (int)sizeof(int));
  vlen = 0;
  for (asub = 0; asub <= xoffset; asub++) {
    if (r2->data[asub] || r3->data[asub]) {
      r4->data[vlen] = asub + 1;
      vlen++;
    }
  }

  emxFree_boolean_T(&r3);
  emxFree_boolean_T(&r2);
  vlen = r4->size[0];
  for (i7 = 0; i7 < vlen; i7++) {
    power_range->data[r4->data[i7] - 1].re = 1.0E-100;
    power_range->data[r4->data[i7] - 1].im = 0.0;
  }

  emxFree_int32_T(&r4);

  /*  Use normalized entropy. */
  for (i7 = 0; i7 < 2; i7++) {
    uv0[i7] = (unsigned int)power_range->size[i7];
  }

  i7 = a->size[0] * a->size[1];
  a->size[0] = (int)uv0[0];
  a->size[1] = (int)uv0[1];
  emxEnsureCapacity((emxArray__common *)a, i7, (int)sizeof(creal_T));
  vlen = power_range->size[0] * power_range->size[1];
  for (k = 0; k + 1 <= vlen; k++) {
    if ((power_range->data[k].im == 0.0) && rtIsNaN(power_range->data[k].re)) {
      s_re = power_range->data[k].re;
      s_im = power_range->data[k].im;
    } else if ((fabs(power_range->data[k].re) > 8.9884656743115785E+307) ||
               (fabs(power_range->data[k].im) > 8.9884656743115785E+307)) {
      s_re = scalar_real_log2(rt_hypotd_snf(power_range->data[k].re / 2.0,
        power_range->data[k].im / 2.0)) + 1.0;
      s_im = rt_atan2d_snf(power_range->data[k].im, power_range->data[k].re) /
        0.69314718055994529;
    } else {
      s_re = scalar_real_log2(rt_hypotd_snf(power_range->data[k].re,
        power_range->data[k].im));
      s_im = rt_atan2d_snf(power_range->data[k].im, power_range->data[k].re) /
        0.69314718055994529;
    }

    a->data[k].re = s_re;
    a->data[k].im = s_im;
  }

  i7 = a->size[0] * a->size[1];
  a->size[0] = power_range->size[0];
  a->size[1] = power_range->size[1];
  emxEnsureCapacity((emxArray__common *)a, i7, (int)sizeof(creal_T));
  vlen = power_range->size[0] * power_range->size[1];
  for (i7 = 0; i7 < vlen; i7++) {
    s_re = power_range->data[i7].re;
    s_im = power_range->data[i7].im;
    brm = a->data[i7].re;
    y_re = a->data[i7].im;
    a->data[i7].re = s_re * brm - s_im * y_re;
    a->data[i7].im = s_re * y_re + s_im * brm;
  }

  i7 = entr->size[0] * entr->size[1];
  entr->size[0] = 1;
  entr->size[1] = a->size[1];
  emxEnsureCapacity((emxArray__common *)entr, i7, (int)sizeof(creal_T));
  if ((a->size[0] == 0) || (a->size[1] == 0)) {
    i7 = entr->size[0] * entr->size[1];
    entr->size[0] = 1;
    emxEnsureCapacity((emxArray__common *)entr, i7, (int)sizeof(creal_T));
    vlen = entr->size[1];
    for (i7 = 0; i7 < vlen; i7++) {
      entr->data[entr->size[0] * i7].re = 0.0;
      entr->data[entr->size[0] * i7].im = 0.0;
    }
  } else {
    vlen = a->size[0];
    for (asub = 0; asub + 1 <= a->size[1]; asub++) {
      xoffset = asub * vlen;
      s_re = a->data[xoffset].re;
      s_im = a->data[xoffset].im;
      for (k = 2; k <= vlen; k++) {
        s_re += a->data[(xoffset + k) - 1].re;
        s_im += a->data[(xoffset + k) - 1].im;
      }

      entr->data[asub].re = s_re;
      entr->data[asub].im = s_im;
    }
  }

  emxFree_creal_T(&a);
  s_re = scalar_real_log2(power_range->size[0]);
  i7 = entr->size[0] * entr->size[1];
  entr->size[0] = 1;
  emxEnsureCapacity((emxArray__common *)entr, i7, (int)sizeof(creal_T));
  vlen = entr->size[0];
  xoffset = entr->size[1];
  vlen *= xoffset;
  for (i7 = 0; i7 < vlen; i7++) {
    s_im = -entr->data[i7].re;
    brm = -entr->data[i7].im;
    if (brm == 0.0) {
      entr->data[i7].re = s_im / s_re;
      entr->data[i7].im = 0.0;
    } else if (s_im == 0.0) {
      entr->data[i7].re = 0.0;
      entr->data[i7].im = brm / s_re;
    } else {
      entr->data[i7].re = s_im / s_re;
      entr->data[i7].im = brm / s_re;
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
