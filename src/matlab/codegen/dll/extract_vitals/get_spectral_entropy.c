/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: get_spectral_entropy.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 01-Mar-2018 16:31:19
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "extract_vitals.h"
#include "get_spectral_entropy.h"
#include "extract_vitals_emxutil.h"
#include "log21.h"

/* Function Declarations */
static double rt_atan2d_snf(double u0, double u1);
static double rt_hypotd_snf(double u0, double u1);

/* Function Definitions */

/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  int b_u0;
  int b_u1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }

    if (u1 > 0.0) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }

    y = atan2(b_u0, b_u1);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_hypotd_snf(double u0, double u1)
{
  double y;
  double a;
  double b;
  a = fabs(u0);
  b = fabs(u1);
  if (a < b) {
    a /= b;
    y = b * sqrt(a * a + 1.0);
  } else if (a > b) {
    b /= a;
    y = a * sqrt(b * b + 1.0);
  } else if (rtIsNaN(b)) {
    y = b;
  } else {
    y = a * 1.4142135623730951;
  }

  return y;
}

/*
 * Arguments    : emxArray_creal_T *power_range
 *                emxArray_creal_T *entr
 * Return Type  : void
 */
void get_spectral_entropy(emxArray_creal_T *power_range, emxArray_creal_T *entr)
{
  int vlen;
  emxArray_creal_T *y;
  int i10;
  int asub;
  int xoffset;
  emxArray_creal_T *a;
  double s_re;
  double s_im;
  int k;
  int na1;
  emxArray_creal_T *av;
  emxArray_boolean_T *r0;
  int bsub;
  int ak;
  int bk;
  int nc1;
  int ck;
  emxArray_boolean_T *r1;
  emxArray_creal_T *cv;
  double y_re;
  double y_im;
  double av_re;
  double av_im;
  double brm;
  emxArray_int32_T *r2;
  unsigned int uv0[2];

  /*  Get normalized entropy from spectral power range. */
  if (power_range->size[0] == 1) {
    vlen = power_range->size[1];
    i10 = power_range->size[0] * power_range->size[1];
    power_range->size[0] = vlen;
    power_range->size[1] = 1;
    emxEnsureCapacity((emxArray__common *)power_range, i10, (int)sizeof(creal_T));
  }

  emxInit_creal_T(&y, 2);

  /*    A = sqrt(power_range); */
  /*  Make spectrum a PDF */
  i10 = y->size[0] * y->size[1];
  y->size[0] = 1;
  y->size[1] = power_range->size[1];
  emxEnsureCapacity((emxArray__common *)y, i10, (int)sizeof(creal_T));
  if ((power_range->size[0] == 0) || (power_range->size[1] == 0)) {
    i10 = y->size[0] * y->size[1];
    y->size[0] = 1;
    emxEnsureCapacity((emxArray__common *)y, i10, (int)sizeof(creal_T));
    vlen = y->size[1];
    for (i10 = 0; i10 < vlen; i10++) {
      y->data[y->size[0] * i10].re = 0.0;
      y->data[y->size[0] * i10].im = 0.0;
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
  i10 = a->size[0] * a->size[1];
  a->size[0] = power_range->size[0];
  a->size[1] = power_range->size[1];
  emxEnsureCapacity((emxArray__common *)a, i10, (int)sizeof(creal_T));
  vlen = power_range->size[0] * power_range->size[1];
  for (i10 = 0; i10 < vlen; i10++) {
    a->data[i10] = power_range->data[i10];
  }

  na1 = power_range->size[0];
  vlen = power_range->size[0];
  if (power_range->size[1] <= y->size[1]) {
    xoffset = power_range->size[1];
  } else {
    xoffset = y->size[1];
  }

  i10 = power_range->size[0] * power_range->size[1];
  power_range->size[0] = vlen;
  power_range->size[1] = xoffset;
  emxEnsureCapacity((emxArray__common *)power_range, i10, (int)sizeof(creal_T));
  if (!((power_range->size[0] == 0) || (power_range->size[1] == 0))) {
    emxInit_creal_T1(&av, 1);
    i10 = av->size[0];
    av->size[0] = na1;
    emxEnsureCapacity((emxArray__common *)av, i10, (int)sizeof(creal_T));
    asub = 1;
    bsub = 1;
    ak = -1;
    bk = 0;
    nc1 = power_range->size[0];
    i10 = power_range->size[0] * power_range->size[1] - power_range->size[0];
    ck = 0;
    emxInit_creal_T1(&cv, 1);
    while (ck <= i10) {
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
  emxInit_boolean_T(&r0, 2);

  /*  Happens when whole spectrum range has 0 power. */
  i10 = r0->size[0] * r0->size[1];
  r0->size[0] = power_range->size[0];
  r0->size[1] = power_range->size[1];
  emxEnsureCapacity((emxArray__common *)r0, i10, (int)sizeof(boolean_T));
  vlen = power_range->size[0] * power_range->size[1];
  for (i10 = 0; i10 < vlen; i10++) {
    r0->data[i10] = rtIsNaN(power_range->data[i10].re);
  }

  emxInit_boolean_T(&r1, 2);
  i10 = r1->size[0] * r1->size[1];
  r1->size[0] = power_range->size[0];
  r1->size[1] = power_range->size[1];
  emxEnsureCapacity((emxArray__common *)r1, i10, (int)sizeof(boolean_T));
  vlen = power_range->size[0] * power_range->size[1];
  for (i10 = 0; i10 < vlen; i10++) {
    r1->data[i10] = rtIsNaN(power_range->data[i10].im);
  }

  i10 = r0->size[0] * r0->size[1];
  emxEnsureCapacity((emxArray__common *)r0, i10, (int)sizeof(boolean_T));
  i10 = r0->size[0];
  xoffset = r0->size[1];
  vlen = i10 * xoffset;
  for (i10 = 0; i10 < vlen; i10++) {
    r0->data[i10] = (r0->data[i10] || r1->data[i10]);
  }

  i10 = r1->size[0] * r1->size[1];
  r1->size[0] = power_range->size[0];
  r1->size[1] = power_range->size[1];
  emxEnsureCapacity((emxArray__common *)r1, i10, (int)sizeof(boolean_T));
  vlen = power_range->size[0] * power_range->size[1];
  for (i10 = 0; i10 < vlen; i10++) {
    r1->data[i10] = ((power_range->data[i10].re == 0.0) && (power_range->
      data[i10].im == 0.0));
  }

  emxInit_int32_T1(&r2, 1);
  xoffset = r0->size[0] * r0->size[1] - 1;
  vlen = 0;
  for (asub = 0; asub <= xoffset; asub++) {
    if (r0->data[asub] || r1->data[asub]) {
      vlen++;
    }
  }

  i10 = r2->size[0];
  r2->size[0] = vlen;
  emxEnsureCapacity((emxArray__common *)r2, i10, (int)sizeof(int));
  vlen = 0;
  for (asub = 0; asub <= xoffset; asub++) {
    if (r0->data[asub] || r1->data[asub]) {
      r2->data[vlen] = asub + 1;
      vlen++;
    }
  }

  emxFree_boolean_T(&r1);
  emxFree_boolean_T(&r0);
  vlen = r2->size[0];
  for (i10 = 0; i10 < vlen; i10++) {
    power_range->data[r2->data[i10] - 1].re = 1.0E-100;
    power_range->data[r2->data[i10] - 1].im = 0.0;
  }

  emxFree_int32_T(&r2);

  /*  Use normalized entropy. */
  for (i10 = 0; i10 < 2; i10++) {
    uv0[i10] = (unsigned int)power_range->size[i10];
  }

  i10 = a->size[0] * a->size[1];
  a->size[0] = (int)uv0[0];
  a->size[1] = (int)uv0[1];
  emxEnsureCapacity((emxArray__common *)a, i10, (int)sizeof(creal_T));
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

  i10 = a->size[0] * a->size[1];
  a->size[0] = power_range->size[0];
  a->size[1] = power_range->size[1];
  emxEnsureCapacity((emxArray__common *)a, i10, (int)sizeof(creal_T));
  vlen = power_range->size[0] * power_range->size[1];
  for (i10 = 0; i10 < vlen; i10++) {
    s_re = power_range->data[i10].re;
    s_im = power_range->data[i10].im;
    brm = a->data[i10].re;
    y_re = a->data[i10].im;
    a->data[i10].re = s_re * brm - s_im * y_re;
    a->data[i10].im = s_re * y_re + s_im * brm;
  }

  i10 = entr->size[0] * entr->size[1];
  entr->size[0] = 1;
  entr->size[1] = a->size[1];
  emxEnsureCapacity((emxArray__common *)entr, i10, (int)sizeof(creal_T));
  if ((a->size[0] == 0) || (a->size[1] == 0)) {
    i10 = entr->size[0] * entr->size[1];
    entr->size[0] = 1;
    emxEnsureCapacity((emxArray__common *)entr, i10, (int)sizeof(creal_T));
    vlen = entr->size[1];
    for (i10 = 0; i10 < vlen; i10++) {
      entr->data[entr->size[0] * i10].re = 0.0;
      entr->data[entr->size[0] * i10].im = 0.0;
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
  i10 = entr->size[0] * entr->size[1];
  entr->size[0] = 1;
  emxEnsureCapacity((emxArray__common *)entr, i10, (int)sizeof(creal_T));
  vlen = entr->size[0];
  xoffset = entr->size[1];
  vlen *= xoffset;
  for (i10 = 0; i10 < vlen; i10++) {
    s_im = -entr->data[i10].re;
    brm = -entr->data[i10].im;
    if (brm == 0.0) {
      entr->data[i10].re = s_im / s_re;
      entr->data[i10].im = 0.0;
    } else if (s_im == 0.0) {
      entr->data[i10].re = 0.0;
      entr->data[i10].im = brm / s_re;
    } else {
      entr->data[i10].re = s_im / s_re;
      entr->data[i10].im = brm / s_re;
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
