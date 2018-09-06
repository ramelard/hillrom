/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: extract_vitals_tk1.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 06-Sep-2018 15:49:36
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "extract_vitals_tk1.h"
#include "interp1.h"
#include "extract_vitals_tk1_emxutil.h"
#include "filter.h"
#include "relop.h"
#include "plot_power_spectrum.h"
#include "log.h"
#include "kalmanfilt.h"
#include "exp.h"
#include "abs1.h"
#include "fft.h"
#include "rdivide.h"
#include "sum.h"
#include "get_spectral_entropy.h"
#include "permute.h"
#include "imresize.h"
#include "floor.h"

/* Function Declarations */
static int div_s32(int numerator, int denominator);

/* Function Definitions */

/*
 * Arguments    : int numerator
 *                int denominator
 * Return Type  : int
 */
static int div_s32(int numerator, int denominator)
{
  int quotient;
  unsigned int absNumerator;
  unsigned int absDenominator;
  boolean_T quotientNeedsNegation;
  if (denominator == 0) {
    if (numerator >= 0) {
      quotient = MAX_int32_T;
    } else {
      quotient = MIN_int32_T;
    }
  } else {
    if (numerator < 0) {
      absNumerator = ~(unsigned int)numerator + 1U;
    } else {
      absNumerator = (unsigned int)numerator;
    }

    if (denominator < 0) {
      absDenominator = ~(unsigned int)denominator + 1U;
    } else {
      absDenominator = (unsigned int)denominator;
    }

    quotientNeedsNegation = ((numerator < 0) != (denominator < 0));
    absNumerator /= absDenominator;
    if (quotientNeedsNegation) {
      quotient = -(int)absNumerator;
    } else {
      quotient = (int)absNumerator;
    }
  }

  return quotient;
}

/*
 * assert(isa(frames,'double'))
 *  assert(isa(block_size,'double'))
 * Arguments    : const emxArray_real_T *frames_head
 *                const emxArray_real_T *frames_body
 *                emxArray_real_T *timestamps
 *                double fps
 *                double block_size
 *                double *hr
 *                double *rr
 * Return Type  : void
 */
void extract_vitals_tk1(const emxArray_real_T *frames_head, const
  emxArray_real_T *frames_body, emxArray_real_T *timestamps, double fps, double
  block_size, double *hr, double *rr)
{
  int i0;
  int ixstart;
  int calclen;
  emxArray_real_T *Bbody;
  emxArray_real_T *x;
  int nx;
  emxArray_real_T *Rbody;
  int sz[2];
  int k;
  double mtmp;
  int ix;
  boolean_T exitg14;
  double b_mtmp;
  emxArray_real_T *freq;
  boolean_T exitg13;
  double ndbl;
  double apnd;
  double cdiff;
  double absa;
  double absb;
  emxArray_real_T *Rbody_interp;
  int i;
  emxArray_real_T *heartrate;
  emxArray_int32_T *r0;
  emxArray_real_T *b_Rbody;
  boolean_T exitg12;
  emxArray_real_T *Bhead;
  boolean_T exitg11;
  boolean_T exitg10;
  boolean_T exitg9;
  emxArray_real_T *Rhead_interp;
  emxArray_real_T *c_Rbody;
  boolean_T exitg8;
  boolean_T exitg7;
  double b[7];
  double a[7];
  static const double b_b[7] = { 0.00012962453960508764, -0.0,
    -0.00038887361881526295, -0.0, 0.00038887361881526295, -0.0,
    -0.00012962453960508764 };

  static const double c_b[7] = { 1.7039678072730181E-5, -0.0,
    -5.1119034218190554E-5, -0.0, 5.1119034218190554E-5, -0.0,
    -1.7039678072730181E-5 };

  static const double b_a[7] = { 1.0, -5.6499842920680887, 13.437840425961046,
    -17.216466040870813, 12.530961031088907, -4.9132087149677943,
    0.8109608719072916 };

  emxArray_real_T *Ahead_filt;
  static const double c_a[7] = { 1.0, -5.8594042604279757, 14.341232615905842,
    -18.767329167203371, 13.849161777112428, -5.4642253416872641,
    0.90056608898162283 };

  emxArray_real_T *b_Rhead_interp;
  emxArray_real_T *r1;
  emxArray_creal_T *Y;
  emxArray_creal_T *Fbreathing;
  emxArray_creal_T *Fheartrate;
  emxArray_boolean_T *b_x;
  int ii_data[1];
  boolean_T exitg6;
  double idx1_data[1];
  boolean_T exitg5;
  int idx2_data[1];
  emxArray_creal_T *b_Fheartrate;
  emxArray_creal_T *HRentr;
  emxArray_creal_T *b_HRentr;
  emxArray_creal_T *b_Fbreathing;
  creal_T R;
  emxArray_creal_T *RRentr;
  emxArray_creal_T *b_RRentr;
  emxArray_real_T *b_Bbody;
  emxArray_real_T *r2;
  int br;
  int m;
  creal_T patch[9];
  emxArray_real_T *wrr;
  creal_T c_mtmp;
  boolean_T exitg4;
  emxArray_real_T *breathing;
  boolean_T guard1 = false;
  unsigned int Rbody_idx_0;
  emxArray_creal_T *b_Ahead_filt;
  emxArray_creal_T *b_R;
  emxArray_creal_T *c_R;
  boolean_T exitg3;
  emxArray_real_T *b_wrr;
  emxArray_real_T *b_freq;
  emxArray_creal_T *c_Fheartrate;
  emxArray_creal_T *d_Fheartrate;
  emxArray_real_T *c_freq;
  boolean_T exitg2;
  boolean_T exitg1;

  /*  fprintf('%d,%d',size(frames_head, 1), size(frames_head, 2)); */
  /*  fprintf('%d,%d',size(frames_body, 1), size(frames_body, 2)); */
  /*  fprintf('%d,%d',size(timestamps, 1), size(timestamps, 2)); */
  i0 = timestamps->size[0] * timestamps->size[1];
  timestamps->size[0] = 1;
  emxEnsureCapacity((emxArray__common *)timestamps, i0, (int)sizeof(double));
  ixstart = timestamps->size[0];
  calclen = timestamps->size[1];
  ixstart *= calclen;
  for (i0 = 0; i0 < ixstart; i0++) {
    timestamps->data[i0] *= 1000.0;
  }

  b_floor(timestamps);
  i0 = timestamps->size[0] * timestamps->size[1];
  timestamps->size[0] = 1;
  emxEnsureCapacity((emxArray__common *)timestamps, i0, (int)sizeof(double));
  ixstart = timestamps->size[0];
  calclen = timestamps->size[1];
  ixstart *= calclen;
  for (i0 = 0; i0 < ixstart; i0++) {
    timestamps->data[i0] /= 1000.0;
  }

  emxInit_real_T(&Bbody, 3);
  emxInit_real_T(&x, 3);

  /*  ??? */
  /*  Bbody = permute(frames_body, [2 3 1]);  % make it h x w x t */
  imresize(frames_body, 1.0 / block_size, Bbody);
  permute(Bbody, x);
  nx = x->size[0] * x->size[1] * x->size[2];
  if (timestamps->size[1] > 0) {
    calclen = div_s32(nx, timestamps->size[1]);
  } else {
    calclen = 0;
  }

  emxInit_real_T1(&Rbody, 2);
  sz[0] = timestamps->size[1];
  i0 = Rbody->size[0] * Rbody->size[1];
  Rbody->size[0] = sz[0];
  Rbody->size[1] = calclen;
  emxEnsureCapacity((emxArray__common *)Rbody, i0, (int)sizeof(double));
  for (k = 0; k + 1 <= nx; k++) {
    Rbody->data[k] = x->data[k];
  }

  /*  Interpolate between uneven timestamped measurements */
  ixstart = 1;
  nx = timestamps->size[1];
  mtmp = timestamps->data[0];
  if (timestamps->size[1] > 1) {
    if (rtIsNaN(timestamps->data[0])) {
      ix = 2;
      exitg14 = false;
      while ((!exitg14) && (ix <= nx)) {
        ixstart = ix;
        if (!rtIsNaN(timestamps->data[ix - 1])) {
          mtmp = timestamps->data[ix - 1];
          exitg14 = true;
        } else {
          ix++;
        }
      }
    }

    if (ixstart < timestamps->size[1]) {
      while (ixstart + 1 <= nx) {
        if (timestamps->data[ixstart] < mtmp) {
          mtmp = timestamps->data[ixstart];
        }

        ixstart++;
      }
    }
  }

  ixstart = 1;
  nx = timestamps->size[1];
  b_mtmp = timestamps->data[0];
  if (timestamps->size[1] > 1) {
    if (rtIsNaN(timestamps->data[0])) {
      ix = 2;
      exitg13 = false;
      while ((!exitg13) && (ix <= nx)) {
        ixstart = ix;
        if (!rtIsNaN(timestamps->data[ix - 1])) {
          b_mtmp = timestamps->data[ix - 1];
          exitg13 = true;
        } else {
          ix++;
        }
      }
    }

    if (ixstart < timestamps->size[1]) {
      while (ixstart + 1 <= nx) {
        if (timestamps->data[ixstart] > b_mtmp) {
          b_mtmp = timestamps->data[ixstart];
        }

        ixstart++;
      }
    }
  }

  emxInit_real_T1(&freq, 2);
  if (rtIsNaN(mtmp) || rtIsNaN(b_mtmp)) {
    i0 = freq->size[0] * freq->size[1];
    freq->size[0] = 1;
    freq->size[1] = 1;
    emxEnsureCapacity((emxArray__common *)freq, i0, (int)sizeof(double));
    freq->data[0] = rtNaN;
  } else if (b_mtmp < mtmp) {
    i0 = freq->size[0] * freq->size[1];
    freq->size[0] = 1;
    freq->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)freq, i0, (int)sizeof(double));
  } else if ((rtIsInf(mtmp) || rtIsInf(b_mtmp)) && (mtmp == b_mtmp)) {
    i0 = freq->size[0] * freq->size[1];
    freq->size[0] = 1;
    freq->size[1] = 1;
    emxEnsureCapacity((emxArray__common *)freq, i0, (int)sizeof(double));
    freq->data[0] = rtNaN;
  } else {
    ndbl = floor((b_mtmp - mtmp) / 0.033333333333333333 + 0.5);
    apnd = mtmp + ndbl * 0.033333333333333333;
    cdiff = apnd - b_mtmp;
    absa = fabs(mtmp);
    absb = fabs(b_mtmp);
    if ((absa >= absb) || rtIsNaN(absb)) {
      absb = absa;
    }

    if (fabs(cdiff) < 4.4408920985006262E-16 * absb) {
      ndbl++;
      apnd = b_mtmp;
    } else if (cdiff > 0.0) {
      apnd = mtmp + (ndbl - 1.0) * 0.033333333333333333;
    } else {
      ndbl++;
    }

    if (ndbl >= 0.0) {
      nx = (int)ndbl;
    } else {
      nx = 0;
    }

    i0 = freq->size[0] * freq->size[1];
    freq->size[0] = 1;
    freq->size[1] = nx;
    emxEnsureCapacity((emxArray__common *)freq, i0, (int)sizeof(double));
    if (nx > 0) {
      freq->data[0] = mtmp;
      if (nx > 1) {
        freq->data[nx - 1] = apnd;
        ixstart = (nx - 1) / 2;
        for (k = 1; k < ixstart; k++) {
          ndbl = (double)k * 0.033333333333333333;
          freq->data[k] = mtmp + ndbl;
          freq->data[(nx - k) - 1] = apnd - ndbl;
        }

        if (ixstart << 1 == nx - 1) {
          freq->data[ixstart] = (mtmp + apnd) / 2.0;
        } else {
          ndbl = (double)ixstart * 0.033333333333333333;
          freq->data[ixstart] = mtmp + ndbl;
          freq->data[ixstart + 1] = apnd - ndbl;
        }
      }
    }
  }

  emxInit_real_T1(&Rbody_interp, 2);
  i0 = Rbody_interp->size[0] * Rbody_interp->size[1];
  Rbody_interp->size[0] = freq->size[1];
  Rbody_interp->size[1] = Rbody->size[1];
  emxEnsureCapacity((emxArray__common *)Rbody_interp, i0, (int)sizeof(double));
  ixstart = freq->size[1] * Rbody->size[1];
  for (i0 = 0; i0 < ixstart; i0++) {
    Rbody_interp->data[i0] = 0.0;
  }

  i = 0;
  emxInit_real_T1(&heartrate, 2);
  emxInit_int32_T(&r0, 1);
  emxInit_real_T1(&b_Rbody, 2);
  while (i <= Rbody->size[1] - 1) {
    ixstart = Rbody_interp->size[0];
    i0 = r0->size[0];
    r0->size[0] = ixstart;
    emxEnsureCapacity((emxArray__common *)r0, i0, (int)sizeof(int));
    for (i0 = 0; i0 < ixstart; i0++) {
      r0->data[i0] = i0;
    }

    ixstart = 1;
    nx = timestamps->size[1];
    mtmp = timestamps->data[0];
    if (timestamps->size[1] > 1) {
      if (rtIsNaN(timestamps->data[0])) {
        ix = 2;
        exitg12 = false;
        while ((!exitg12) && (ix <= nx)) {
          ixstart = ix;
          if (!rtIsNaN(timestamps->data[ix - 1])) {
            mtmp = timestamps->data[ix - 1];
            exitg12 = true;
          } else {
            ix++;
          }
        }
      }

      if (ixstart < timestamps->size[1]) {
        while (ixstart + 1 <= nx) {
          if (timestamps->data[ixstart] < mtmp) {
            mtmp = timestamps->data[ixstart];
          }

          ixstart++;
        }
      }
    }

    ixstart = 1;
    nx = timestamps->size[1];
    b_mtmp = timestamps->data[0];
    if (timestamps->size[1] > 1) {
      if (rtIsNaN(timestamps->data[0])) {
        ix = 2;
        exitg11 = false;
        while ((!exitg11) && (ix <= nx)) {
          ixstart = ix;
          if (!rtIsNaN(timestamps->data[ix - 1])) {
            b_mtmp = timestamps->data[ix - 1];
            exitg11 = true;
          } else {
            ix++;
          }
        }
      }

      if (ixstart < timestamps->size[1]) {
        while (ixstart + 1 <= nx) {
          if (timestamps->data[ixstart] > b_mtmp) {
            b_mtmp = timestamps->data[ixstart];
          }

          ixstart++;
        }
      }
    }

    if (rtIsNaN(mtmp) || rtIsNaN(b_mtmp)) {
      i0 = heartrate->size[0] * heartrate->size[1];
      heartrate->size[0] = 1;
      heartrate->size[1] = 1;
      emxEnsureCapacity((emxArray__common *)heartrate, i0, (int)sizeof(double));
      heartrate->data[0] = rtNaN;
    } else if (b_mtmp < mtmp) {
      i0 = heartrate->size[0] * heartrate->size[1];
      heartrate->size[0] = 1;
      heartrate->size[1] = 0;
      emxEnsureCapacity((emxArray__common *)heartrate, i0, (int)sizeof(double));
    } else if ((rtIsInf(mtmp) || rtIsInf(b_mtmp)) && (mtmp == b_mtmp)) {
      i0 = heartrate->size[0] * heartrate->size[1];
      heartrate->size[0] = 1;
      heartrate->size[1] = 1;
      emxEnsureCapacity((emxArray__common *)heartrate, i0, (int)sizeof(double));
      heartrate->data[0] = rtNaN;
    } else {
      ndbl = floor((b_mtmp - mtmp) / 0.033333333333333333 + 0.5);
      apnd = mtmp + ndbl * 0.033333333333333333;
      cdiff = apnd - b_mtmp;
      absa = fabs(mtmp);
      absb = fabs(b_mtmp);
      if ((absa >= absb) || rtIsNaN(absb)) {
        absb = absa;
      }

      if (fabs(cdiff) < 4.4408920985006262E-16 * absb) {
        ndbl++;
        apnd = b_mtmp;
      } else if (cdiff > 0.0) {
        apnd = mtmp + (ndbl - 1.0) * 0.033333333333333333;
      } else {
        ndbl++;
      }

      if (ndbl >= 0.0) {
        nx = (int)ndbl;
      } else {
        nx = 0;
      }

      i0 = heartrate->size[0] * heartrate->size[1];
      heartrate->size[0] = 1;
      heartrate->size[1] = nx;
      emxEnsureCapacity((emxArray__common *)heartrate, i0, (int)sizeof(double));
      if (nx > 0) {
        heartrate->data[0] = mtmp;
        if (nx > 1) {
          heartrate->data[nx - 1] = apnd;
          ixstart = (nx - 1) / 2;
          for (k = 1; k < ixstart; k++) {
            ndbl = (double)k * 0.033333333333333333;
            heartrate->data[k] = mtmp + ndbl;
            heartrate->data[(nx - k) - 1] = apnd - ndbl;
          }

          if (ixstart << 1 == nx - 1) {
            heartrate->data[ixstart] = (mtmp + apnd) / 2.0;
          } else {
            ndbl = (double)ixstart * 0.033333333333333333;
            heartrate->data[ixstart] = mtmp + ndbl;
            heartrate->data[ixstart + 1] = apnd - ndbl;
          }
        }
      }
    }

    ixstart = Rbody->size[0];
    i0 = b_Rbody->size[0] * b_Rbody->size[1];
    b_Rbody->size[0] = 1;
    b_Rbody->size[1] = ixstart;
    emxEnsureCapacity((emxArray__common *)b_Rbody, i0, (int)sizeof(double));
    for (i0 = 0; i0 < ixstart; i0++) {
      b_Rbody->data[b_Rbody->size[0] * i0] = Rbody->data[i0 + Rbody->size[0] * i];
    }

    interp1(timestamps, b_Rbody, heartrate, freq);
    calclen = r0->size[0];
    for (i0 = 0; i0 < calclen; i0++) {
      Rbody_interp->data[r0->data[i0] + Rbody_interp->size[0] * i] = freq->
        data[i0];
    }

    i++;
  }

  emxFree_real_T(&b_Rbody);
  i0 = Rbody_interp->size[0] * Rbody_interp->size[1];
  emxEnsureCapacity((emxArray__common *)Rbody_interp, i0, (int)sizeof(double));
  ixstart = Rbody_interp->size[0];
  calclen = Rbody_interp->size[1];
  ixstart *= calclen;
  for (i0 = 0; i0 < ixstart; i0++) {
    Rbody_interp->data[i0]++;
  }

  c_log(Rbody_interp);
  i0 = Rbody_interp->size[0] * Rbody_interp->size[1];
  emxEnsureCapacity((emxArray__common *)Rbody_interp, i0, (int)sizeof(double));
  ixstart = Rbody_interp->size[0];
  calclen = Rbody_interp->size[1];
  ixstart *= calclen;
  for (i0 = 0; i0 < ixstart; i0++) {
    Rbody_interp->data[i0] = -Rbody_interp->data[i0];
  }

  emxInit_real_T(&Bhead, 3);

  /*  Bhead = permute(frames_head, [2 3 1]);  % make it h x w x t */
  imresize(frames_head, 1.0 / block_size, Bhead);
  permute(Bhead, x);
  nx = x->size[0] * x->size[1] * x->size[2];
  emxFree_real_T(&Bhead);
  if (timestamps->size[1] > 0) {
    calclen = div_s32(nx, timestamps->size[1]);
  } else {
    calclen = 0;
  }

  ixstart = timestamps->size[1];
  i0 = Rbody->size[0] * Rbody->size[1];
  Rbody->size[0] = ixstart;
  Rbody->size[1] = calclen;
  emxEnsureCapacity((emxArray__common *)Rbody, i0, (int)sizeof(double));
  for (k = 0; k + 1 <= nx; k++) {
    Rbody->data[k] = x->data[k];
  }

  emxFree_real_T(&x);

  /*  Interpolate between uneven timestamped measurements */
  ixstart = 1;
  nx = timestamps->size[1];
  mtmp = timestamps->data[0];
  if (timestamps->size[1] > 1) {
    if (rtIsNaN(timestamps->data[0])) {
      ix = 2;
      exitg10 = false;
      while ((!exitg10) && (ix <= nx)) {
        ixstart = ix;
        if (!rtIsNaN(timestamps->data[ix - 1])) {
          mtmp = timestamps->data[ix - 1];
          exitg10 = true;
        } else {
          ix++;
        }
      }
    }

    if (ixstart < timestamps->size[1]) {
      while (ixstart + 1 <= nx) {
        if (timestamps->data[ixstart] < mtmp) {
          mtmp = timestamps->data[ixstart];
        }

        ixstart++;
      }
    }
  }

  ixstart = 1;
  nx = timestamps->size[1];
  b_mtmp = timestamps->data[0];
  if (timestamps->size[1] > 1) {
    if (rtIsNaN(timestamps->data[0])) {
      ix = 2;
      exitg9 = false;
      while ((!exitg9) && (ix <= nx)) {
        ixstart = ix;
        if (!rtIsNaN(timestamps->data[ix - 1])) {
          b_mtmp = timestamps->data[ix - 1];
          exitg9 = true;
        } else {
          ix++;
        }
      }
    }

    if (ixstart < timestamps->size[1]) {
      while (ixstart + 1 <= nx) {
        if (timestamps->data[ixstart] > b_mtmp) {
          b_mtmp = timestamps->data[ixstart];
        }

        ixstart++;
      }
    }
  }

  if (rtIsNaN(mtmp) || rtIsNaN(b_mtmp)) {
    i0 = freq->size[0] * freq->size[1];
    freq->size[0] = 1;
    freq->size[1] = 1;
    emxEnsureCapacity((emxArray__common *)freq, i0, (int)sizeof(double));
    freq->data[0] = rtNaN;
  } else if (b_mtmp < mtmp) {
    i0 = freq->size[0] * freq->size[1];
    freq->size[0] = 1;
    freq->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)freq, i0, (int)sizeof(double));
  } else if ((rtIsInf(mtmp) || rtIsInf(b_mtmp)) && (mtmp == b_mtmp)) {
    i0 = freq->size[0] * freq->size[1];
    freq->size[0] = 1;
    freq->size[1] = 1;
    emxEnsureCapacity((emxArray__common *)freq, i0, (int)sizeof(double));
    freq->data[0] = rtNaN;
  } else {
    ndbl = floor((b_mtmp - mtmp) / 0.033333333333333333 + 0.5);
    apnd = mtmp + ndbl * 0.033333333333333333;
    cdiff = apnd - b_mtmp;
    absa = fabs(mtmp);
    absb = fabs(b_mtmp);
    if ((absa >= absb) || rtIsNaN(absb)) {
      absb = absa;
    }

    if (fabs(cdiff) < 4.4408920985006262E-16 * absb) {
      ndbl++;
      apnd = b_mtmp;
    } else if (cdiff > 0.0) {
      apnd = mtmp + (ndbl - 1.0) * 0.033333333333333333;
    } else {
      ndbl++;
    }

    if (ndbl >= 0.0) {
      nx = (int)ndbl;
    } else {
      nx = 0;
    }

    i0 = freq->size[0] * freq->size[1];
    freq->size[0] = 1;
    freq->size[1] = nx;
    emxEnsureCapacity((emxArray__common *)freq, i0, (int)sizeof(double));
    if (nx > 0) {
      freq->data[0] = mtmp;
      if (nx > 1) {
        freq->data[nx - 1] = apnd;
        ixstart = (nx - 1) / 2;
        for (k = 1; k < ixstart; k++) {
          ndbl = (double)k * 0.033333333333333333;
          freq->data[k] = mtmp + ndbl;
          freq->data[(nx - k) - 1] = apnd - ndbl;
        }

        if (ixstart << 1 == nx - 1) {
          freq->data[ixstart] = (mtmp + apnd) / 2.0;
        } else {
          ndbl = (double)ixstart * 0.033333333333333333;
          freq->data[ixstart] = mtmp + ndbl;
          freq->data[ixstart + 1] = apnd - ndbl;
        }
      }
    }
  }

  emxInit_real_T1(&Rhead_interp, 2);
  i0 = Rhead_interp->size[0] * Rhead_interp->size[1];
  Rhead_interp->size[0] = freq->size[1];
  Rhead_interp->size[1] = Rbody->size[1];
  emxEnsureCapacity((emxArray__common *)Rhead_interp, i0, (int)sizeof(double));
  ixstart = freq->size[1] * Rbody->size[1];
  for (i0 = 0; i0 < ixstart; i0++) {
    Rhead_interp->data[i0] = 0.0;
  }

  i = 0;
  emxInit_real_T1(&c_Rbody, 2);
  while (i <= Rbody->size[1] - 1) {
    ixstart = Rhead_interp->size[0];
    i0 = r0->size[0];
    r0->size[0] = ixstart;
    emxEnsureCapacity((emxArray__common *)r0, i0, (int)sizeof(int));
    for (i0 = 0; i0 < ixstart; i0++) {
      r0->data[i0] = i0;
    }

    ixstart = 1;
    nx = timestamps->size[1];
    mtmp = timestamps->data[0];
    if (timestamps->size[1] > 1) {
      if (rtIsNaN(timestamps->data[0])) {
        ix = 2;
        exitg8 = false;
        while ((!exitg8) && (ix <= nx)) {
          ixstart = ix;
          if (!rtIsNaN(timestamps->data[ix - 1])) {
            mtmp = timestamps->data[ix - 1];
            exitg8 = true;
          } else {
            ix++;
          }
        }
      }

      if (ixstart < timestamps->size[1]) {
        while (ixstart + 1 <= nx) {
          if (timestamps->data[ixstart] < mtmp) {
            mtmp = timestamps->data[ixstart];
          }

          ixstart++;
        }
      }
    }

    ixstart = 1;
    nx = timestamps->size[1];
    b_mtmp = timestamps->data[0];
    if (timestamps->size[1] > 1) {
      if (rtIsNaN(timestamps->data[0])) {
        ix = 2;
        exitg7 = false;
        while ((!exitg7) && (ix <= nx)) {
          ixstart = ix;
          if (!rtIsNaN(timestamps->data[ix - 1])) {
            b_mtmp = timestamps->data[ix - 1];
            exitg7 = true;
          } else {
            ix++;
          }
        }
      }

      if (ixstart < timestamps->size[1]) {
        while (ixstart + 1 <= nx) {
          if (timestamps->data[ixstart] > b_mtmp) {
            b_mtmp = timestamps->data[ixstart];
          }

          ixstart++;
        }
      }
    }

    if (rtIsNaN(mtmp) || rtIsNaN(b_mtmp)) {
      i0 = heartrate->size[0] * heartrate->size[1];
      heartrate->size[0] = 1;
      heartrate->size[1] = 1;
      emxEnsureCapacity((emxArray__common *)heartrate, i0, (int)sizeof(double));
      heartrate->data[0] = rtNaN;
    } else if (b_mtmp < mtmp) {
      i0 = heartrate->size[0] * heartrate->size[1];
      heartrate->size[0] = 1;
      heartrate->size[1] = 0;
      emxEnsureCapacity((emxArray__common *)heartrate, i0, (int)sizeof(double));
    } else if ((rtIsInf(mtmp) || rtIsInf(b_mtmp)) && (mtmp == b_mtmp)) {
      i0 = heartrate->size[0] * heartrate->size[1];
      heartrate->size[0] = 1;
      heartrate->size[1] = 1;
      emxEnsureCapacity((emxArray__common *)heartrate, i0, (int)sizeof(double));
      heartrate->data[0] = rtNaN;
    } else {
      ndbl = floor((b_mtmp - mtmp) / 0.033333333333333333 + 0.5);
      apnd = mtmp + ndbl * 0.033333333333333333;
      cdiff = apnd - b_mtmp;
      absa = fabs(mtmp);
      absb = fabs(b_mtmp);
      if ((absa >= absb) || rtIsNaN(absb)) {
        absb = absa;
      }

      if (fabs(cdiff) < 4.4408920985006262E-16 * absb) {
        ndbl++;
        apnd = b_mtmp;
      } else if (cdiff > 0.0) {
        apnd = mtmp + (ndbl - 1.0) * 0.033333333333333333;
      } else {
        ndbl++;
      }

      if (ndbl >= 0.0) {
        nx = (int)ndbl;
      } else {
        nx = 0;
      }

      i0 = heartrate->size[0] * heartrate->size[1];
      heartrate->size[0] = 1;
      heartrate->size[1] = nx;
      emxEnsureCapacity((emxArray__common *)heartrate, i0, (int)sizeof(double));
      if (nx > 0) {
        heartrate->data[0] = mtmp;
        if (nx > 1) {
          heartrate->data[nx - 1] = apnd;
          ixstart = (nx - 1) / 2;
          for (k = 1; k < ixstart; k++) {
            ndbl = (double)k * 0.033333333333333333;
            heartrate->data[k] = mtmp + ndbl;
            heartrate->data[(nx - k) - 1] = apnd - ndbl;
          }

          if (ixstart << 1 == nx - 1) {
            heartrate->data[ixstart] = (mtmp + apnd) / 2.0;
          } else {
            ndbl = (double)ixstart * 0.033333333333333333;
            heartrate->data[ixstart] = mtmp + ndbl;
            heartrate->data[ixstart + 1] = apnd - ndbl;
          }
        }
      }
    }

    ixstart = Rbody->size[0];
    i0 = c_Rbody->size[0] * c_Rbody->size[1];
    c_Rbody->size[0] = 1;
    c_Rbody->size[1] = ixstart;
    emxEnsureCapacity((emxArray__common *)c_Rbody, i0, (int)sizeof(double));
    for (i0 = 0; i0 < ixstart; i0++) {
      c_Rbody->data[c_Rbody->size[0] * i0] = Rbody->data[i0 + Rbody->size[0] * i];
    }

    interp1(timestamps, c_Rbody, heartrate, freq);
    calclen = r0->size[0];
    for (i0 = 0; i0 < calclen; i0++) {
      Rhead_interp->data[r0->data[i0] + Rhead_interp->size[0] * i] = freq->
        data[i0];
    }

    i++;
  }

  emxFree_real_T(&c_Rbody);
  i0 = Rhead_interp->size[0] * Rhead_interp->size[1];
  emxEnsureCapacity((emxArray__common *)Rhead_interp, i0, (int)sizeof(double));
  ixstart = Rhead_interp->size[0];
  calclen = Rhead_interp->size[1];
  ixstart *= calclen;
  for (i0 = 0; i0 < ixstart; i0++) {
    Rhead_interp->data[i0]++;
  }

  c_log(Rhead_interp);
  i0 = Rhead_interp->size[0] * Rhead_interp->size[1];
  emxEnsureCapacity((emxArray__common *)Rhead_interp, i0, (int)sizeof(double));
  ixstart = Rhead_interp->size[0];
  calclen = Rhead_interp->size[1];
  ixstart *= calclen;
  for (i0 = 0; i0 < ixstart; i0++) {
    Rhead_interp->data[i0] = -Rhead_interp->data[i0];
  }

  /*  */
  /*  breathing = mean(Abody,2); */
  /*  heartrate = mean(Ahead,2); */
  /*  Get rid of breathing component of heartrate signal. */
  /*  low_freq = 30/60; */
  /*  high_freq = 350/60; */
  /*  x_ts = timeseries(Ahead, [0:T-1]./60); */
  /*  x_filt = idealfilter(x_ts, [low_freq high_freq], 'pass'); */
  /*  Ahead_filt = x_filt.Data; */
  /*  butter() requires constant valued inputs, so put in if statements. */
  if (fabs(fps - 30.0) < 0.1) {
    for (i0 = 0; i0 < 7; i0++) {
      b[i0] = b_b[i0];
      a[i0] = b_a[i0];
    }
  } else {
    if (fabs(fps - 60.0) < 0.1) {
      for (i0 = 0; i0 < 7; i0++) {
        b[i0] = c_b[i0];
        a[i0] = c_a[i0];
      }
    }
  }

  for (i0 = 0; i0 < 2; i0++) {
    sz[i0] = Rhead_interp->size[i0];
  }

  emxInit_real_T1(&Ahead_filt, 2);
  i0 = Ahead_filt->size[0] * Ahead_filt->size[1];
  Ahead_filt->size[0] = sz[0];
  Ahead_filt->size[1] = sz[1];
  emxEnsureCapacity((emxArray__common *)Ahead_filt, i0, (int)sizeof(double));
  ixstart = sz[0] * sz[1];
  for (i0 = 0; i0 < ixstart; i0++) {
    Ahead_filt->data[i0] = 0.0;
  }

  i = 0;
  emxInit_real_T2(&b_Rhead_interp, 1);
  emxInit_real_T2(&r1, 1);
  while (i <= sz[1] - 1) {
    ixstart = Rhead_interp->size[0];
    i0 = b_Rhead_interp->size[0];
    b_Rhead_interp->size[0] = ixstart;
    emxEnsureCapacity((emxArray__common *)b_Rhead_interp, i0, (int)sizeof(double));
    for (i0 = 0; i0 < ixstart; i0++) {
      b_Rhead_interp->data[i0] = Rhead_interp->data[i0 + Rhead_interp->size[0] *
        i];
    }

    filter(b, a, b_Rhead_interp, r1);
    ixstart = r1->size[0];
    for (i0 = 0; i0 < ixstart; i0++) {
      Ahead_filt->data[i0 + Ahead_filt->size[0] * i] = r1->data[i0];
    }

    i++;
  }

  emxFree_real_T(&r1);
  emxFree_real_T(&b_Rhead_interp);
  emxInit_creal_T(&Y, 2);

  /*  Abody = zeros(size(Abody)); */
  /*  for i = 1:size(Abody,2) */
  /*    Abody(:,i) = filter(b, a, Abody(:,i)); */
  /*  end */
  /*  Do weighted average of signals based on (inverse) entropy to yield a */
  /*  signal (heartrate and breathing). */
  /*  power = (a^2+b^2)/N */
  /*  Should be same as 1/N*abs(Y)^2 */
  fft(Rbody_interp, Rbody_interp->size[0], Y);
  i0 = Y->size[0] * Y->size[1];
  emxEnsureCapacity((emxArray__common *)Y, i0, (int)sizeof(creal_T));
  ixstart = Y->size[0];
  calclen = Y->size[1];
  nx = Rbody_interp->size[0];
  ixstart *= calclen;
  for (i0 = 0; i0 < ixstart; i0++) {
    b_mtmp = Y->data[i0].re;
    ndbl = -Y->data[i0].im;
    cdiff = Y->data[i0].re * b_mtmp - Y->data[i0].im * ndbl;
    ndbl = Y->data[i0].re * ndbl + Y->data[i0].im * b_mtmp;
    if (ndbl == 0.0) {
      Y->data[i0].re = cdiff / (double)nx;
      Y->data[i0].im = 0.0;
    } else if (cdiff == 0.0) {
      Y->data[i0].re = 0.0;
      Y->data[i0].im = ndbl / (double)nx;
    } else {
      Y->data[i0].re = cdiff / (double)nx;
      Y->data[i0].im = ndbl / (double)nx;
    }
  }

  i0 = (int)floor((double)Rbody_interp->size[0] / 2.0);
  if (1 > i0) {
    ixstart = 0;
  } else {
    ixstart = i0;
  }

  emxInit_creal_T(&Fbreathing, 2);
  calclen = Y->size[1];
  i0 = Fbreathing->size[0] * Fbreathing->size[1];
  Fbreathing->size[0] = ixstart;
  Fbreathing->size[1] = calclen;
  emxEnsureCapacity((emxArray__common *)Fbreathing, i0, (int)sizeof(creal_T));
  for (i0 = 0; i0 < calclen; i0++) {
    for (k = 0; k < ixstart; k++) {
      Fbreathing->data[k + Fbreathing->size[0] * i0] = Y->data[k + Y->size[0] *
        i0];
    }
  }

  fft(Rhead_interp, Rbody_interp->size[0], Y);
  i0 = Y->size[0] * Y->size[1];
  emxEnsureCapacity((emxArray__common *)Y, i0, (int)sizeof(creal_T));
  ixstart = Y->size[0];
  calclen = Y->size[1];
  nx = Rbody_interp->size[0];
  ixstart *= calclen;
  emxFree_real_T(&Rhead_interp);
  for (i0 = 0; i0 < ixstart; i0++) {
    b_mtmp = Y->data[i0].re;
    ndbl = -Y->data[i0].im;
    cdiff = Y->data[i0].re * b_mtmp - Y->data[i0].im * ndbl;
    ndbl = Y->data[i0].re * ndbl + Y->data[i0].im * b_mtmp;
    if (ndbl == 0.0) {
      Y->data[i0].re = cdiff / (double)nx;
      Y->data[i0].im = 0.0;
    } else if (cdiff == 0.0) {
      Y->data[i0].re = 0.0;
      Y->data[i0].im = ndbl / (double)nx;
    } else {
      Y->data[i0].re = cdiff / (double)nx;
      Y->data[i0].im = ndbl / (double)nx;
    }
  }

  ndbl = fps / (double)Rbody_interp->size[0];
  b_mtmp = (double)Rbody_interp->size[0] / 2.0;
  ixstart = (int)floor(b_mtmp);
  if (ixstart - 1 < 0) {
    i0 = freq->size[0] * freq->size[1];
    freq->size[0] = 1;
    freq->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)freq, i0, (int)sizeof(double));
  } else {
    i0 = freq->size[0] * freq->size[1];
    freq->size[0] = 1;
    freq->size[1] = ixstart;
    emxEnsureCapacity((emxArray__common *)freq, i0, (int)sizeof(double));
    ixstart--;
    for (i0 = 0; i0 <= ixstart; i0++) {
      freq->data[freq->size[0] * i0] = i0;
    }
  }

  i0 = freq->size[0] * freq->size[1];
  freq->size[0] = 1;
  emxEnsureCapacity((emxArray__common *)freq, i0, (int)sizeof(double));
  ixstart = freq->size[0];
  calclen = freq->size[1];
  ixstart *= calclen;
  for (i0 = 0; i0 < ixstart; i0++) {
    freq->data[i0] *= ndbl;
  }

  i0 = (int)floor((double)Rbody_interp->size[0] / 2.0);
  if (1 > i0) {
    ixstart = 0;
  } else {
    ixstart = i0;
  }

  emxInit_creal_T(&Fheartrate, 2);
  calclen = Y->size[1];
  i0 = Fheartrate->size[0] * Fheartrate->size[1];
  Fheartrate->size[0] = ixstart;
  Fheartrate->size[1] = calclen;
  emxEnsureCapacity((emxArray__common *)Fheartrate, i0, (int)sizeof(creal_T));
  for (i0 = 0; i0 < calclen; i0++) {
    for (k = 0; k < ixstart; k++) {
      Fheartrate->data[k + Fheartrate->size[0] * i0] = Y->data[k + Y->size[0] *
        i0];
    }
  }

  emxInit_boolean_T(&b_x, 2);

  /*  [freq, Fbreathing] = plot_power_spectrum(Abody, 60); */
  /*  [freq, Fheartrate] = plot_power_spectrum(Ahead_filt, 60); */
  i0 = b_x->size[0] * b_x->size[1];
  b_x->size[0] = 1;
  b_x->size[1] = freq->size[1];
  emxEnsureCapacity((emxArray__common *)b_x, i0, (int)sizeof(boolean_T));
  ixstart = freq->size[0] * freq->size[1];
  for (i0 = 0; i0 < ixstart; i0++) {
    b_x->data[i0] = (freq->data[i0] > 0.083333333333333329);
  }

  k = (1 <= b_x->size[1]);
  ixstart = 0;
  calclen = 1;
  exitg6 = false;
  while ((!exitg6) && (calclen <= b_x->size[1])) {
    if (b_x->data[calclen - 1]) {
      ixstart = 1;
      ii_data[0] = calclen;
      exitg6 = true;
    } else {
      calclen++;
    }
  }

  if (k == 1) {
    if (ixstart == 0) {
      k = 0;
    }
  } else {
    k = !(1 > ixstart);
  }

  for (i0 = 0; i0 < k; i0++) {
    idx1_data[i0] = (double)ii_data[i0] - 1.0;
  }

  i0 = b_x->size[0] * b_x->size[1];
  b_x->size[0] = 1;
  b_x->size[1] = freq->size[1];
  emxEnsureCapacity((emxArray__common *)b_x, i0, (int)sizeof(boolean_T));
  ixstart = freq->size[0] * freq->size[1];
  for (i0 = 0; i0 < ixstart; i0++) {
    b_x->data[i0] = (freq->data[i0] > 0.5);
  }

  k = (1 <= b_x->size[1]);
  ixstart = 0;
  calclen = 1;
  exitg5 = false;
  while ((!exitg5) && (calclen <= b_x->size[1])) {
    if (b_x->data[calclen - 1]) {
      ixstart = 1;
      ii_data[0] = calclen;
      exitg5 = true;
    } else {
      calclen++;
    }
  }

  emxFree_boolean_T(&b_x);
  if (k == 1) {
    if (ixstart == 0) {
      k = 0;
    }
  } else {
    k = !(1 > ixstart);
  }

  for (i0 = 0; i0 < k; i0++) {
    idx2_data[i0] = ii_data[i0];
  }

  if (idx1_data[0] < 1.0) {
    i0 = heartrate->size[0] * heartrate->size[1];
    heartrate->size[0] = 1;
    heartrate->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)heartrate, i0, (int)sizeof(double));
  } else {
    i0 = heartrate->size[0] * heartrate->size[1];
    heartrate->size[0] = 1;
    heartrate->size[1] = (int)idx1_data[0];
    emxEnsureCapacity((emxArray__common *)heartrate, i0, (int)sizeof(double));
    ixstart = (int)idx1_data[0] - 1;
    for (i0 = 0; i0 <= ixstart; i0++) {
      heartrate->data[heartrate->size[0] * i0] = 1.0 + (double)i0;
    }
  }

  if (Fbreathing->size[0] < idx2_data[0]) {
    i0 = freq->size[0] * freq->size[1];
    freq->size[0] = 1;
    freq->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)freq, i0, (int)sizeof(double));
  } else {
    i0 = Fbreathing->size[0];
    k = freq->size[0] * freq->size[1];
    freq->size[0] = 1;
    freq->size[1] = (i0 - idx2_data[0]) + 1;
    emxEnsureCapacity((emxArray__common *)freq, k, (int)sizeof(double));
    ixstart = i0 - idx2_data[0];
    for (i0 = 0; i0 <= ixstart; i0++) {
      freq->data[freq->size[0] * i0] = idx2_data[0] + i0;
    }
  }

  i0 = r0->size[0];
  r0->size[0] = heartrate->size[1] + freq->size[1];
  emxEnsureCapacity((emxArray__common *)r0, i0, (int)sizeof(int));
  ixstart = heartrate->size[1];
  for (i0 = 0; i0 < ixstart; i0++) {
    r0->data[i0] = (int)heartrate->data[heartrate->size[0] * i0];
  }

  ixstart = freq->size[1];
  for (i0 = 0; i0 < ixstart; i0++) {
    r0->data[i0 + heartrate->size[1]] = (int)freq->data[freq->size[0] * i0];
  }

  ixstart = Fbreathing->size[1];
  calclen = r0->size[0];
  for (i0 = 0; i0 < ixstart; i0++) {
    for (k = 0; k < calclen; k++) {
      Fbreathing->data[(r0->data[k] + Fbreathing->size[0] * i0) - 1].re = 0.0;
      Fbreathing->data[(r0->data[k] + Fbreathing->size[0] * i0) - 1].im = 0.0;
    }
  }

  ixstart = Y->size[1];
  for (i0 = 0; i0 < ixstart; i0++) {
    Fheartrate->data[Fheartrate->size[0] * i0].re = 0.0;
    Fheartrate->data[Fheartrate->size[0] * i0].im = 0.0;
  }

  emxInit_creal_T(&b_Fheartrate, 2);

  /*  Fbreathing(1,:) = zeros(1, size(Fbreathing,2)); */
  /*  Fheartrate(1,:) = zeros(1, size(Fheartrate,2)); */
  i0 = b_Fheartrate->size[0] * b_Fheartrate->size[1];
  b_Fheartrate->size[0] = Fheartrate->size[0];
  b_Fheartrate->size[1] = Fheartrate->size[1];
  emxEnsureCapacity((emxArray__common *)b_Fheartrate, i0, (int)sizeof(creal_T));
  ixstart = Fheartrate->size[0] * Fheartrate->size[1];
  for (i0 = 0; i0 < ixstart; i0++) {
    b_Fheartrate->data[i0] = Fheartrate->data[i0];
  }

  emxFree_creal_T(&Fheartrate);
  emxInit_creal_T(&HRentr, 2);
  get_spectral_entropy(b_Fheartrate, HRentr);
  i0 = HRentr->size[0] * HRentr->size[1];
  HRentr->size[0] = 1;
  emxEnsureCapacity((emxArray__common *)HRentr, i0, (int)sizeof(creal_T));
  ixstart = HRentr->size[0];
  calclen = HRentr->size[1];
  ixstart *= calclen;
  emxFree_creal_T(&b_Fheartrate);
  for (i0 = 0; i0 < ixstart; i0++) {
    HRentr->data[i0].re = 1.0 - HRentr->data[i0].re;
    HRentr->data[i0].im = 0.0 - HRentr->data[i0].im;
  }

  emxInit_creal_T(&b_HRentr, 2);
  i0 = b_HRentr->size[0] * b_HRentr->size[1];
  b_HRentr->size[0] = 1;
  b_HRentr->size[1] = HRentr->size[1];
  emxEnsureCapacity((emxArray__common *)b_HRentr, i0, (int)sizeof(creal_T));
  ixstart = HRentr->size[0] * HRentr->size[1];
  for (i0 = 0; i0 < ixstart; i0++) {
    b_HRentr->data[i0] = HRentr->data[i0];
  }

  emxInit_creal_T(&b_Fbreathing, 2);
  R = sum(HRentr);
  b_rdivide(b_HRentr, R, HRentr);
  i0 = b_Fbreathing->size[0] * b_Fbreathing->size[1];
  b_Fbreathing->size[0] = Fbreathing->size[0];
  b_Fbreathing->size[1] = Fbreathing->size[1];
  emxEnsureCapacity((emxArray__common *)b_Fbreathing, i0, (int)sizeof(creal_T));
  ixstart = Fbreathing->size[0] * Fbreathing->size[1];
  emxFree_creal_T(&b_HRentr);
  for (i0 = 0; i0 < ixstart; i0++) {
    b_Fbreathing->data[i0] = Fbreathing->data[i0];
  }

  emxFree_creal_T(&Fbreathing);
  emxInit_creal_T(&RRentr, 2);
  get_spectral_entropy(b_Fbreathing, RRentr);
  i0 = RRentr->size[0] * RRentr->size[1];
  RRentr->size[0] = 1;
  emxEnsureCapacity((emxArray__common *)RRentr, i0, (int)sizeof(creal_T));
  ixstart = RRentr->size[0];
  calclen = RRentr->size[1];
  ixstart *= calclen;
  emxFree_creal_T(&b_Fbreathing);
  for (i0 = 0; i0 < ixstart; i0++) {
    RRentr->data[i0].re = 1.0 - RRentr->data[i0].re;
    RRentr->data[i0].im = 0.0 - RRentr->data[i0].im;
  }

  emxInit_creal_T(&b_RRentr, 2);
  i0 = b_RRentr->size[0] * b_RRentr->size[1];
  b_RRentr->size[0] = 1;
  b_RRentr->size[1] = RRentr->size[1];
  emxEnsureCapacity((emxArray__common *)b_RRentr, i0, (int)sizeof(creal_T));
  ixstart = RRentr->size[0] * RRentr->size[1];
  for (i0 = 0; i0 < ixstart; i0++) {
    b_RRentr->data[i0] = RRentr->data[i0];
  }

  emxInit_real_T1(&b_Bbody, 2);
  R = sum(RRentr);
  b_rdivide(b_RRentr, R, RRentr);

  /*  trust neighborhoods of high weight */
  ixstart = Bbody->size[0];
  calclen = Bbody->size[1];
  i0 = b_Bbody->size[0] * b_Bbody->size[1];
  b_Bbody->size[0] = ixstart;
  b_Bbody->size[1] = calclen;
  emxEnsureCapacity((emxArray__common *)b_Bbody, i0, (int)sizeof(double));
  emxFree_creal_T(&b_RRentr);
  for (i0 = 0; i0 < calclen; i0++) {
    for (k = 0; k < ixstart; k++) {
      b_Bbody->data[k + b_Bbody->size[0] * i0] = Bbody->data[k + Bbody->size[0] *
        i0];
    }
  }

  emxFree_real_T(&Bbody);
  for (i0 = 0; i0 < 2; i0++) {
    sz[i0] = b_Bbody->size[i0];
  }

  emxFree_real_T(&b_Bbody);
  i0 = Y->size[0] * Y->size[1];
  Y->size[0] = sz[0];
  Y->size[1] = sz[1];
  emxEnsureCapacity((emxArray__common *)Y, i0, (int)sizeof(creal_T));
  for (k = 0; k + 1 <= RRentr->size[1]; k++) {
    Y->data[k] = RRentr->data[k];
  }

  emxFree_creal_T(&RRentr);
  i0 = Y->size[0] * Y->size[1];
  emxEnsureCapacity((emxArray__common *)Y, i0, (int)sizeof(creal_T));
  ixstart = Y->size[0];
  calclen = Y->size[1];
  ixstart *= calclen;
  for (i0 = 0; i0 < ixstart; i0++) {
    Y->data[i0].re = 1.0 - Y->data[i0].re;
    Y->data[i0].im = 0.0 - Y->data[i0].im;
  }

  /*  B2 = colfilt(B,[2 2],'sliding',@min); */
  /*  B(1:end-1,1:end-1) = B2(1:end-1,1:end-1); */
  i0 = (int)(((double)Y->size[0] - 1.0) / 2.0);
  for (i = 0; i < i0; i++) {
    nx = (i << 1) + 1;
    k = (int)(((double)Y->size[1] - 1.0) / 2.0);
    for (br = 0; br < k; br++) {
      m = (br << 1) + 1;
      for (calclen = 0; calclen < 3; calclen++) {
        for (ixstart = 0; ixstart < 3; ixstart++) {
          patch[ixstart + 3 * calclen] = Y->data[((ixstart + nx) + Y->size[0] *
            ((calclen + m) - 1)) - 1];
        }
      }

      ixstart = 1;
      c_mtmp = patch[0];
      if (rtIsNaN(patch[0].re) || rtIsNaN(patch[0].im)) {
        ix = 1;
        exitg4 = false;
        while ((!exitg4) && (ix + 1 < 10)) {
          ixstart = ix + 1;
          if (!(rtIsNaN(patch[ix].re) || rtIsNaN(patch[ix].im))) {
            c_mtmp = patch[ix];
            exitg4 = true;
          } else {
            ix++;
          }
        }
      }

      if (ixstart < 9) {
        while (ixstart + 1 < 10) {
          if (relop(patch[ixstart], c_mtmp)) {
            c_mtmp = patch[ixstart];
          }

          ixstart++;
        }
      }

      Y->data[nx + Y->size[0] * m] = c_mtmp;
    }
  }

  emxInit_real_T2(&r2, 1);

  /*  !! */
  ixstart = Y->size[0] * Y->size[1];
  i0 = r2->size[0];
  r2->size[0] = ixstart;
  emxEnsureCapacity((emxArray__common *)r2, i0, (int)sizeof(double));
  for (i0 = 0; i0 < ixstart; i0++) {
    r2->data[i0] = 1.0;
  }

  emxInit_real_T2(&wrr, 1);
  ixstart = Y->size[0] * Y->size[1];
  rdivide(r2, ixstart, wrr);

  /*  Use fft to get rid of phase differences between signals. */
  b_fft(Rbody_interp, Y);
  b_abs(Y, Rbody);
  Rbody->data[0] = 0.0;
  emxFree_real_T(&r2);
  emxFree_creal_T(&Y);
  emxFree_real_T(&Rbody_interp);
  if (idx1_data[0] < 1.0) {
    i0 = heartrate->size[0] * heartrate->size[1];
    heartrate->size[0] = 1;
    heartrate->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)heartrate, i0, (int)sizeof(double));
  } else {
    i0 = heartrate->size[0] * heartrate->size[1];
    heartrate->size[0] = 1;
    heartrate->size[1] = (int)idx1_data[0];
    emxEnsureCapacity((emxArray__common *)heartrate, i0, (int)sizeof(double));
    ixstart = (int)idx1_data[0] - 1;
    for (i0 = 0; i0 <= ixstart; i0++) {
      heartrate->data[heartrate->size[0] * i0] = 1.0 + (double)i0;
    }
  }

  if (Rbody->size[0] < idx2_data[0]) {
    i0 = freq->size[0] * freq->size[1];
    freq->size[0] = 1;
    freq->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)freq, i0, (int)sizeof(double));
  } else {
    i0 = Rbody->size[0];
    k = freq->size[0] * freq->size[1];
    freq->size[0] = 1;
    freq->size[1] = (i0 - idx2_data[0]) + 1;
    emxEnsureCapacity((emxArray__common *)freq, k, (int)sizeof(double));
    ixstart = i0 - idx2_data[0];
    for (i0 = 0; i0 <= ixstart; i0++) {
      freq->data[freq->size[0] * i0] = idx2_data[0] + i0;
    }
  }

  i0 = r0->size[0];
  r0->size[0] = heartrate->size[1] + freq->size[1];
  emxEnsureCapacity((emxArray__common *)r0, i0, (int)sizeof(int));
  ixstart = heartrate->size[1];
  for (i0 = 0; i0 < ixstart; i0++) {
    r0->data[i0] = (int)heartrate->data[heartrate->size[0] * i0];
  }

  ixstart = freq->size[1];
  for (i0 = 0; i0 < ixstart; i0++) {
    r0->data[i0 + heartrate->size[1]] = (int)freq->data[freq->size[0] * i0];
  }

  ixstart = Rbody->size[1];
  calclen = r0->size[0];
  for (i0 = 0; i0 < ixstart; i0++) {
    for (k = 0; k < calclen; k++) {
      Rbody->data[(r0->data[k] + Rbody->size[0] * i0) - 1] = 0.0;
    }
  }

  emxFree_int32_T(&r0);

  /*  breathing = ifft(F * wrr'); */
  emxInit_real_T2(&breathing, 1);
  guard1 = false;
  if (Rbody->size[1] == 1) {
    guard1 = true;
  } else {
    ixstart = wrr->size[0];
    if (ixstart == 1) {
      guard1 = true;
    } else {
      k = Rbody->size[1];
      Rbody_idx_0 = (unsigned int)Rbody->size[0];
      i0 = breathing->size[0];
      breathing->size[0] = (int)Rbody_idx_0;
      emxEnsureCapacity((emxArray__common *)breathing, i0, (int)sizeof(double));
      m = Rbody->size[0];
      calclen = breathing->size[0];
      i0 = breathing->size[0];
      breathing->size[0] = calclen;
      emxEnsureCapacity((emxArray__common *)breathing, i0, (int)sizeof(double));
      for (i0 = 0; i0 < calclen; i0++) {
        breathing->data[i0] = 0.0;
      }

      if (Rbody->size[0] != 0) {
        calclen = 0;
        while ((m > 0) && (calclen <= 0)) {
          for (ix = 1; ix <= m; ix++) {
            breathing->data[ix - 1] = 0.0;
          }

          calclen = m;
        }

        br = 0;
        calclen = 0;
        while ((m > 0) && (calclen <= 0)) {
          calclen = -1;
          i0 = br + k;
          for (ixstart = br; ixstart + 1 <= i0; ixstart++) {
            if (wrr->data[ixstart] != 0.0) {
              nx = calclen;
              for (ix = 0; ix + 1 <= m; ix++) {
                nx++;
                breathing->data[ix] += wrr->data[ixstart] * Rbody->data[nx];
              }
            }

            calclen += m;
          }

          br += k;
          calclen = m;
        }
      }
    }
  }

  if (guard1) {
    i0 = breathing->size[0];
    breathing->size[0] = Rbody->size[0];
    emxEnsureCapacity((emxArray__common *)breathing, i0, (int)sizeof(double));
    ixstart = Rbody->size[0];
    for (i0 = 0; i0 < ixstart; i0++) {
      breathing->data[i0] = 0.0;
      calclen = Rbody->size[1];
      for (k = 0; k < calclen; k++) {
        breathing->data[i0] += Rbody->data[i0 + Rbody->size[0] * k] * wrr->
          data[k];
      }
    }
  }

  emxFree_real_T(&Rbody);
  emxInit_creal_T(&b_Ahead_filt, 2);

  /*  breathing = Abody * wrr'; */
  i0 = b_Ahead_filt->size[0] * b_Ahead_filt->size[1];
  b_Ahead_filt->size[0] = Ahead_filt->size[0];
  b_Ahead_filt->size[1] = Ahead_filt->size[1];
  emxEnsureCapacity((emxArray__common *)b_Ahead_filt, i0, (int)sizeof(creal_T));
  ixstart = Ahead_filt->size[1];
  for (i0 = 0; i0 < ixstart; i0++) {
    calclen = Ahead_filt->size[0];
    for (k = 0; k < calclen; k++) {
      b_Ahead_filt->data[k + b_Ahead_filt->size[0] * i0].re = Ahead_filt->data[k
        + Ahead_filt->size[0] * i0];
      b_Ahead_filt->data[k + b_Ahead_filt->size[0] * i0].im = 0.0;
    }
  }

  emxInit_creal_T1(&b_R, 1);
  i0 = b_R->size[0];
  b_R->size[0] = b_Ahead_filt->size[0];
  emxEnsureCapacity((emxArray__common *)b_R, i0, (int)sizeof(creal_T));
  ixstart = b_Ahead_filt->size[0];
  for (i0 = 0; i0 < ixstart; i0++) {
    cdiff = 0.0;
    absa = 0.0;
    calclen = b_Ahead_filt->size[1];
    for (k = 0; k < calclen; k++) {
      b_mtmp = HRentr->data[HRentr->size[0] * k].re;
      ndbl = -HRentr->data[HRentr->size[0] * k].im;
      absb = b_Ahead_filt->data[i0 + b_Ahead_filt->size[0] * k].re * b_mtmp -
        b_Ahead_filt->data[i0 + b_Ahead_filt->size[0] * k].im * ndbl;
      b_mtmp = b_Ahead_filt->data[i0 + b_Ahead_filt->size[0] * k].re * ndbl +
        b_Ahead_filt->data[i0 + b_Ahead_filt->size[0] * k].im * b_mtmp;
      cdiff += absb;
      absa += b_mtmp;
    }

    b_R->data[i0].re = -cdiff;
    b_R->data[i0].im = -absa;
  }

  emxFree_creal_T(&b_Ahead_filt);
  b_exp(b_R);
  i0 = b_R->size[0];
  emxEnsureCapacity((emxArray__common *)b_R, i0, (int)sizeof(creal_T));
  ixstart = b_R->size[0];
  for (i0 = 0; i0 < ixstart; i0++) {
    b_R->data[i0].re--;
  }

  emxInit_creal_T(&c_R, 2);
  i0 = c_R->size[0] * c_R->size[1];
  c_R->size[0] = 1;
  c_R->size[1] = b_R->size[0];
  emxEnsureCapacity((emxArray__common *)c_R, i0, (int)sizeof(creal_T));
  ixstart = b_R->size[0];
  for (i0 = 0; i0 < ixstart; i0++) {
    c_R->data[c_R->size[0] * i0].re = b_R->data[i0].re;
    c_R->data[c_R->size[0] * i0].im = -b_R->data[i0].im;
  }

  kalmanfilt(c_R, freq);
  i0 = freq->size[0] * freq->size[1];
  freq->size[0] = 1;
  emxEnsureCapacity((emxArray__common *)freq, i0, (int)sizeof(double));
  ixstart = freq->size[0];
  calclen = freq->size[1];
  ixstart *= calclen;
  emxFree_creal_T(&c_R);
  for (i0 = 0; i0 < ixstart; i0++) {
    freq->data[i0]++;
  }

  d_log(freq);
  i0 = heartrate->size[0] * heartrate->size[1];
  heartrate->size[0] = 1;
  heartrate->size[1] = freq->size[1];
  emxEnsureCapacity((emxArray__common *)heartrate, i0, (int)sizeof(double));
  ixstart = freq->size[0] * freq->size[1];
  for (i0 = 0; i0 < ixstart; i0++) {
    heartrate->data[i0] = -freq->data[i0];
  }

  ixstart = 1;
  nx = HRentr->size[1];
  c_mtmp = HRentr->data[0];
  calclen = 0;
  if (HRentr->size[1] > 1) {
    if (rtIsNaN(HRentr->data[0].re) || rtIsNaN(HRentr->data[0].im)) {
      ix = 2;
      exitg3 = false;
      while ((!exitg3) && (ix <= nx)) {
        ixstart = ix;
        if (!(rtIsNaN(HRentr->data[ix - 1].re) || rtIsNaN(HRentr->data[ix - 1].
              im))) {
          c_mtmp = HRentr->data[ix - 1];
          calclen = ix - 1;
          exitg3 = true;
        } else {
          ix++;
        }
      }
    }

    if (ixstart < HRentr->size[1]) {
      while (ixstart + 1 <= nx) {
        R = HRentr->data[ixstart];
        if (b_relop(R, c_mtmp)) {
          c_mtmp = HRentr->data[ixstart];
          calclen = ixstart;
        }

        ixstart++;
      }
    }
  }

  emxFree_creal_T(&HRentr);
  ixstart = Ahead_filt->size[0];
  i0 = wrr->size[0];
  wrr->size[0] = ixstart;
  emxEnsureCapacity((emxArray__common *)wrr, i0, (int)sizeof(double));
  for (i0 = 0; i0 < ixstart; i0++) {
    wrr->data[i0] = -Ahead_filt->data[i0 + Ahead_filt->size[0] * calclen];
  }

  emxFree_real_T(&Ahead_filt);
  c_exp(wrr);
  i0 = wrr->size[0];
  emxEnsureCapacity((emxArray__common *)wrr, i0, (int)sizeof(double));
  ixstart = wrr->size[0];
  for (i0 = 0; i0 < ixstart; i0++) {
    wrr->data[i0]--;
  }

  emxInit_real_T1(&b_wrr, 2);
  i0 = b_wrr->size[0] * b_wrr->size[1];
  b_wrr->size[0] = 1;
  b_wrr->size[1] = wrr->size[0];
  emxEnsureCapacity((emxArray__common *)b_wrr, i0, (int)sizeof(double));
  ixstart = wrr->size[0];
  for (i0 = 0; i0 < ixstart; i0++) {
    b_wrr->data[b_wrr->size[0] * i0] = wrr->data[i0];
  }

  emxFree_real_T(&wrr);
  emxInit_real_T1(&b_freq, 2);
  b_kalmanfilt(b_wrr, freq);
  i0 = b_freq->size[0] * b_freq->size[1];
  b_freq->size[0] = 1;
  b_freq->size[1] = freq->size[1];
  emxEnsureCapacity((emxArray__common *)b_freq, i0, (int)sizeof(double));
  ixstart = freq->size[0] * freq->size[1];
  emxFree_real_T(&b_wrr);
  for (i0 = 0; i0 < ixstart; i0++) {
    b_freq->data[i0] = freq->data[i0] + 1.0;
  }

  emxInit_creal_T1(&c_Fheartrate, 1);
  emxInit_creal_T1(&d_Fheartrate, 1);
  emxInit_real_T1(&c_freq, 2);
  b_log(b_freq, c_freq);

  /*  */
  /*  figure; */
  /*  subplot(1,2,1), */
  /*  [H,W,~] = size(Bhead); */
  /*  imshow(flipud((reshape(whr,[H W]))),[]) */
  /*  title('Pulse weights') */
  /*  subplot(1,2,2) */
  /*  [H,W,~] = size(Bbody); */
  /*  imshow(flipud(reshape(wrr,[H W])),[]); */
  /*  title('Breathing weights') */
  /*  Find vitals */
  plot_power_spectrum(heartrate, fps, freq, c_Fheartrate);
  i0 = d_Fheartrate->size[0];
  d_Fheartrate->size[0] = c_Fheartrate->size[0];
  emxEnsureCapacity((emxArray__common *)d_Fheartrate, i0, (int)sizeof(creal_T));
  ixstart = c_Fheartrate->size[0];
  emxFree_real_T(&c_freq);
  emxFree_real_T(&b_freq);
  emxFree_real_T(&heartrate);
  for (i0 = 0; i0 < ixstart; i0++) {
    d_Fheartrate->data[i0] = c_Fheartrate->data[i0];
  }

  /*  idx1 = find(freq<5/60, 1, 'first'); */
  /*  idx2 = find(freq>25/60, 1, 'first'); */
  /*  Fbreathing([1:idx1, idx2:end],:) = 0; */
  i0 = (int)floor((double)breathing->size[0] / 2.0);
  if (1 > i0) {
    ixstart = 0;
  } else {
    ixstart = i0;
  }

  i0 = b_R->size[0];
  b_R->size[0] = ixstart;
  emxEnsureCapacity((emxArray__common *)b_R, i0, (int)sizeof(creal_T));
  for (i0 = 0; i0 < ixstart; i0++) {
    b_R->data[i0].re = breathing->data[i0];
    b_R->data[i0].im = 0.0;
  }

  emxFree_real_T(&breathing);
  d_Fheartrate->data[0].re = 0.0;
  d_Fheartrate->data[0].im = 0.0;
  ixstart = 1;
  nx = c_Fheartrate->size[0];
  c_mtmp = d_Fheartrate->data[0];
  calclen = 0;
  if (c_Fheartrate->size[0] > 1) {
    if (rtIsNaN(d_Fheartrate->data[0].re) || rtIsNaN(d_Fheartrate->data[0].im))
    {
      ix = 2;
      exitg2 = false;
      while ((!exitg2) && (ix <= nx)) {
        ixstart = ix;
        if (!(rtIsNaN(d_Fheartrate->data[ix - 1].re) || rtIsNaN
              (d_Fheartrate->data[ix - 1].im))) {
          c_mtmp = d_Fheartrate->data[ix - 1];
          calclen = ix - 1;
          exitg2 = true;
        } else {
          ix++;
        }
      }
    }

    if (ixstart < c_Fheartrate->size[0]) {
      while (ixstart + 1 <= nx) {
        R = d_Fheartrate->data[ixstart];
        if (b_relop(R, c_mtmp)) {
          c_mtmp = d_Fheartrate->data[ixstart];
          calclen = ixstart;
        }

        ixstart++;
      }
    }
  }

  emxFree_creal_T(&d_Fheartrate);
  emxFree_creal_T(&c_Fheartrate);
  *hr = 60.0 * freq->data[calclen];
  ixstart = 1;
  nx = b_R->size[0];
  c_mtmp = b_R->data[0];
  calclen = 0;
  if (b_R->size[0] > 1) {
    if (rtIsNaN(b_R->data[0].re) || rtIsNaN(b_R->data[0].im)) {
      ix = 2;
      exitg1 = false;
      while ((!exitg1) && (ix <= nx)) {
        ixstart = ix;
        if (!(rtIsNaN(b_R->data[ix - 1].re) || rtIsNaN(b_R->data[ix - 1].im))) {
          c_mtmp = b_R->data[ix - 1];
          calclen = ix - 1;
          exitg1 = true;
        } else {
          ix++;
        }
      }
    }

    if (ixstart < b_R->size[0]) {
      while (ixstart + 1 <= nx) {
        R = b_R->data[ixstart];
        if (b_relop(R, c_mtmp)) {
          c_mtmp = b_R->data[ixstart];
          calclen = ixstart;
        }

        ixstart++;
      }
    }
  }

  emxFree_creal_T(&b_R);
  *rr = 60.0 * freq->data[calclen];

  /*  t = [0:numel(heartrate_best)-1] ./ 60; */
  /*  figure; */
  /*  subplot(2,2,1), plot(t,heartrate_best), title('Heart rate') */
  /*  hold on, plot(t,heartrate,'--k') */
  /*  xlabel('time (s)') */
  /*  ylabel('blood volume (a.u.)') */
  /*  legend('strongest signal','weighted average') */
  /*  */
  /*  subplot(2,2,2), plot(t,breathing), title('Breathing') */
  /*  xlabel('time (s)') */
  /*  ylabel('breathing amplitude (a.u.)') */
  /*  */
  /*  subplot(2,2,3), plot(60*freq,Fheartrate), title(sprintf('HR=%u bpm',round(hr*60))) */
  /*  [~,maxidx] = max(Fheartrate); */
  /*  hold on, plot(60*freq(maxidx),Fheartrate(maxidx),'or') */
  /*  xlim([0 300]) */
  /*  xlabel('frequency (1/min)') */
  /*  ylabel('spectral power (dB)') */
  /*  */
  /*  subplot(2,2,4), plot(60*freq,Fbreathing), title(sprintf('RR=%u bpm',round(rr*60))) */
  /*  [~,maxidx] = max(Fbreathing); */
  /*  hold on, plot(60*freq(maxidx),Fbreathing(maxidx),'or') */
  /*  xlim([0 300]) */
  /*  xlabel('frequency (1/min)') */
  /*  ylabel('spectral power (dB)') */
  emxFree_real_T(&freq);
}

/*
 * File trailer for extract_vitals_tk1.c
 *
 * [EOF]
 */
