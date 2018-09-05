/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: extract_vitals_tk1.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 05-Sep-2018 11:34:41
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
#include "ifft.h"
#include "abs1.h"
#include "fft.h"
#include "rdivide.h"
#include "sum.h"
#include "get_spectral_entropy.h"
#include "imresize.h"
#include "permute.h"
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
 * Arguments    : emxArray_real_T *frames_head
 *                emxArray_real_T *frames_body
 *                emxArray_real_T *timestamps
 *                double fps
 *                double block_size
 *                double *hr
 *                double *rr
 * Return Type  : void
 */
void extract_vitals_tk1(emxArray_real_T *frames_head, emxArray_real_T
  *frames_body, emxArray_real_T *timestamps, double fps, double block_size,
  double *hr, double *rr)
{
  int i0;
  int calclen;
  int sz_idx_0;
  int n;
  emxArray_real_T *b_frames_body;
  emxArray_real_T *Bbody;
  int nx;
  emxArray_real_T *Rbody;
  double mtmp;
  boolean_T exitg11;
  double kd;
  emxArray_real_T *xhat;
  boolean_T exitg10;
  double ndbl;
  double apnd;
  double cdiff;
  double absa;
  double absb;
  emxArray_real_T *Rbody_interp;
  emxArray_real_T *heartrate;
  emxArray_int32_T *r0;
  emxArray_real_T *b_Rbody;
  boolean_T exitg9;
  emxArray_real_T *b_frames_head;
  boolean_T exitg8;
  boolean_T exitg7;
  boolean_T exitg6;
  emxArray_real_T *Rhead_interp;
  emxArray_real_T *c_Rbody;
  boolean_T exitg5;
  boolean_T exitg4;
  double b[7];
  double a[7];
  int iv0[2];
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
  static const double d_b[7] = { 3.9712366981837905E-5, -0.0,
    -0.00011913710094551373, -0.0, 0.00011913710094551373, -0.0,
    -3.9712366981837905E-5 };

  static const double e_b[7] = { 5.1357508103469185E-6, -0.0,
    -1.5407252431040758E-5, -0.0, 1.5407252431040758E-5, -0.0,
    -5.1357508103469185E-6 };

  static const double d_a[7] = { 1.0, -5.8559376958155, 14.294144350096424,
    -18.616444515106217, 13.643865522768245, -5.335286109282694,
    0.86965845063745406 };

  static const double e_a[7] = { 1.0, -5.92906126698601, 14.648903554781828,
    -19.304906460144259, 14.311912062632029, -5.6594126190914427,
    0.93256472886118125 };

  emxArray_real_T *b_Rbody_interp;
  emxArray_real_T *r2;
  emxArray_creal_T *Y;
  emxArray_creal_T *Fbreathing;
  emxArray_creal_T *Fheartrate;
  emxArray_creal_T *b_Fheartrate;
  emxArray_creal_T *HRentr;
  emxArray_creal_T *b_HRentr;
  emxArray_creal_T *b_Fbreathing;
  creal_T c_Fbreathing;
  emxArray_creal_T *RRentr;
  emxArray_creal_T *b_RRentr;
  emxArray_creal_T *d_Rbody;
  emxArray_creal_T *e_Rbody;
  emxArray_creal_T *breathing;
  emxArray_creal_T *b_Ahead_filt;
  emxArray_creal_T *R;
  emxArray_creal_T *b_R;
  creal_T b_mtmp;
  emxArray_real_T *x;
  boolean_T exitg3;
  emxArray_real_T *b_x;
  emxArray_real_T *c_x;
  emxArray_real_T *b_xhat;
  emxArray_creal_T *d_Fbreathing;
  emxArray_real_T *c_xhat;
  boolean_T exitg2;
  boolean_T exitg1;

  /*  fprintf('%d,%d',size(frames_head, 1), size(frames_head, 2)); */
  /*  fprintf('%d,%d',size(frames_body, 1), size(frames_body, 2)); */
  /*  fprintf('%d,%d',size(timestamps, 1), size(timestamps, 2)); */
  /*  c_interp = 100*30; */
  /*  timestamps = floor(timestamps * 100); % ??? */
  i0 = timestamps->size[0] * timestamps->size[1];
  timestamps->size[0] = 1;
  emxEnsureCapacity((emxArray__common *)timestamps, i0, (int)sizeof(double));
  calclen = timestamps->size[0];
  sz_idx_0 = timestamps->size[1];
  n = calclen * sz_idx_0;
  for (i0 = 0; i0 < n; i0++) {
    timestamps->data[i0] *= 100.0;
  }

  b_floor(timestamps);
  i0 = timestamps->size[0] * timestamps->size[1];
  timestamps->size[0] = 1;
  emxEnsureCapacity((emxArray__common *)timestamps, i0, (int)sizeof(double));
  calclen = timestamps->size[0];
  sz_idx_0 = timestamps->size[1];
  n = calclen * sz_idx_0;
  for (i0 = 0; i0 < n; i0++) {
    timestamps->data[i0] /= 100.0;
  }

  emxInit_real_T(&b_frames_body, 3);

  /*  ??? */
  i0 = b_frames_body->size[0] * b_frames_body->size[1] * b_frames_body->size[2];
  b_frames_body->size[0] = frames_body->size[0];
  b_frames_body->size[1] = frames_body->size[1];
  b_frames_body->size[2] = frames_body->size[2];
  emxEnsureCapacity((emxArray__common *)b_frames_body, i0, (int)sizeof(double));
  n = frames_body->size[0] * frames_body->size[1] * frames_body->size[2];
  for (i0 = 0; i0 < n; i0++) {
    b_frames_body->data[i0] = frames_body->data[i0];
  }

  emxInit_real_T(&Bbody, 3);
  permute(b_frames_body, frames_body);
  imresize(frames_body, 1.0 / block_size, Bbody);
  nx = Bbody->size[0] * Bbody->size[1] * Bbody->size[2];
  emxFree_real_T(&b_frames_body);
  if (Bbody->size[2] > 0) {
    calclen = div_s32(nx, Bbody->size[2]);
  } else {
    calclen = 0;
  }

  emxInit_real_T1(&Rbody, 2);
  sz_idx_0 = Bbody->size[2];
  i0 = Rbody->size[0] * Rbody->size[1];
  Rbody->size[0] = sz_idx_0;
  Rbody->size[1] = calclen;
  emxEnsureCapacity((emxArray__common *)Rbody, i0, (int)sizeof(double));
  for (sz_idx_0 = 0; sz_idx_0 + 1 <= nx; sz_idx_0++) {
    Rbody->data[sz_idx_0] = Bbody->data[sz_idx_0];
  }

  /*  Interpolate between uneven timestamped measurements */
  calclen = 1;
  n = timestamps->size[1];
  mtmp = timestamps->data[0];
  if (timestamps->size[1] > 1) {
    if (rtIsNaN(timestamps->data[0])) {
      sz_idx_0 = 2;
      exitg11 = false;
      while ((!exitg11) && (sz_idx_0 <= n)) {
        calclen = sz_idx_0;
        if (!rtIsNaN(timestamps->data[sz_idx_0 - 1])) {
          mtmp = timestamps->data[sz_idx_0 - 1];
          exitg11 = true;
        } else {
          sz_idx_0++;
        }
      }
    }

    if (calclen < timestamps->size[1]) {
      while (calclen + 1 <= n) {
        if (timestamps->data[calclen] < mtmp) {
          mtmp = timestamps->data[calclen];
        }

        calclen++;
      }
    }
  }

  calclen = 1;
  n = timestamps->size[1];
  kd = timestamps->data[0];
  if (timestamps->size[1] > 1) {
    if (rtIsNaN(timestamps->data[0])) {
      sz_idx_0 = 2;
      exitg10 = false;
      while ((!exitg10) && (sz_idx_0 <= n)) {
        calclen = sz_idx_0;
        if (!rtIsNaN(timestamps->data[sz_idx_0 - 1])) {
          kd = timestamps->data[sz_idx_0 - 1];
          exitg10 = true;
        } else {
          sz_idx_0++;
        }
      }
    }

    if (calclen < timestamps->size[1]) {
      while (calclen + 1 <= n) {
        if (timestamps->data[calclen] > kd) {
          kd = timestamps->data[calclen];
        }

        calclen++;
      }
    }
  }

  emxInit_real_T1(&xhat, 2);
  if (rtIsNaN(mtmp) || rtIsNaN(kd)) {
    i0 = xhat->size[0] * xhat->size[1];
    xhat->size[0] = 1;
    xhat->size[1] = 1;
    emxEnsureCapacity((emxArray__common *)xhat, i0, (int)sizeof(double));
    xhat->data[0] = rtNaN;
  } else if (kd < mtmp) {
    i0 = xhat->size[0] * xhat->size[1];
    xhat->size[0] = 1;
    xhat->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)xhat, i0, (int)sizeof(double));
  } else if ((rtIsInf(mtmp) || rtIsInf(kd)) && (mtmp == kd)) {
    i0 = xhat->size[0] * xhat->size[1];
    xhat->size[0] = 1;
    xhat->size[1] = 1;
    emxEnsureCapacity((emxArray__common *)xhat, i0, (int)sizeof(double));
    xhat->data[0] = rtNaN;
  } else {
    ndbl = floor((kd - mtmp) / 0.033333333333333333 + 0.5);
    apnd = mtmp + ndbl * 0.033333333333333333;
    cdiff = apnd - kd;
    absa = fabs(mtmp);
    absb = fabs(kd);
    if ((absa >= absb) || rtIsNaN(absb)) {
      absb = absa;
    }

    if (fabs(cdiff) < 4.4408920985006262E-16 * absb) {
      ndbl++;
      apnd = kd;
    } else if (cdiff > 0.0) {
      apnd = mtmp + (ndbl - 1.0) * 0.033333333333333333;
    } else {
      ndbl++;
    }

    if (ndbl >= 0.0) {
      n = (int)ndbl;
    } else {
      n = 0;
    }

    i0 = xhat->size[0] * xhat->size[1];
    xhat->size[0] = 1;
    xhat->size[1] = n;
    emxEnsureCapacity((emxArray__common *)xhat, i0, (int)sizeof(double));
    if (n > 0) {
      xhat->data[0] = mtmp;
      if (n > 1) {
        xhat->data[n - 1] = apnd;
        calclen = (n - 1) / 2;
        for (sz_idx_0 = 1; sz_idx_0 < calclen; sz_idx_0++) {
          kd = (double)sz_idx_0 * 0.033333333333333333;
          xhat->data[sz_idx_0] = mtmp + kd;
          xhat->data[(n - sz_idx_0) - 1] = apnd - kd;
        }

        if (calclen << 1 == n - 1) {
          xhat->data[calclen] = (mtmp + apnd) / 2.0;
        } else {
          kd = (double)calclen * 0.033333333333333333;
          xhat->data[calclen] = mtmp + kd;
          xhat->data[calclen + 1] = apnd - kd;
        }
      }
    }
  }

  emxInit_real_T1(&Rbody_interp, 2);
  i0 = Rbody_interp->size[0] * Rbody_interp->size[1];
  Rbody_interp->size[0] = xhat->size[1];
  Rbody_interp->size[1] = Rbody->size[1];
  emxEnsureCapacity((emxArray__common *)Rbody_interp, i0, (int)sizeof(double));
  n = xhat->size[1] * Rbody->size[1];
  for (i0 = 0; i0 < n; i0++) {
    Rbody_interp->data[i0] = 0.0;
  }

  nx = 0;
  emxInit_real_T1(&heartrate, 2);
  emxInit_int32_T(&r0, 1);
  emxInit_real_T1(&b_Rbody, 2);
  while (nx <= Rbody->size[1] - 1) {
    n = Rbody_interp->size[0];
    i0 = r0->size[0];
    r0->size[0] = n;
    emxEnsureCapacity((emxArray__common *)r0, i0, (int)sizeof(int));
    for (i0 = 0; i0 < n; i0++) {
      r0->data[i0] = i0;
    }

    calclen = 1;
    n = timestamps->size[1];
    mtmp = timestamps->data[0];
    if (timestamps->size[1] > 1) {
      if (rtIsNaN(timestamps->data[0])) {
        sz_idx_0 = 2;
        exitg9 = false;
        while ((!exitg9) && (sz_idx_0 <= n)) {
          calclen = sz_idx_0;
          if (!rtIsNaN(timestamps->data[sz_idx_0 - 1])) {
            mtmp = timestamps->data[sz_idx_0 - 1];
            exitg9 = true;
          } else {
            sz_idx_0++;
          }
        }
      }

      if (calclen < timestamps->size[1]) {
        while (calclen + 1 <= n) {
          if (timestamps->data[calclen] < mtmp) {
            mtmp = timestamps->data[calclen];
          }

          calclen++;
        }
      }
    }

    calclen = 1;
    n = timestamps->size[1];
    kd = timestamps->data[0];
    if (timestamps->size[1] > 1) {
      if (rtIsNaN(timestamps->data[0])) {
        sz_idx_0 = 2;
        exitg8 = false;
        while ((!exitg8) && (sz_idx_0 <= n)) {
          calclen = sz_idx_0;
          if (!rtIsNaN(timestamps->data[sz_idx_0 - 1])) {
            kd = timestamps->data[sz_idx_0 - 1];
            exitg8 = true;
          } else {
            sz_idx_0++;
          }
        }
      }

      if (calclen < timestamps->size[1]) {
        while (calclen + 1 <= n) {
          if (timestamps->data[calclen] > kd) {
            kd = timestamps->data[calclen];
          }

          calclen++;
        }
      }
    }

    if (rtIsNaN(mtmp) || rtIsNaN(kd)) {
      i0 = heartrate->size[0] * heartrate->size[1];
      heartrate->size[0] = 1;
      heartrate->size[1] = 1;
      emxEnsureCapacity((emxArray__common *)heartrate, i0, (int)sizeof(double));
      heartrate->data[0] = rtNaN;
    } else if (kd < mtmp) {
      i0 = heartrate->size[0] * heartrate->size[1];
      heartrate->size[0] = 1;
      heartrate->size[1] = 0;
      emxEnsureCapacity((emxArray__common *)heartrate, i0, (int)sizeof(double));
    } else if ((rtIsInf(mtmp) || rtIsInf(kd)) && (mtmp == kd)) {
      i0 = heartrate->size[0] * heartrate->size[1];
      heartrate->size[0] = 1;
      heartrate->size[1] = 1;
      emxEnsureCapacity((emxArray__common *)heartrate, i0, (int)sizeof(double));
      heartrate->data[0] = rtNaN;
    } else {
      ndbl = floor((kd - mtmp) / 0.033333333333333333 + 0.5);
      apnd = mtmp + ndbl * 0.033333333333333333;
      cdiff = apnd - kd;
      absa = fabs(mtmp);
      absb = fabs(kd);
      if ((absa >= absb) || rtIsNaN(absb)) {
        absb = absa;
      }

      if (fabs(cdiff) < 4.4408920985006262E-16 * absb) {
        ndbl++;
        apnd = kd;
      } else if (cdiff > 0.0) {
        apnd = mtmp + (ndbl - 1.0) * 0.033333333333333333;
      } else {
        ndbl++;
      }

      if (ndbl >= 0.0) {
        n = (int)ndbl;
      } else {
        n = 0;
      }

      i0 = heartrate->size[0] * heartrate->size[1];
      heartrate->size[0] = 1;
      heartrate->size[1] = n;
      emxEnsureCapacity((emxArray__common *)heartrate, i0, (int)sizeof(double));
      if (n > 0) {
        heartrate->data[0] = mtmp;
        if (n > 1) {
          heartrate->data[n - 1] = apnd;
          calclen = (n - 1) / 2;
          for (sz_idx_0 = 1; sz_idx_0 < calclen; sz_idx_0++) {
            kd = (double)sz_idx_0 * 0.033333333333333333;
            heartrate->data[sz_idx_0] = mtmp + kd;
            heartrate->data[(n - sz_idx_0) - 1] = apnd - kd;
          }

          if (calclen << 1 == n - 1) {
            heartrate->data[calclen] = (mtmp + apnd) / 2.0;
          } else {
            kd = (double)calclen * 0.033333333333333333;
            heartrate->data[calclen] = mtmp + kd;
            heartrate->data[calclen + 1] = apnd - kd;
          }
        }
      }
    }

    n = Rbody->size[0];
    i0 = b_Rbody->size[0] * b_Rbody->size[1];
    b_Rbody->size[0] = 1;
    b_Rbody->size[1] = n;
    emxEnsureCapacity((emxArray__common *)b_Rbody, i0, (int)sizeof(double));
    for (i0 = 0; i0 < n; i0++) {
      b_Rbody->data[b_Rbody->size[0] * i0] = Rbody->data[i0 + Rbody->size[0] *
        nx];
    }

    interp1(timestamps, b_Rbody, heartrate, xhat);
    calclen = r0->size[0];
    for (i0 = 0; i0 < calclen; i0++) {
      Rbody_interp->data[r0->data[i0] + Rbody_interp->size[0] * nx] = xhat->
        data[i0];
    }

    nx++;
  }

  emxFree_real_T(&b_Rbody);
  i0 = Rbody_interp->size[0] * Rbody_interp->size[1];
  emxEnsureCapacity((emxArray__common *)Rbody_interp, i0, (int)sizeof(double));
  calclen = Rbody_interp->size[0];
  sz_idx_0 = Rbody_interp->size[1];
  n = calclen * sz_idx_0;
  for (i0 = 0; i0 < n; i0++) {
    Rbody_interp->data[i0]++;
  }

  c_log(Rbody_interp);
  i0 = Rbody_interp->size[0] * Rbody_interp->size[1];
  emxEnsureCapacity((emxArray__common *)Rbody_interp, i0, (int)sizeof(double));
  calclen = Rbody_interp->size[0];
  sz_idx_0 = Rbody_interp->size[1];
  n = calclen * sz_idx_0;
  for (i0 = 0; i0 < n; i0++) {
    Rbody_interp->data[i0] = -Rbody_interp->data[i0];
  }

  emxInit_real_T(&b_frames_head, 3);
  i0 = b_frames_head->size[0] * b_frames_head->size[1] * b_frames_head->size[2];
  b_frames_head->size[0] = frames_head->size[0];
  b_frames_head->size[1] = frames_head->size[1];
  b_frames_head->size[2] = frames_head->size[2];
  emxEnsureCapacity((emxArray__common *)b_frames_head, i0, (int)sizeof(double));
  n = frames_head->size[0] * frames_head->size[1] * frames_head->size[2];
  for (i0 = 0; i0 < n; i0++) {
    b_frames_head->data[i0] = frames_head->data[i0];
  }

  permute(b_frames_head, frames_head);
  imresize(frames_head, 1.0 / block_size, Bbody);
  nx = Bbody->size[0] * Bbody->size[1] * Bbody->size[2];
  emxFree_real_T(&b_frames_head);
  if (Bbody->size[2] > 0) {
    calclen = div_s32(nx, Bbody->size[2]);
  } else {
    calclen = 0;
  }

  sz_idx_0 = Bbody->size[2];
  i0 = Rbody->size[0] * Rbody->size[1];
  Rbody->size[0] = sz_idx_0;
  Rbody->size[1] = calclen;
  emxEnsureCapacity((emxArray__common *)Rbody, i0, (int)sizeof(double));
  for (sz_idx_0 = 0; sz_idx_0 + 1 <= nx; sz_idx_0++) {
    Rbody->data[sz_idx_0] = Bbody->data[sz_idx_0];
  }

  emxFree_real_T(&Bbody);

  /*  Interpolate between uneven timestamped measurements */
  calclen = 1;
  n = timestamps->size[1];
  mtmp = timestamps->data[0];
  if (timestamps->size[1] > 1) {
    if (rtIsNaN(timestamps->data[0])) {
      sz_idx_0 = 2;
      exitg7 = false;
      while ((!exitg7) && (sz_idx_0 <= n)) {
        calclen = sz_idx_0;
        if (!rtIsNaN(timestamps->data[sz_idx_0 - 1])) {
          mtmp = timestamps->data[sz_idx_0 - 1];
          exitg7 = true;
        } else {
          sz_idx_0++;
        }
      }
    }

    if (calclen < timestamps->size[1]) {
      while (calclen + 1 <= n) {
        if (timestamps->data[calclen] < mtmp) {
          mtmp = timestamps->data[calclen];
        }

        calclen++;
      }
    }
  }

  calclen = 1;
  n = timestamps->size[1];
  kd = timestamps->data[0];
  if (timestamps->size[1] > 1) {
    if (rtIsNaN(timestamps->data[0])) {
      sz_idx_0 = 2;
      exitg6 = false;
      while ((!exitg6) && (sz_idx_0 <= n)) {
        calclen = sz_idx_0;
        if (!rtIsNaN(timestamps->data[sz_idx_0 - 1])) {
          kd = timestamps->data[sz_idx_0 - 1];
          exitg6 = true;
        } else {
          sz_idx_0++;
        }
      }
    }

    if (calclen < timestamps->size[1]) {
      while (calclen + 1 <= n) {
        if (timestamps->data[calclen] > kd) {
          kd = timestamps->data[calclen];
        }

        calclen++;
      }
    }
  }

  if (rtIsNaN(mtmp) || rtIsNaN(kd)) {
    i0 = xhat->size[0] * xhat->size[1];
    xhat->size[0] = 1;
    xhat->size[1] = 1;
    emxEnsureCapacity((emxArray__common *)xhat, i0, (int)sizeof(double));
    xhat->data[0] = rtNaN;
  } else if (kd < mtmp) {
    i0 = xhat->size[0] * xhat->size[1];
    xhat->size[0] = 1;
    xhat->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)xhat, i0, (int)sizeof(double));
  } else if ((rtIsInf(mtmp) || rtIsInf(kd)) && (mtmp == kd)) {
    i0 = xhat->size[0] * xhat->size[1];
    xhat->size[0] = 1;
    xhat->size[1] = 1;
    emxEnsureCapacity((emxArray__common *)xhat, i0, (int)sizeof(double));
    xhat->data[0] = rtNaN;
  } else {
    ndbl = floor((kd - mtmp) / 0.033333333333333333 + 0.5);
    apnd = mtmp + ndbl * 0.033333333333333333;
    cdiff = apnd - kd;
    absa = fabs(mtmp);
    absb = fabs(kd);
    if ((absa >= absb) || rtIsNaN(absb)) {
      absb = absa;
    }

    if (fabs(cdiff) < 4.4408920985006262E-16 * absb) {
      ndbl++;
      apnd = kd;
    } else if (cdiff > 0.0) {
      apnd = mtmp + (ndbl - 1.0) * 0.033333333333333333;
    } else {
      ndbl++;
    }

    if (ndbl >= 0.0) {
      n = (int)ndbl;
    } else {
      n = 0;
    }

    i0 = xhat->size[0] * xhat->size[1];
    xhat->size[0] = 1;
    xhat->size[1] = n;
    emxEnsureCapacity((emxArray__common *)xhat, i0, (int)sizeof(double));
    if (n > 0) {
      xhat->data[0] = mtmp;
      if (n > 1) {
        xhat->data[n - 1] = apnd;
        calclen = (n - 1) / 2;
        for (sz_idx_0 = 1; sz_idx_0 < calclen; sz_idx_0++) {
          kd = (double)sz_idx_0 * 0.033333333333333333;
          xhat->data[sz_idx_0] = mtmp + kd;
          xhat->data[(n - sz_idx_0) - 1] = apnd - kd;
        }

        if (calclen << 1 == n - 1) {
          xhat->data[calclen] = (mtmp + apnd) / 2.0;
        } else {
          kd = (double)calclen * 0.033333333333333333;
          xhat->data[calclen] = mtmp + kd;
          xhat->data[calclen + 1] = apnd - kd;
        }
      }
    }
  }

  emxInit_real_T1(&Rhead_interp, 2);
  i0 = Rhead_interp->size[0] * Rhead_interp->size[1];
  Rhead_interp->size[0] = xhat->size[1];
  Rhead_interp->size[1] = Rbody->size[1];
  emxEnsureCapacity((emxArray__common *)Rhead_interp, i0, (int)sizeof(double));
  n = xhat->size[1] * Rbody->size[1];
  for (i0 = 0; i0 < n; i0++) {
    Rhead_interp->data[i0] = 0.0;
  }

  nx = 0;
  emxInit_real_T1(&c_Rbody, 2);
  while (nx <= Rbody->size[1] - 1) {
    n = Rhead_interp->size[0];
    i0 = r0->size[0];
    r0->size[0] = n;
    emxEnsureCapacity((emxArray__common *)r0, i0, (int)sizeof(int));
    for (i0 = 0; i0 < n; i0++) {
      r0->data[i0] = i0;
    }

    calclen = 1;
    n = timestamps->size[1];
    mtmp = timestamps->data[0];
    if (timestamps->size[1] > 1) {
      if (rtIsNaN(timestamps->data[0])) {
        sz_idx_0 = 2;
        exitg5 = false;
        while ((!exitg5) && (sz_idx_0 <= n)) {
          calclen = sz_idx_0;
          if (!rtIsNaN(timestamps->data[sz_idx_0 - 1])) {
            mtmp = timestamps->data[sz_idx_0 - 1];
            exitg5 = true;
          } else {
            sz_idx_0++;
          }
        }
      }

      if (calclen < timestamps->size[1]) {
        while (calclen + 1 <= n) {
          if (timestamps->data[calclen] < mtmp) {
            mtmp = timestamps->data[calclen];
          }

          calclen++;
        }
      }
    }

    calclen = 1;
    n = timestamps->size[1];
    kd = timestamps->data[0];
    if (timestamps->size[1] > 1) {
      if (rtIsNaN(timestamps->data[0])) {
        sz_idx_0 = 2;
        exitg4 = false;
        while ((!exitg4) && (sz_idx_0 <= n)) {
          calclen = sz_idx_0;
          if (!rtIsNaN(timestamps->data[sz_idx_0 - 1])) {
            kd = timestamps->data[sz_idx_0 - 1];
            exitg4 = true;
          } else {
            sz_idx_0++;
          }
        }
      }

      if (calclen < timestamps->size[1]) {
        while (calclen + 1 <= n) {
          if (timestamps->data[calclen] > kd) {
            kd = timestamps->data[calclen];
          }

          calclen++;
        }
      }
    }

    if (rtIsNaN(mtmp) || rtIsNaN(kd)) {
      i0 = heartrate->size[0] * heartrate->size[1];
      heartrate->size[0] = 1;
      heartrate->size[1] = 1;
      emxEnsureCapacity((emxArray__common *)heartrate, i0, (int)sizeof(double));
      heartrate->data[0] = rtNaN;
    } else if (kd < mtmp) {
      i0 = heartrate->size[0] * heartrate->size[1];
      heartrate->size[0] = 1;
      heartrate->size[1] = 0;
      emxEnsureCapacity((emxArray__common *)heartrate, i0, (int)sizeof(double));
    } else if ((rtIsInf(mtmp) || rtIsInf(kd)) && (mtmp == kd)) {
      i0 = heartrate->size[0] * heartrate->size[1];
      heartrate->size[0] = 1;
      heartrate->size[1] = 1;
      emxEnsureCapacity((emxArray__common *)heartrate, i0, (int)sizeof(double));
      heartrate->data[0] = rtNaN;
    } else {
      ndbl = floor((kd - mtmp) / 0.033333333333333333 + 0.5);
      apnd = mtmp + ndbl * 0.033333333333333333;
      cdiff = apnd - kd;
      absa = fabs(mtmp);
      absb = fabs(kd);
      if ((absa >= absb) || rtIsNaN(absb)) {
        absb = absa;
      }

      if (fabs(cdiff) < 4.4408920985006262E-16 * absb) {
        ndbl++;
        apnd = kd;
      } else if (cdiff > 0.0) {
        apnd = mtmp + (ndbl - 1.0) * 0.033333333333333333;
      } else {
        ndbl++;
      }

      if (ndbl >= 0.0) {
        n = (int)ndbl;
      } else {
        n = 0;
      }

      i0 = heartrate->size[0] * heartrate->size[1];
      heartrate->size[0] = 1;
      heartrate->size[1] = n;
      emxEnsureCapacity((emxArray__common *)heartrate, i0, (int)sizeof(double));
      if (n > 0) {
        heartrate->data[0] = mtmp;
        if (n > 1) {
          heartrate->data[n - 1] = apnd;
          calclen = (n - 1) / 2;
          for (sz_idx_0 = 1; sz_idx_0 < calclen; sz_idx_0++) {
            kd = (double)sz_idx_0 * 0.033333333333333333;
            heartrate->data[sz_idx_0] = mtmp + kd;
            heartrate->data[(n - sz_idx_0) - 1] = apnd - kd;
          }

          if (calclen << 1 == n - 1) {
            heartrate->data[calclen] = (mtmp + apnd) / 2.0;
          } else {
            kd = (double)calclen * 0.033333333333333333;
            heartrate->data[calclen] = mtmp + kd;
            heartrate->data[calclen + 1] = apnd - kd;
          }
        }
      }
    }

    n = Rbody->size[0];
    i0 = c_Rbody->size[0] * c_Rbody->size[1];
    c_Rbody->size[0] = 1;
    c_Rbody->size[1] = n;
    emxEnsureCapacity((emxArray__common *)c_Rbody, i0, (int)sizeof(double));
    for (i0 = 0; i0 < n; i0++) {
      c_Rbody->data[c_Rbody->size[0] * i0] = Rbody->data[i0 + Rbody->size[0] *
        nx];
    }

    interp1(timestamps, c_Rbody, heartrate, xhat);
    calclen = r0->size[0];
    for (i0 = 0; i0 < calclen; i0++) {
      Rhead_interp->data[r0->data[i0] + Rhead_interp->size[0] * nx] = xhat->
        data[i0];
    }

    nx++;
  }

  emxFree_real_T(&c_Rbody);
  emxFree_int32_T(&r0);
  i0 = Rhead_interp->size[0] * Rhead_interp->size[1];
  emxEnsureCapacity((emxArray__common *)Rhead_interp, i0, (int)sizeof(double));
  calclen = Rhead_interp->size[0];
  sz_idx_0 = Rhead_interp->size[1];
  n = calclen * sz_idx_0;
  for (i0 = 0; i0 < n; i0++) {
    Rhead_interp->data[i0]++;
  }

  c_log(Rhead_interp);
  i0 = Rhead_interp->size[0] * Rhead_interp->size[1];
  emxEnsureCapacity((emxArray__common *)Rhead_interp, i0, (int)sizeof(double));
  calclen = Rhead_interp->size[0];
  sz_idx_0 = Rhead_interp->size[1];
  n = calclen * sz_idx_0;
  for (i0 = 0; i0 < n; i0++) {
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
    iv0[i0] = Rhead_interp->size[i0];
  }

  emxInit_real_T1(&Ahead_filt, 2);
  i0 = Ahead_filt->size[0] * Ahead_filt->size[1];
  Ahead_filt->size[0] = iv0[0];
  Ahead_filt->size[1] = iv0[1];
  emxEnsureCapacity((emxArray__common *)Ahead_filt, i0, (int)sizeof(double));
  n = iv0[0] * iv0[1];
  for (i0 = 0; i0 < n; i0++) {
    Ahead_filt->data[i0] = 0.0;
  }

  nx = 0;
  emxInit_real_T2(&b_Rhead_interp, 1);
  emxInit_real_T2(&r1, 1);
  while (nx <= iv0[1] - 1) {
    n = Rhead_interp->size[0];
    i0 = b_Rhead_interp->size[0];
    b_Rhead_interp->size[0] = n;
    emxEnsureCapacity((emxArray__common *)b_Rhead_interp, i0, (int)sizeof(double));
    for (i0 = 0; i0 < n; i0++) {
      b_Rhead_interp->data[i0] = Rhead_interp->data[i0 + Rhead_interp->size[0] *
        nx];
    }

    filter(b, a, b_Rhead_interp, r1);
    n = r1->size[0];
    for (i0 = 0; i0 < n; i0++) {
      Ahead_filt->data[i0 + Ahead_filt->size[0] * nx] = r1->data[i0];
    }

    nx++;
  }

  emxFree_real_T(&r1);
  emxFree_real_T(&b_Rhead_interp);
  if (fabs(fps - 30.0) < 0.1) {
    for (i0 = 0; i0 < 7; i0++) {
      b[i0] = d_b[i0];
      a[i0] = d_a[i0];
    }
  } else {
    if (fabs(fps - 60.0) < 0.1) {
      for (i0 = 0; i0 < 7; i0++) {
        b[i0] = e_b[i0];
        a[i0] = e_a[i0];
      }
    }
  }

  for (i0 = 0; i0 < 2; i0++) {
    iv0[i0] = Rbody_interp->size[i0];
  }

  i0 = Rbody_interp->size[0] * Rbody_interp->size[1];
  Rbody_interp->size[0] = iv0[0];
  Rbody_interp->size[1] = iv0[1];
  emxEnsureCapacity((emxArray__common *)Rbody_interp, i0, (int)sizeof(double));
  n = iv0[0] * iv0[1];
  for (i0 = 0; i0 < n; i0++) {
    Rbody_interp->data[i0] = 0.0;
  }

  nx = 0;
  emxInit_real_T2(&b_Rbody_interp, 1);
  emxInit_real_T2(&r2, 1);
  while (nx <= iv0[1] - 1) {
    n = Rbody_interp->size[0];
    i0 = b_Rbody_interp->size[0];
    b_Rbody_interp->size[0] = n;
    emxEnsureCapacity((emxArray__common *)b_Rbody_interp, i0, (int)sizeof(double));
    for (i0 = 0; i0 < n; i0++) {
      b_Rbody_interp->data[i0] = Rbody_interp->data[i0 + Rbody_interp->size[0] *
        nx];
    }

    filter(b, a, b_Rbody_interp, r2);
    n = r2->size[0];
    for (i0 = 0; i0 < n; i0++) {
      Rbody_interp->data[i0 + Rbody_interp->size[0] * nx] = r2->data[i0];
    }

    nx++;
  }

  emxFree_real_T(&r2);
  emxFree_real_T(&b_Rbody_interp);
  emxInit_creal_T(&Y, 2);

  /*  Do weighted average of signals based on (inverse) entropy to yield a */
  /*  signal (heartrate and breathing). */
  /*  power = (a^2+b^2)/N */
  /*  Should be same as 1/N*abs(Y)^2 */
  fft(Rbody_interp, Rbody_interp->size[0], Y);
  i0 = Y->size[0] * Y->size[1];
  emxEnsureCapacity((emxArray__common *)Y, i0, (int)sizeof(creal_T));
  calclen = Y->size[0];
  sz_idx_0 = Y->size[1];
  nx = Rbody_interp->size[0];
  n = calclen * sz_idx_0;
  for (i0 = 0; i0 < n; i0++) {
    kd = Y->data[i0].re;
    ndbl = -Y->data[i0].im;
    cdiff = Y->data[i0].re * kd - Y->data[i0].im * ndbl;
    ndbl = Y->data[i0].re * ndbl + Y->data[i0].im * kd;
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
    n = 0;
  } else {
    n = i0;
  }

  emxInit_creal_T(&Fbreathing, 2);
  calclen = Y->size[1];
  i0 = Fbreathing->size[0] * Fbreathing->size[1];
  Fbreathing->size[0] = n;
  Fbreathing->size[1] = calclen;
  emxEnsureCapacity((emxArray__common *)Fbreathing, i0, (int)sizeof(creal_T));
  for (i0 = 0; i0 < calclen; i0++) {
    for (sz_idx_0 = 0; sz_idx_0 < n; sz_idx_0++) {
      Fbreathing->data[sz_idx_0 + Fbreathing->size[0] * i0] = Y->data[sz_idx_0 +
        Y->size[0] * i0];
    }
  }

  fft(Rhead_interp, Rbody_interp->size[0], Y);
  i0 = Y->size[0] * Y->size[1];
  emxEnsureCapacity((emxArray__common *)Y, i0, (int)sizeof(creal_T));
  calclen = Y->size[0];
  sz_idx_0 = Y->size[1];
  nx = Rbody_interp->size[0];
  n = calclen * sz_idx_0;
  emxFree_real_T(&Rhead_interp);
  for (i0 = 0; i0 < n; i0++) {
    kd = Y->data[i0].re;
    ndbl = -Y->data[i0].im;
    cdiff = Y->data[i0].re * kd - Y->data[i0].im * ndbl;
    ndbl = Y->data[i0].re * ndbl + Y->data[i0].im * kd;
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
    n = 0;
  } else {
    n = i0;
  }

  emxInit_creal_T(&Fheartrate, 2);
  calclen = Y->size[1];
  i0 = Fheartrate->size[0] * Fheartrate->size[1];
  Fheartrate->size[0] = n;
  Fheartrate->size[1] = calclen;
  emxEnsureCapacity((emxArray__common *)Fheartrate, i0, (int)sizeof(creal_T));
  for (i0 = 0; i0 < calclen; i0++) {
    for (sz_idx_0 = 0; sz_idx_0 < n; sz_idx_0++) {
      Fheartrate->data[sz_idx_0 + Fheartrate->size[0] * i0] = Y->data[sz_idx_0 +
        Y->size[0] * i0];
    }
  }

  /*  [freq, Fbreathing] = plot_power_spectrum(Abody, 60); */
  /*  [freq, Fheartrate] = plot_power_spectrum(Ahead_filt, 60); */
  n = Fbreathing->size[1];
  for (i0 = 0; i0 < n; i0++) {
    Fbreathing->data[Fbreathing->size[0] * i0].re = 0.0;
    Fbreathing->data[Fbreathing->size[0] * i0].im = 0.0;
  }

  n = Y->size[1];
  for (i0 = 0; i0 < n; i0++) {
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
  n = Fheartrate->size[0] * Fheartrate->size[1];
  for (i0 = 0; i0 < n; i0++) {
    b_Fheartrate->data[i0] = Fheartrate->data[i0];
  }

  emxFree_creal_T(&Fheartrate);
  emxInit_creal_T(&HRentr, 2);
  get_spectral_entropy(b_Fheartrate, HRentr);
  i0 = HRentr->size[0] * HRentr->size[1];
  HRentr->size[0] = 1;
  emxEnsureCapacity((emxArray__common *)HRentr, i0, (int)sizeof(creal_T));
  calclen = HRentr->size[0];
  sz_idx_0 = HRentr->size[1];
  n = calclen * sz_idx_0;
  emxFree_creal_T(&b_Fheartrate);
  for (i0 = 0; i0 < n; i0++) {
    HRentr->data[i0].re = 1.0 - HRentr->data[i0].re;
    HRentr->data[i0].im = 0.0 - HRentr->data[i0].im;
  }

  emxInit_creal_T(&b_HRentr, 2);
  i0 = b_HRentr->size[0] * b_HRentr->size[1];
  b_HRentr->size[0] = 1;
  b_HRentr->size[1] = HRentr->size[1];
  emxEnsureCapacity((emxArray__common *)b_HRentr, i0, (int)sizeof(creal_T));
  n = HRentr->size[0] * HRentr->size[1];
  for (i0 = 0; i0 < n; i0++) {
    b_HRentr->data[i0] = HRentr->data[i0];
  }

  emxInit_creal_T(&b_Fbreathing, 2);
  c_Fbreathing = sum(HRentr);
  rdivide(b_HRentr, c_Fbreathing, HRentr);
  i0 = b_Fbreathing->size[0] * b_Fbreathing->size[1];
  b_Fbreathing->size[0] = Fbreathing->size[0];
  b_Fbreathing->size[1] = Fbreathing->size[1];
  emxEnsureCapacity((emxArray__common *)b_Fbreathing, i0, (int)sizeof(creal_T));
  n = Fbreathing->size[0] * Fbreathing->size[1];
  emxFree_creal_T(&b_HRentr);
  for (i0 = 0; i0 < n; i0++) {
    b_Fbreathing->data[i0] = Fbreathing->data[i0];
  }

  emxFree_creal_T(&Fbreathing);
  emxInit_creal_T(&RRentr, 2);
  get_spectral_entropy(b_Fbreathing, RRentr);
  i0 = RRentr->size[0] * RRentr->size[1];
  RRentr->size[0] = 1;
  emxEnsureCapacity((emxArray__common *)RRentr, i0, (int)sizeof(creal_T));
  calclen = RRentr->size[0];
  sz_idx_0 = RRentr->size[1];
  n = calclen * sz_idx_0;
  emxFree_creal_T(&b_Fbreathing);
  for (i0 = 0; i0 < n; i0++) {
    RRentr->data[i0].re = 1.0 - RRentr->data[i0].re;
    RRentr->data[i0].im = 0.0 - RRentr->data[i0].im;
  }

  emxInit_creal_T(&b_RRentr, 2);
  i0 = b_RRentr->size[0] * b_RRentr->size[1];
  b_RRentr->size[0] = 1;
  b_RRentr->size[1] = RRentr->size[1];
  emxEnsureCapacity((emxArray__common *)b_RRentr, i0, (int)sizeof(creal_T));
  n = RRentr->size[0] * RRentr->size[1];
  for (i0 = 0; i0 < n; i0++) {
    b_RRentr->data[i0] = RRentr->data[i0];
  }

  c_Fbreathing = sum(RRentr);
  rdivide(b_RRentr, c_Fbreathing, RRentr);

  /*  Use fft to get rid of phase differences between signals. */
  b_fft(Rbody_interp, Y);
  b_abs(Y, Rbody);
  n = Rbody->size[1];
  emxFree_creal_T(&b_RRentr);
  emxFree_creal_T(&Y);
  emxFree_real_T(&Rbody_interp);
  for (i0 = 0; i0 < n; i0++) {
    Rbody->data[Rbody->size[0] * i0] = 0.0;
  }

  emxInit_creal_T1(&d_Rbody, 1);
  emxInit_creal_T(&e_Rbody, 2);
  i0 = e_Rbody->size[0] * e_Rbody->size[1];
  e_Rbody->size[0] = Rbody->size[0];
  e_Rbody->size[1] = Rbody->size[1];
  emxEnsureCapacity((emxArray__common *)e_Rbody, i0, (int)sizeof(creal_T));
  n = Rbody->size[1];
  for (i0 = 0; i0 < n; i0++) {
    calclen = Rbody->size[0];
    for (sz_idx_0 = 0; sz_idx_0 < calclen; sz_idx_0++) {
      e_Rbody->data[sz_idx_0 + e_Rbody->size[0] * i0].re = Rbody->data[sz_idx_0
        + Rbody->size[0] * i0];
      e_Rbody->data[sz_idx_0 + e_Rbody->size[0] * i0].im = 0.0;
    }
  }

  emxFree_real_T(&Rbody);
  i0 = d_Rbody->size[0];
  d_Rbody->size[0] = e_Rbody->size[0];
  emxEnsureCapacity((emxArray__common *)d_Rbody, i0, (int)sizeof(creal_T));
  n = e_Rbody->size[0];
  for (i0 = 0; i0 < n; i0++) {
    d_Rbody->data[i0].re = 0.0;
    d_Rbody->data[i0].im = 0.0;
    calclen = e_Rbody->size[1];
    for (sz_idx_0 = 0; sz_idx_0 < calclen; sz_idx_0++) {
      kd = RRentr->data[RRentr->size[0] * sz_idx_0].re;
      ndbl = -RRentr->data[RRentr->size[0] * sz_idx_0].im;
      cdiff = e_Rbody->data[i0 + e_Rbody->size[0] * sz_idx_0].re * kd -
        e_Rbody->data[i0 + e_Rbody->size[0] * sz_idx_0].im * ndbl;
      kd = e_Rbody->data[i0 + e_Rbody->size[0] * sz_idx_0].re * ndbl +
        e_Rbody->data[i0 + e_Rbody->size[0] * sz_idx_0].im * kd;
      d_Rbody->data[i0].re += cdiff;
      d_Rbody->data[i0].im += kd;
    }
  }

  emxFree_creal_T(&e_Rbody);
  emxFree_creal_T(&RRentr);
  emxInit_creal_T1(&breathing, 1);
  emxInit_creal_T(&b_Ahead_filt, 2);
  ifft(d_Rbody, breathing);
  i0 = b_Ahead_filt->size[0] * b_Ahead_filt->size[1];
  b_Ahead_filt->size[0] = Ahead_filt->size[0];
  b_Ahead_filt->size[1] = Ahead_filt->size[1];
  emxEnsureCapacity((emxArray__common *)b_Ahead_filt, i0, (int)sizeof(creal_T));
  n = Ahead_filt->size[1];
  emxFree_creal_T(&d_Rbody);
  for (i0 = 0; i0 < n; i0++) {
    calclen = Ahead_filt->size[0];
    for (sz_idx_0 = 0; sz_idx_0 < calclen; sz_idx_0++) {
      b_Ahead_filt->data[sz_idx_0 + b_Ahead_filt->size[0] * i0].re =
        Ahead_filt->data[sz_idx_0 + Ahead_filt->size[0] * i0];
      b_Ahead_filt->data[sz_idx_0 + b_Ahead_filt->size[0] * i0].im = 0.0;
    }
  }

  emxInit_creal_T1(&R, 1);
  i0 = R->size[0];
  R->size[0] = b_Ahead_filt->size[0];
  emxEnsureCapacity((emxArray__common *)R, i0, (int)sizeof(creal_T));
  n = b_Ahead_filt->size[0];
  for (i0 = 0; i0 < n; i0++) {
    cdiff = 0.0;
    absa = 0.0;
    calclen = b_Ahead_filt->size[1];
    for (sz_idx_0 = 0; sz_idx_0 < calclen; sz_idx_0++) {
      kd = HRentr->data[HRentr->size[0] * sz_idx_0].re;
      ndbl = -HRentr->data[HRentr->size[0] * sz_idx_0].im;
      absb = b_Ahead_filt->data[i0 + b_Ahead_filt->size[0] * sz_idx_0].re * kd -
        b_Ahead_filt->data[i0 + b_Ahead_filt->size[0] * sz_idx_0].im * ndbl;
      kd = b_Ahead_filt->data[i0 + b_Ahead_filt->size[0] * sz_idx_0].re * ndbl +
        b_Ahead_filt->data[i0 + b_Ahead_filt->size[0] * sz_idx_0].im * kd;
      cdiff += absb;
      absa += kd;
    }

    R->data[i0].re = -cdiff;
    R->data[i0].im = -absa;
  }

  emxFree_creal_T(&b_Ahead_filt);
  b_exp(R);
  i0 = R->size[0];
  emxEnsureCapacity((emxArray__common *)R, i0, (int)sizeof(creal_T));
  n = R->size[0];
  for (i0 = 0; i0 < n; i0++) {
    R->data[i0].re--;
  }

  emxInit_creal_T(&b_R, 2);
  i0 = b_R->size[0] * b_R->size[1];
  b_R->size[0] = 1;
  b_R->size[1] = R->size[0];
  emxEnsureCapacity((emxArray__common *)b_R, i0, (int)sizeof(creal_T));
  n = R->size[0];
  for (i0 = 0; i0 < n; i0++) {
    b_R->data[b_R->size[0] * i0].re = R->data[i0].re;
    b_R->data[b_R->size[0] * i0].im = -R->data[i0].im;
  }

  kalmanfilt(b_R, xhat);
  i0 = xhat->size[0] * xhat->size[1];
  xhat->size[0] = 1;
  emxEnsureCapacity((emxArray__common *)xhat, i0, (int)sizeof(double));
  calclen = xhat->size[0];
  sz_idx_0 = xhat->size[1];
  n = calclen * sz_idx_0;
  emxFree_creal_T(&b_R);
  for (i0 = 0; i0 < n; i0++) {
    xhat->data[i0]++;
  }

  d_log(xhat);
  i0 = heartrate->size[0] * heartrate->size[1];
  heartrate->size[0] = 1;
  heartrate->size[1] = xhat->size[1];
  emxEnsureCapacity((emxArray__common *)heartrate, i0, (int)sizeof(double));
  n = xhat->size[0] * xhat->size[1];
  for (i0 = 0; i0 < n; i0++) {
    heartrate->data[i0] = -xhat->data[i0];
  }

  calclen = 1;
  n = HRentr->size[1];
  b_mtmp = HRentr->data[0];
  nx = 0;
  if (HRentr->size[1] > 1) {
    if (rtIsNaN(HRentr->data[0].re) || rtIsNaN(HRentr->data[0].im)) {
      sz_idx_0 = 2;
      exitg3 = false;
      while ((!exitg3) && (sz_idx_0 <= n)) {
        calclen = sz_idx_0;
        if (!(rtIsNaN(HRentr->data[sz_idx_0 - 1].re) || rtIsNaN(HRentr->
              data[sz_idx_0 - 1].im))) {
          b_mtmp = HRentr->data[sz_idx_0 - 1];
          nx = sz_idx_0 - 1;
          exitg3 = true;
        } else {
          sz_idx_0++;
        }
      }
    }

    if (calclen < HRentr->size[1]) {
      while (calclen + 1 <= n) {
        c_Fbreathing = HRentr->data[calclen];
        if (relop(c_Fbreathing, b_mtmp)) {
          b_mtmp = HRentr->data[calclen];
          nx = calclen;
        }

        calclen++;
      }
    }
  }

  emxFree_creal_T(&HRentr);
  emxInit_real_T2(&x, 1);
  n = Ahead_filt->size[0];
  i0 = x->size[0];
  x->size[0] = n;
  emxEnsureCapacity((emxArray__common *)x, i0, (int)sizeof(double));
  for (i0 = 0; i0 < n; i0++) {
    x->data[i0] = -Ahead_filt->data[i0 + Ahead_filt->size[0] * nx];
  }

  emxFree_real_T(&Ahead_filt);
  emxInit_real_T2(&b_x, 1);
  i0 = b_x->size[0];
  b_x->size[0] = x->size[0];
  emxEnsureCapacity((emxArray__common *)b_x, i0, (int)sizeof(double));
  n = x->size[0];
  for (i0 = 0; i0 < n; i0++) {
    b_x->data[i0] = x->data[i0];
  }

  for (sz_idx_0 = 0; sz_idx_0 + 1 <= x->size[0]; sz_idx_0++) {
    b_x->data[sz_idx_0] = exp(b_x->data[sz_idx_0]);
  }

  emxFree_real_T(&x);
  emxInit_real_T1(&c_x, 2);
  i0 = c_x->size[0] * c_x->size[1];
  c_x->size[0] = 1;
  c_x->size[1] = b_x->size[0];
  emxEnsureCapacity((emxArray__common *)c_x, i0, (int)sizeof(double));
  n = b_x->size[0];
  for (i0 = 0; i0 < n; i0++) {
    c_x->data[c_x->size[0] * i0] = b_x->data[i0] - 1.0;
  }

  emxFree_real_T(&b_x);
  emxInit_real_T1(&b_xhat, 2);
  b_kalmanfilt(c_x, xhat);
  i0 = b_xhat->size[0] * b_xhat->size[1];
  b_xhat->size[0] = 1;
  b_xhat->size[1] = xhat->size[1];
  emxEnsureCapacity((emxArray__common *)b_xhat, i0, (int)sizeof(double));
  n = xhat->size[0] * xhat->size[1];
  emxFree_real_T(&c_x);
  for (i0 = 0; i0 < n; i0++) {
    b_xhat->data[i0] = xhat->data[i0] + 1.0;
  }

  emxInit_creal_T1(&d_Fbreathing, 1);
  emxInit_real_T1(&c_xhat, 2);
  b_log(b_xhat, c_xhat);

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
  plot_power_spectrum(breathing, fps, xhat, d_Fbreathing);
  b_plot_power_spectrum(heartrate, fps, xhat, R);
  d_Fbreathing->data[0].re = 0.0;
  d_Fbreathing->data[0].im = 0.0;
  R->data[0].re = 0.0;
  R->data[0].im = 0.0;
  calclen = 1;
  n = R->size[0];
  b_mtmp = R->data[0];
  nx = 0;
  emxFree_real_T(&c_xhat);
  emxFree_real_T(&b_xhat);
  emxFree_real_T(&heartrate);
  emxFree_creal_T(&breathing);
  if (R->size[0] > 1) {
    if (rtIsNaN(R->data[0].re) || rtIsNaN(R->data[0].im)) {
      sz_idx_0 = 2;
      exitg2 = false;
      while ((!exitg2) && (sz_idx_0 <= n)) {
        calclen = sz_idx_0;
        if (!(rtIsNaN(R->data[sz_idx_0 - 1].re) || rtIsNaN(R->data[sz_idx_0 - 1]
              .im))) {
          b_mtmp = R->data[sz_idx_0 - 1];
          nx = sz_idx_0 - 1;
          exitg2 = true;
        } else {
          sz_idx_0++;
        }
      }
    }

    if (calclen < R->size[0]) {
      while (calclen + 1 <= n) {
        c_Fbreathing = R->data[calclen];
        if (relop(c_Fbreathing, b_mtmp)) {
          b_mtmp = R->data[calclen];
          nx = calclen;
        }

        calclen++;
      }
    }
  }

  emxFree_creal_T(&R);
  *hr = 60.0 * xhat->data[nx];
  calclen = 1;
  n = d_Fbreathing->size[0];
  b_mtmp = d_Fbreathing->data[0];
  nx = 0;
  if (d_Fbreathing->size[0] > 1) {
    if (rtIsNaN(d_Fbreathing->data[0].re) || rtIsNaN(d_Fbreathing->data[0].im))
    {
      sz_idx_0 = 2;
      exitg1 = false;
      while ((!exitg1) && (sz_idx_0 <= n)) {
        calclen = sz_idx_0;
        if (!(rtIsNaN(d_Fbreathing->data[sz_idx_0 - 1].re) || rtIsNaN
              (d_Fbreathing->data[sz_idx_0 - 1].im))) {
          b_mtmp = d_Fbreathing->data[sz_idx_0 - 1];
          nx = sz_idx_0 - 1;
          exitg1 = true;
        } else {
          sz_idx_0++;
        }
      }
    }

    if (calclen < d_Fbreathing->size[0]) {
      while (calclen + 1 <= n) {
        c_Fbreathing = d_Fbreathing->data[calclen];
        if (relop(c_Fbreathing, b_mtmp)) {
          b_mtmp = d_Fbreathing->data[calclen];
          nx = calclen;
        }

        calclen++;
      }
    }
  }

  emxFree_creal_T(&d_Fbreathing);
  *rr = 60.0 * xhat->data[nx];

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
  emxFree_real_T(&xhat);
}

/*
 * File trailer for extract_vitals_tk1.c
 *
 * [EOF]
 */
