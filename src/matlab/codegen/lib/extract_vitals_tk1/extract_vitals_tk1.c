/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: extract_vitals_tk1.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 27-Mar-2019 00:43:16
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "extract_vitals_tk1.h"
#include "interp1.h"
#include "extract_vitals_tk1_emxutil.h"
#include "relop.h"
#include "rdivide.h"
#include "sum.h"
#include "get_spectral_entropy.h"
#include "fft.h"
#include "log.h"
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
  int calclen;
  int sz_idx_0;
  int n;
  emxArray_real_T *Bbody;
  emxArray_real_T *x;
  int nx;
  emxArray_real_T *Rbody;
  int k;
  double mtmp;
  boolean_T exitg13;
  double b_mtmp;
  emxArray_real_T *freq;
  boolean_T exitg12;
  double ndbl;
  double apnd;
  double cdiff;
  double absa;
  double absb;
  emxArray_real_T *Rbody_interp;
  int br;
  emxArray_int32_T *r0;
  emxArray_real_T *y;
  emxArray_real_T *b_Rbody;
  boolean_T exitg11;
  boolean_T exitg10;
  boolean_T exitg9;
  boolean_T exitg8;
  emxArray_real_T *Rhead_interp;
  emxArray_real_T *c_Rbody;
  boolean_T exitg7;
  boolean_T exitg6;
  emxArray_creal_T *Y;
  emxArray_creal_T *Fbreathing;
  emxArray_creal_T *Fheartrate;
  emxArray_boolean_T *b_x;
  int ii_data[1];
  boolean_T exitg5;
  double idx1_data[1];
  boolean_T exitg4;
  int idx2_data[1];
  emxArray_real_T *b_y;
  boolean_T exitg3;
  emxArray_creal_T *b_Fheartrate;
  emxArray_creal_T *HRentr;
  emxArray_creal_T *whr;
  emxArray_creal_T *b_whr;
  creal_T breathing;
  creal_T temp;
  emxArray_creal_T *b;
  emxArray_creal_T *heartrate;
  int m;
  emxArray_creal_T *b_Fbreathing;
  boolean_T exitg2;
  boolean_T b_b;
  emxArray_creal_T *b_HRentr;
  emxArray_creal_T *b_breathing;
  boolean_T guard1 = false;
  boolean_T exitg1;

  /*  fprintf('%d,%d',size(frames_head, 1), size(frames_head, 2)); */
  /*  fprintf('%d,%d',size(frames_body, 1), size(frames_body, 2)); */
  /*  fprintf('%d,%d',size(timestamps, 1), size(timestamps, 2)); */
  i0 = timestamps->size[0] * timestamps->size[1];
  timestamps->size[0] = 1;
  emxEnsureCapacity((emxArray__common *)timestamps, i0, (int)sizeof(double));
  calclen = timestamps->size[0];
  sz_idx_0 = timestamps->size[1];
  n = calclen * sz_idx_0;
  for (i0 = 0; i0 < n; i0++) {
    timestamps->data[i0] *= 1000.0;
  }

  b_floor(timestamps);
  i0 = timestamps->size[0] * timestamps->size[1];
  timestamps->size[0] = 1;
  emxEnsureCapacity((emxArray__common *)timestamps, i0, (int)sizeof(double));
  calclen = timestamps->size[0];
  sz_idx_0 = timestamps->size[1];
  n = calclen * sz_idx_0;
  for (i0 = 0; i0 < n; i0++) {
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
  sz_idx_0 = timestamps->size[1];
  i0 = Rbody->size[0] * Rbody->size[1];
  Rbody->size[0] = sz_idx_0;
  Rbody->size[1] = calclen;
  emxEnsureCapacity((emxArray__common *)Rbody, i0, (int)sizeof(double));
  for (k = 0; k + 1 <= nx; k++) {
    Rbody->data[k] = x->data[k];
  }

  /*  Interpolate between uneven timestamped measurements */
  sz_idx_0 = 1;
  n = timestamps->size[1];
  mtmp = timestamps->data[0];
  if (timestamps->size[1] > 1) {
    if (rtIsNaN(timestamps->data[0])) {
      nx = 2;
      exitg13 = false;
      while ((!exitg13) && (nx <= n)) {
        sz_idx_0 = nx;
        if (!rtIsNaN(timestamps->data[nx - 1])) {
          mtmp = timestamps->data[nx - 1];
          exitg13 = true;
        } else {
          nx++;
        }
      }
    }

    if (sz_idx_0 < timestamps->size[1]) {
      while (sz_idx_0 + 1 <= n) {
        if (timestamps->data[sz_idx_0] < mtmp) {
          mtmp = timestamps->data[sz_idx_0];
        }

        sz_idx_0++;
      }
    }
  }

  sz_idx_0 = 1;
  n = timestamps->size[1];
  b_mtmp = timestamps->data[0];
  if (timestamps->size[1] > 1) {
    if (rtIsNaN(timestamps->data[0])) {
      nx = 2;
      exitg12 = false;
      while ((!exitg12) && (nx <= n)) {
        sz_idx_0 = nx;
        if (!rtIsNaN(timestamps->data[nx - 1])) {
          b_mtmp = timestamps->data[nx - 1];
          exitg12 = true;
        } else {
          nx++;
        }
      }
    }

    if (sz_idx_0 < timestamps->size[1]) {
      while (sz_idx_0 + 1 <= n) {
        if (timestamps->data[sz_idx_0] > b_mtmp) {
          b_mtmp = timestamps->data[sz_idx_0];
        }

        sz_idx_0++;
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
      n = (int)ndbl;
    } else {
      n = 0;
    }

    i0 = freq->size[0] * freq->size[1];
    freq->size[0] = 1;
    freq->size[1] = n;
    emxEnsureCapacity((emxArray__common *)freq, i0, (int)sizeof(double));
    if (n > 0) {
      freq->data[0] = mtmp;
      if (n > 1) {
        freq->data[n - 1] = apnd;
        calclen = (n - 1) / 2;
        for (k = 1; k < calclen; k++) {
          ndbl = (double)k * 0.033333333333333333;
          freq->data[k] = mtmp + ndbl;
          freq->data[(n - k) - 1] = apnd - ndbl;
        }

        if (calclen << 1 == n - 1) {
          freq->data[calclen] = (mtmp + apnd) / 2.0;
        } else {
          ndbl = (double)calclen * 0.033333333333333333;
          freq->data[calclen] = mtmp + ndbl;
          freq->data[calclen + 1] = apnd - ndbl;
        }
      }
    }
  }

  emxInit_real_T1(&Rbody_interp, 2);
  i0 = Rbody_interp->size[0] * Rbody_interp->size[1];
  Rbody_interp->size[0] = freq->size[1];
  Rbody_interp->size[1] = Rbody->size[1];
  emxEnsureCapacity((emxArray__common *)Rbody_interp, i0, (int)sizeof(double));
  n = freq->size[1] * Rbody->size[1];
  for (i0 = 0; i0 < n; i0++) {
    Rbody_interp->data[i0] = 0.0;
  }

  br = 0;
  emxInit_int32_T(&r0, 1);
  emxInit_real_T1(&y, 2);
  emxInit_real_T1(&b_Rbody, 2);
  while (br <= Rbody->size[1] - 1) {
    n = Rbody_interp->size[0];
    i0 = r0->size[0];
    r0->size[0] = n;
    emxEnsureCapacity((emxArray__common *)r0, i0, (int)sizeof(int));
    for (i0 = 0; i0 < n; i0++) {
      r0->data[i0] = i0;
    }

    sz_idx_0 = 1;
    n = timestamps->size[1];
    mtmp = timestamps->data[0];
    if (timestamps->size[1] > 1) {
      if (rtIsNaN(timestamps->data[0])) {
        nx = 2;
        exitg11 = false;
        while ((!exitg11) && (nx <= n)) {
          sz_idx_0 = nx;
          if (!rtIsNaN(timestamps->data[nx - 1])) {
            mtmp = timestamps->data[nx - 1];
            exitg11 = true;
          } else {
            nx++;
          }
        }
      }

      if (sz_idx_0 < timestamps->size[1]) {
        while (sz_idx_0 + 1 <= n) {
          if (timestamps->data[sz_idx_0] < mtmp) {
            mtmp = timestamps->data[sz_idx_0];
          }

          sz_idx_0++;
        }
      }
    }

    sz_idx_0 = 1;
    n = timestamps->size[1];
    b_mtmp = timestamps->data[0];
    if (timestamps->size[1] > 1) {
      if (rtIsNaN(timestamps->data[0])) {
        nx = 2;
        exitg10 = false;
        while ((!exitg10) && (nx <= n)) {
          sz_idx_0 = nx;
          if (!rtIsNaN(timestamps->data[nx - 1])) {
            b_mtmp = timestamps->data[nx - 1];
            exitg10 = true;
          } else {
            nx++;
          }
        }
      }

      if (sz_idx_0 < timestamps->size[1]) {
        while (sz_idx_0 + 1 <= n) {
          if (timestamps->data[sz_idx_0] > b_mtmp) {
            b_mtmp = timestamps->data[sz_idx_0];
          }

          sz_idx_0++;
        }
      }
    }

    if (rtIsNaN(mtmp) || rtIsNaN(b_mtmp)) {
      i0 = y->size[0] * y->size[1];
      y->size[0] = 1;
      y->size[1] = 1;
      emxEnsureCapacity((emxArray__common *)y, i0, (int)sizeof(double));
      y->data[0] = rtNaN;
    } else if (b_mtmp < mtmp) {
      i0 = y->size[0] * y->size[1];
      y->size[0] = 1;
      y->size[1] = 0;
      emxEnsureCapacity((emxArray__common *)y, i0, (int)sizeof(double));
    } else if ((rtIsInf(mtmp) || rtIsInf(b_mtmp)) && (mtmp == b_mtmp)) {
      i0 = y->size[0] * y->size[1];
      y->size[0] = 1;
      y->size[1] = 1;
      emxEnsureCapacity((emxArray__common *)y, i0, (int)sizeof(double));
      y->data[0] = rtNaN;
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
        n = (int)ndbl;
      } else {
        n = 0;
      }

      i0 = y->size[0] * y->size[1];
      y->size[0] = 1;
      y->size[1] = n;
      emxEnsureCapacity((emxArray__common *)y, i0, (int)sizeof(double));
      if (n > 0) {
        y->data[0] = mtmp;
        if (n > 1) {
          y->data[n - 1] = apnd;
          calclen = (n - 1) / 2;
          for (k = 1; k < calclen; k++) {
            ndbl = (double)k * 0.033333333333333333;
            y->data[k] = mtmp + ndbl;
            y->data[(n - k) - 1] = apnd - ndbl;
          }

          if (calclen << 1 == n - 1) {
            y->data[calclen] = (mtmp + apnd) / 2.0;
          } else {
            ndbl = (double)calclen * 0.033333333333333333;
            y->data[calclen] = mtmp + ndbl;
            y->data[calclen + 1] = apnd - ndbl;
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
        br];
    }

    interp1(timestamps, b_Rbody, y, freq);
    nx = r0->size[0];
    for (i0 = 0; i0 < nx; i0++) {
      Rbody_interp->data[r0->data[i0] + Rbody_interp->size[0] * br] = freq->
        data[i0];
    }

    br++;
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

  b_log(Rbody_interp);
  i0 = Rbody_interp->size[0] * Rbody_interp->size[1];
  emxEnsureCapacity((emxArray__common *)Rbody_interp, i0, (int)sizeof(double));
  calclen = Rbody_interp->size[0];
  sz_idx_0 = Rbody_interp->size[1];
  n = calclen * sz_idx_0;
  for (i0 = 0; i0 < n; i0++) {
    Rbody_interp->data[i0] = -Rbody_interp->data[i0];
  }

  /*  Bhead = permute(frames_head, [2 3 1]);  % make it h x w x t */
  imresize(frames_head, 1.0 / block_size, Bbody);
  permute(Bbody, x);
  nx = x->size[0] * x->size[1] * x->size[2];
  emxFree_real_T(&Bbody);
  if (timestamps->size[1] > 0) {
    calclen = div_s32(nx, timestamps->size[1]);
  } else {
    calclen = 0;
  }

  sz_idx_0 = timestamps->size[1];
  i0 = Rbody->size[0] * Rbody->size[1];
  Rbody->size[0] = sz_idx_0;
  Rbody->size[1] = calclen;
  emxEnsureCapacity((emxArray__common *)Rbody, i0, (int)sizeof(double));
  for (k = 0; k + 1 <= nx; k++) {
    Rbody->data[k] = x->data[k];
  }

  emxFree_real_T(&x);

  /*  Interpolate between uneven timestamped measurements */
  sz_idx_0 = 1;
  n = timestamps->size[1];
  mtmp = timestamps->data[0];
  if (timestamps->size[1] > 1) {
    if (rtIsNaN(timestamps->data[0])) {
      nx = 2;
      exitg9 = false;
      while ((!exitg9) && (nx <= n)) {
        sz_idx_0 = nx;
        if (!rtIsNaN(timestamps->data[nx - 1])) {
          mtmp = timestamps->data[nx - 1];
          exitg9 = true;
        } else {
          nx++;
        }
      }
    }

    if (sz_idx_0 < timestamps->size[1]) {
      while (sz_idx_0 + 1 <= n) {
        if (timestamps->data[sz_idx_0] < mtmp) {
          mtmp = timestamps->data[sz_idx_0];
        }

        sz_idx_0++;
      }
    }
  }

  sz_idx_0 = 1;
  n = timestamps->size[1];
  b_mtmp = timestamps->data[0];
  if (timestamps->size[1] > 1) {
    if (rtIsNaN(timestamps->data[0])) {
      nx = 2;
      exitg8 = false;
      while ((!exitg8) && (nx <= n)) {
        sz_idx_0 = nx;
        if (!rtIsNaN(timestamps->data[nx - 1])) {
          b_mtmp = timestamps->data[nx - 1];
          exitg8 = true;
        } else {
          nx++;
        }
      }
    }

    if (sz_idx_0 < timestamps->size[1]) {
      while (sz_idx_0 + 1 <= n) {
        if (timestamps->data[sz_idx_0] > b_mtmp) {
          b_mtmp = timestamps->data[sz_idx_0];
        }

        sz_idx_0++;
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
      n = (int)ndbl;
    } else {
      n = 0;
    }

    i0 = freq->size[0] * freq->size[1];
    freq->size[0] = 1;
    freq->size[1] = n;
    emxEnsureCapacity((emxArray__common *)freq, i0, (int)sizeof(double));
    if (n > 0) {
      freq->data[0] = mtmp;
      if (n > 1) {
        freq->data[n - 1] = apnd;
        calclen = (n - 1) / 2;
        for (k = 1; k < calclen; k++) {
          ndbl = (double)k * 0.033333333333333333;
          freq->data[k] = mtmp + ndbl;
          freq->data[(n - k) - 1] = apnd - ndbl;
        }

        if (calclen << 1 == n - 1) {
          freq->data[calclen] = (mtmp + apnd) / 2.0;
        } else {
          ndbl = (double)calclen * 0.033333333333333333;
          freq->data[calclen] = mtmp + ndbl;
          freq->data[calclen + 1] = apnd - ndbl;
        }
      }
    }
  }

  emxInit_real_T1(&Rhead_interp, 2);
  i0 = Rhead_interp->size[0] * Rhead_interp->size[1];
  Rhead_interp->size[0] = freq->size[1];
  Rhead_interp->size[1] = Rbody->size[1];
  emxEnsureCapacity((emxArray__common *)Rhead_interp, i0, (int)sizeof(double));
  n = freq->size[1] * Rbody->size[1];
  for (i0 = 0; i0 < n; i0++) {
    Rhead_interp->data[i0] = 0.0;
  }

  br = 0;
  emxInit_real_T1(&c_Rbody, 2);
  while (br <= Rbody->size[1] - 1) {
    n = Rhead_interp->size[0];
    i0 = r0->size[0];
    r0->size[0] = n;
    emxEnsureCapacity((emxArray__common *)r0, i0, (int)sizeof(int));
    for (i0 = 0; i0 < n; i0++) {
      r0->data[i0] = i0;
    }

    sz_idx_0 = 1;
    n = timestamps->size[1];
    mtmp = timestamps->data[0];
    if (timestamps->size[1] > 1) {
      if (rtIsNaN(timestamps->data[0])) {
        nx = 2;
        exitg7 = false;
        while ((!exitg7) && (nx <= n)) {
          sz_idx_0 = nx;
          if (!rtIsNaN(timestamps->data[nx - 1])) {
            mtmp = timestamps->data[nx - 1];
            exitg7 = true;
          } else {
            nx++;
          }
        }
      }

      if (sz_idx_0 < timestamps->size[1]) {
        while (sz_idx_0 + 1 <= n) {
          if (timestamps->data[sz_idx_0] < mtmp) {
            mtmp = timestamps->data[sz_idx_0];
          }

          sz_idx_0++;
        }
      }
    }

    sz_idx_0 = 1;
    n = timestamps->size[1];
    b_mtmp = timestamps->data[0];
    if (timestamps->size[1] > 1) {
      if (rtIsNaN(timestamps->data[0])) {
        nx = 2;
        exitg6 = false;
        while ((!exitg6) && (nx <= n)) {
          sz_idx_0 = nx;
          if (!rtIsNaN(timestamps->data[nx - 1])) {
            b_mtmp = timestamps->data[nx - 1];
            exitg6 = true;
          } else {
            nx++;
          }
        }
      }

      if (sz_idx_0 < timestamps->size[1]) {
        while (sz_idx_0 + 1 <= n) {
          if (timestamps->data[sz_idx_0] > b_mtmp) {
            b_mtmp = timestamps->data[sz_idx_0];
          }

          sz_idx_0++;
        }
      }
    }

    if (rtIsNaN(mtmp) || rtIsNaN(b_mtmp)) {
      i0 = y->size[0] * y->size[1];
      y->size[0] = 1;
      y->size[1] = 1;
      emxEnsureCapacity((emxArray__common *)y, i0, (int)sizeof(double));
      y->data[0] = rtNaN;
    } else if (b_mtmp < mtmp) {
      i0 = y->size[0] * y->size[1];
      y->size[0] = 1;
      y->size[1] = 0;
      emxEnsureCapacity((emxArray__common *)y, i0, (int)sizeof(double));
    } else if ((rtIsInf(mtmp) || rtIsInf(b_mtmp)) && (mtmp == b_mtmp)) {
      i0 = y->size[0] * y->size[1];
      y->size[0] = 1;
      y->size[1] = 1;
      emxEnsureCapacity((emxArray__common *)y, i0, (int)sizeof(double));
      y->data[0] = rtNaN;
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
        n = (int)ndbl;
      } else {
        n = 0;
      }

      i0 = y->size[0] * y->size[1];
      y->size[0] = 1;
      y->size[1] = n;
      emxEnsureCapacity((emxArray__common *)y, i0, (int)sizeof(double));
      if (n > 0) {
        y->data[0] = mtmp;
        if (n > 1) {
          y->data[n - 1] = apnd;
          calclen = (n - 1) / 2;
          for (k = 1; k < calclen; k++) {
            ndbl = (double)k * 0.033333333333333333;
            y->data[k] = mtmp + ndbl;
            y->data[(n - k) - 1] = apnd - ndbl;
          }

          if (calclen << 1 == n - 1) {
            y->data[calclen] = (mtmp + apnd) / 2.0;
          } else {
            ndbl = (double)calclen * 0.033333333333333333;
            y->data[calclen] = mtmp + ndbl;
            y->data[calclen + 1] = apnd - ndbl;
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
        br];
    }

    interp1(timestamps, c_Rbody, y, freq);
    nx = r0->size[0];
    for (i0 = 0; i0 < nx; i0++) {
      Rhead_interp->data[r0->data[i0] + Rhead_interp->size[0] * br] = freq->
        data[i0];
    }

    br++;
  }

  emxFree_real_T(&c_Rbody);
  emxFree_real_T(&Rbody);
  i0 = Rhead_interp->size[0] * Rhead_interp->size[1];
  emxEnsureCapacity((emxArray__common *)Rhead_interp, i0, (int)sizeof(double));
  calclen = Rhead_interp->size[0];
  sz_idx_0 = Rhead_interp->size[1];
  n = calclen * sz_idx_0;
  for (i0 = 0; i0 < n; i0++) {
    Rhead_interp->data[i0]++;
  }

  b_log(Rhead_interp);
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
  /*  low_freq = 40/60; % low_freq = 30/60; */
  /*  high_freq = 100/60; % high_freq = 350/60; */
  /*  x_ts = timeseries(Ahead, [0:T-1]./60); */
  /*  x_filt = idealfilter(x_ts, [low_freq high_freq], 'pass'); */
  /*  Ahead_filt = x_filt.Data; */
  /*  butter() requires constant valued inputs, so put in if statements. */
  /*  Abody = zeros(size(Abody)); */
  /*  for i = 1:size(Abody,2) */
  /*    Abody(:,i) = filter(b, a, Abody(:,i)); */
  /*  end */
  /*  Do weighted average of signals based on (inverse) entropy to yield a */
  /*  signal (heartrate and breathing). */
  /*  power = (a^2+b^2)/N */
  /*  Should be same as 1/N*abs(Y)^2 */
  ndbl = fps / (double)Rbody_interp->size[0];
  b_mtmp = (double)Rbody_interp->size[0] / 2.0;
  calclen = (int)floor(b_mtmp);
  if (calclen - 1 < 0) {
    i0 = freq->size[0] * freq->size[1];
    freq->size[0] = 1;
    freq->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)freq, i0, (int)sizeof(double));
  } else {
    i0 = freq->size[0] * freq->size[1];
    freq->size[0] = 1;
    freq->size[1] = calclen;
    emxEnsureCapacity((emxArray__common *)freq, i0, (int)sizeof(double));
    n = calclen - 1;
    for (i0 = 0; i0 <= n; i0++) {
      freq->data[freq->size[0] * i0] = i0;
    }
  }

  i0 = freq->size[0] * freq->size[1];
  freq->size[0] = 1;
  emxEnsureCapacity((emxArray__common *)freq, i0, (int)sizeof(double));
  calclen = freq->size[0];
  sz_idx_0 = freq->size[1];
  n = calclen * sz_idx_0;
  for (i0 = 0; i0 < n; i0++) {
    freq->data[i0] *= ndbl;
  }

  emxInit_creal_T(&Y, 2);
  fft(Rbody_interp, Rbody_interp->size[0], Y);
  i0 = Y->size[0] * Y->size[1];
  emxEnsureCapacity((emxArray__common *)Y, i0, (int)sizeof(creal_T));
  calclen = Y->size[0];
  sz_idx_0 = Y->size[1];
  nx = Rbody_interp->size[0];
  n = calclen * sz_idx_0;
  for (i0 = 0; i0 < n; i0++) {
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
  emxFree_real_T(&Rbody_interp);
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

  emxInit_boolean_T(&b_x, 2);

  /*  [freq, Fbreathing] = plot_power_spectrum(Abody, 60); */
  /*  [freq, Fheartrate] = plot_power_spectrum(Ahead_filt, 60); */
  i0 = b_x->size[0] * b_x->size[1];
  b_x->size[0] = 1;
  b_x->size[1] = freq->size[1];
  emxEnsureCapacity((emxArray__common *)b_x, i0, (int)sizeof(boolean_T));
  calclen = freq->size[0] * freq->size[1];
  for (i0 = 0; i0 < calclen; i0++) {
    b_x->data[i0] = (freq->data[i0] > 0.033333333333333333);
  }

  k = (1 <= b_x->size[1]);
  calclen = 0;
  sz_idx_0 = 1;
  exitg5 = false;
  while ((!exitg5) && (sz_idx_0 <= b_x->size[1])) {
    if (b_x->data[sz_idx_0 - 1]) {
      calclen = 1;
      ii_data[0] = sz_idx_0;
      exitg5 = true;
    } else {
      sz_idx_0++;
    }
  }

  if (k == 1) {
    if (calclen == 0) {
      k = 0;
    }
  } else {
    k = !(1 > calclen);
  }

  for (i0 = 0; i0 < k; i0++) {
    idx1_data[i0] = (double)ii_data[i0] - 1.0;
  }

  i0 = b_x->size[0] * b_x->size[1];
  b_x->size[0] = 1;
  b_x->size[1] = freq->size[1];
  emxEnsureCapacity((emxArray__common *)b_x, i0, (int)sizeof(boolean_T));
  calclen = freq->size[0] * freq->size[1];
  for (i0 = 0; i0 < calclen; i0++) {
    b_x->data[i0] = (freq->data[i0] > 0.41666666666666669);
  }

  k = (1 <= b_x->size[1]);
  calclen = 0;
  sz_idx_0 = 1;
  exitg4 = false;
  while ((!exitg4) && (sz_idx_0 <= b_x->size[1])) {
    if (b_x->data[sz_idx_0 - 1]) {
      calclen = 1;
      ii_data[0] = sz_idx_0;
      exitg4 = true;
    } else {
      sz_idx_0++;
    }
  }

  if (k == 1) {
    if (calclen == 0) {
      k = 0;
    }
  } else {
    k = !(1 > calclen);
  }

  for (i0 = 0; i0 < k; i0++) {
    idx2_data[i0] = ii_data[i0];
  }

  if (idx1_data[0] < 1.0) {
    i0 = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)y, i0, (int)sizeof(double));
  } else {
    i0 = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = (int)idx1_data[0];
    emxEnsureCapacity((emxArray__common *)y, i0, (int)sizeof(double));
    calclen = (int)idx1_data[0] - 1;
    for (i0 = 0; i0 <= calclen; i0++) {
      y->data[y->size[0] * i0] = 1.0 + (double)i0;
    }
  }

  emxInit_real_T1(&b_y, 2);
  if (Fbreathing->size[0] < idx2_data[0]) {
    i0 = b_y->size[0] * b_y->size[1];
    b_y->size[0] = 1;
    b_y->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)b_y, i0, (int)sizeof(double));
  } else {
    i0 = Fbreathing->size[0];
    sz_idx_0 = b_y->size[0] * b_y->size[1];
    b_y->size[0] = 1;
    b_y->size[1] = (i0 - idx2_data[0]) + 1;
    emxEnsureCapacity((emxArray__common *)b_y, sz_idx_0, (int)sizeof(double));
    calclen = i0 - idx2_data[0];
    for (i0 = 0; i0 <= calclen; i0++) {
      b_y->data[b_y->size[0] * i0] = idx2_data[0] + i0;
    }
  }

  i0 = r0->size[0];
  r0->size[0] = y->size[1] + b_y->size[1];
  emxEnsureCapacity((emxArray__common *)r0, i0, (int)sizeof(int));
  calclen = y->size[1];
  for (i0 = 0; i0 < calclen; i0++) {
    r0->data[i0] = (int)y->data[y->size[0] * i0];
  }

  calclen = b_y->size[1];
  for (i0 = 0; i0 < calclen; i0++) {
    r0->data[i0 + y->size[1]] = (int)b_y->data[b_y->size[0] * i0];
  }

  emxFree_real_T(&b_y);
  calclen = Fbreathing->size[1];
  nx = r0->size[0];
  for (i0 = 0; i0 < calclen; i0++) {
    for (sz_idx_0 = 0; sz_idx_0 < nx; sz_idx_0++) {
      Fbreathing->data[(r0->data[sz_idx_0] + Fbreathing->size[0] * i0) - 1].re =
        0.0;
      Fbreathing->data[(r0->data[sz_idx_0] + Fbreathing->size[0] * i0) - 1].im =
        0.0;
    }
  }

  i0 = b_x->size[0] * b_x->size[1];
  b_x->size[0] = 1;
  b_x->size[1] = freq->size[1];
  emxEnsureCapacity((emxArray__common *)b_x, i0, (int)sizeof(boolean_T));
  calclen = freq->size[0] * freq->size[1];
  for (i0 = 0; i0 < calclen; i0++) {
    b_x->data[i0] = (freq->data[i0] > 1.6666666666666667);
  }

  k = (1 <= b_x->size[1]);
  calclen = 0;
  sz_idx_0 = 1;
  exitg3 = false;
  while ((!exitg3) && (sz_idx_0 <= b_x->size[1])) {
    if (b_x->data[sz_idx_0 - 1]) {
      calclen = 1;
      ii_data[0] = sz_idx_0;
      exitg3 = true;
    } else {
      sz_idx_0++;
    }
  }

  emxFree_boolean_T(&b_x);
  if (k == 1) {
    if (calclen == 0) {
      k = 0;
    }
  } else {
    k = !(1 > calclen);
  }

  for (i0 = 0; i0 < k; i0++) {
    idx2_data[i0] = ii_data[i0];
  }

  /*  Fheartrate(1,:) = 0; */
  if (n < idx2_data[0]) {
    i0 = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)y, i0, (int)sizeof(double));
  } else {
    i0 = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = (n - idx2_data[0]) + 1;
    emxEnsureCapacity((emxArray__common *)y, i0, (int)sizeof(double));
    n -= idx2_data[0];
    for (i0 = 0; i0 <= n; i0++) {
      y->data[y->size[0] * i0] = idx2_data[0] + i0;
    }
  }

  i0 = r0->size[0];
  r0->size[0] = 1 + y->size[1];
  emxEnsureCapacity((emxArray__common *)r0, i0, (int)sizeof(int));
  r0->data[0] = 1;
  n = y->size[1];
  for (i0 = 0; i0 < n; i0++) {
    r0->data[i0 + 1] = (int)y->data[y->size[0] * i0];
  }

  emxFree_real_T(&y);
  n = Y->size[1];
  nx = r0->size[0];
  emxFree_creal_T(&Y);
  for (i0 = 0; i0 < n; i0++) {
    for (sz_idx_0 = 0; sz_idx_0 < nx; sz_idx_0++) {
      Fheartrate->data[(r0->data[sz_idx_0] + Fheartrate->size[0] * i0) - 1].re =
        0.0;
      Fheartrate->data[(r0->data[sz_idx_0] + Fheartrate->size[0] * i0) - 1].im =
        0.0;
    }
  }

  emxFree_int32_T(&r0);
  emxInit_creal_T(&b_Fheartrate, 2);

  /*  possible error */
  i0 = b_Fheartrate->size[0] * b_Fheartrate->size[1];
  b_Fheartrate->size[0] = Fheartrate->size[0];
  b_Fheartrate->size[1] = Fheartrate->size[1];
  emxEnsureCapacity((emxArray__common *)b_Fheartrate, i0, (int)sizeof(creal_T));
  n = Fheartrate->size[0] * Fheartrate->size[1];
  for (i0 = 0; i0 < n; i0++) {
    b_Fheartrate->data[i0] = Fheartrate->data[i0];
  }

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
    HRentr->data[i0].re = (1.0 - HRentr->data[i0].re) - 0.1;
    HRentr->data[i0].im = 0.0 - HRentr->data[i0].im;
  }

  emxInit_creal_T(&whr, 2);
  calclen = HRentr->size[1];
  i0 = whr->size[0] * whr->size[1];
  whr->size[0] = 1;
  whr->size[1] = HRentr->size[1];
  emxEnsureCapacity((emxArray__common *)whr, i0, (int)sizeof(creal_T));
  for (k = 0; k + 1 <= calclen; k++) {
    breathing = HRentr->data[k];
    if (relop(breathing)) {
      temp = HRentr->data[k];
    } else {
      temp.re = 0.0;
      temp.im = 0.0;
    }

    whr->data[k] = temp;
  }

  emxInit_creal_T(&b_whr, 2);
  i0 = b_whr->size[0] * b_whr->size[1];
  b_whr->size[0] = 1;
  b_whr->size[1] = whr->size[1];
  emxEnsureCapacity((emxArray__common *)b_whr, i0, (int)sizeof(creal_T));
  n = whr->size[0] * whr->size[1];
  for (i0 = 0; i0 < n; i0++) {
    b_whr->data[i0] = whr->data[i0];
  }

  emxInit_creal_T1(&b, 1);
  breathing = sum(whr);
  rdivide(b_whr, breathing, whr);
  i0 = b->size[0];
  b->size[0] = whr->size[1];
  emxEnsureCapacity((emxArray__common *)b, i0, (int)sizeof(creal_T));
  n = whr->size[1];
  emxFree_creal_T(&b_whr);
  for (i0 = 0; i0 < n; i0++) {
    b->data[i0].re = whr->data[whr->size[0] * i0].re;
    b->data[i0].im = -whr->data[whr->size[0] * i0].im;
  }

  emxFree_creal_T(&whr);
  emxInit_creal_T1(&heartrate, 1);
  if ((Fheartrate->size[1] == 1) || (b->size[0] == 1)) {
    i0 = heartrate->size[0];
    heartrate->size[0] = Fheartrate->size[0];
    emxEnsureCapacity((emxArray__common *)heartrate, i0, (int)sizeof(creal_T));
    n = Fheartrate->size[0];
    for (i0 = 0; i0 < n; i0++) {
      heartrate->data[i0].re = 0.0;
      heartrate->data[i0].im = 0.0;
      calclen = Fheartrate->size[1];
      for (sz_idx_0 = 0; sz_idx_0 < calclen; sz_idx_0++) {
        ndbl = Fheartrate->data[i0 + Fheartrate->size[0] * sz_idx_0].re *
          b->data[sz_idx_0].re - Fheartrate->data[i0 + Fheartrate->size[0] *
          sz_idx_0].im * b->data[sz_idx_0].im;
        b_mtmp = Fheartrate->data[i0 + Fheartrate->size[0] * sz_idx_0].re *
          b->data[sz_idx_0].im + Fheartrate->data[i0 + Fheartrate->size[0] *
          sz_idx_0].im * b->data[sz_idx_0].re;
        heartrate->data[i0].re += ndbl;
        heartrate->data[i0].im += b_mtmp;
      }
    }
  } else {
    k = Fheartrate->size[1];
    calclen = Fheartrate->size[0];
    i0 = heartrate->size[0];
    heartrate->size[0] = calclen;
    emxEnsureCapacity((emxArray__common *)heartrate, i0, (int)sizeof(creal_T));
    m = Fheartrate->size[0];
    calclen = heartrate->size[0];
    i0 = heartrate->size[0];
    heartrate->size[0] = calclen;
    emxEnsureCapacity((emxArray__common *)heartrate, i0, (int)sizeof(creal_T));
    for (i0 = 0; i0 < calclen; i0++) {
      heartrate->data[i0].re = 0.0;
      heartrate->data[i0].im = 0.0;
    }

    if (Fheartrate->size[0] != 0) {
      calclen = 0;
      while ((m > 0) && (calclen <= 0)) {
        for (n = 1; n <= m; n++) {
          heartrate->data[n - 1].re = 0.0;
          heartrate->data[n - 1].im = 0.0;
        }

        calclen = m;
      }

      br = 0;
      calclen = 0;
      while ((m > 0) && (calclen <= 0)) {
        calclen = 0;
        i0 = br + k;
        for (sz_idx_0 = br; sz_idx_0 + 1 <= i0; sz_idx_0++) {
          b_b = ((b->data[sz_idx_0].re != 0.0) || (b->data[sz_idx_0].im != 0.0));
          if (b_b) {
            temp.re = b->data[sz_idx_0].re - 0.0 * b->data[sz_idx_0].im;
            temp.im = b->data[sz_idx_0].im + 0.0 * b->data[sz_idx_0].re;
            nx = calclen;
            for (n = 0; n + 1 <= m; n++) {
              nx++;
              b_mtmp = temp.re * Fheartrate->data[nx - 1].re - temp.im *
                Fheartrate->data[nx - 1].im;
              ndbl = temp.re * Fheartrate->data[nx - 1].im + temp.im *
                Fheartrate->data[nx - 1].re;
              heartrate->data[n].re += b_mtmp;
              heartrate->data[n].im += ndbl;
            }
          }

          calclen += m;
        }

        br += k;
        calclen = m;
      }
    }
  }

  emxFree_creal_T(&b);
  emxFree_creal_T(&Fheartrate);
  sz_idx_0 = 1;
  n = heartrate->size[0];
  temp = heartrate->data[0];
  calclen = 0;
  if (heartrate->size[0] > 1) {
    if (rtIsNaN(heartrate->data[0].re) || rtIsNaN(heartrate->data[0].im)) {
      nx = 2;
      exitg2 = false;
      while ((!exitg2) && (nx <= n)) {
        sz_idx_0 = nx;
        if (!(rtIsNaN(heartrate->data[nx - 1].re) || rtIsNaN(heartrate->data[nx
              - 1].im))) {
          temp = heartrate->data[nx - 1];
          calclen = nx - 1;
          exitg2 = true;
        } else {
          nx++;
        }
      }
    }

    if (sz_idx_0 < heartrate->size[0]) {
      while (sz_idx_0 + 1 <= n) {
        breathing = heartrate->data[sz_idx_0];
        if (b_relop(breathing, temp)) {
          temp = heartrate->data[sz_idx_0];
          calclen = sz_idx_0;
        }

        sz_idx_0++;
      }
    }
  }

  emxFree_creal_T(&heartrate);
  emxInit_creal_T(&b_Fbreathing, 2);
  *hr = 60.0 * freq->data[calclen];
  i0 = b_Fbreathing->size[0] * b_Fbreathing->size[1];
  b_Fbreathing->size[0] = Fbreathing->size[0];
  b_Fbreathing->size[1] = Fbreathing->size[1];
  emxEnsureCapacity((emxArray__common *)b_Fbreathing, i0, (int)sizeof(creal_T));
  n = Fbreathing->size[0] * Fbreathing->size[1];
  for (i0 = 0; i0 < n; i0++) {
    b_Fbreathing->data[i0] = Fbreathing->data[i0];
  }

  get_spectral_entropy(b_Fbreathing, HRentr);
  i0 = HRentr->size[0] * HRentr->size[1];
  HRentr->size[0] = 1;
  emxEnsureCapacity((emxArray__common *)HRentr, i0, (int)sizeof(creal_T));
  calclen = HRentr->size[0];
  sz_idx_0 = HRentr->size[1];
  n = calclen * sz_idx_0;
  emxFree_creal_T(&b_Fbreathing);
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

  breathing = sum(HRentr);
  rdivide(b_HRentr, breathing, HRentr);
  emxFree_creal_T(&b_HRentr);
  emxInit_creal_T1(&b_breathing, 1);
  guard1 = false;
  if (Fbreathing->size[1] == 1) {
    guard1 = true;
  } else {
    calclen = HRentr->size[1];
    if (calclen == 1) {
      guard1 = true;
    } else {
      k = Fbreathing->size[1];
      calclen = Fbreathing->size[0];
      i0 = b_breathing->size[0];
      b_breathing->size[0] = calclen;
      emxEnsureCapacity((emxArray__common *)b_breathing, i0, (int)sizeof(creal_T));
      m = Fbreathing->size[0];
      calclen = b_breathing->size[0];
      i0 = b_breathing->size[0];
      b_breathing->size[0] = calclen;
      emxEnsureCapacity((emxArray__common *)b_breathing, i0, (int)sizeof(creal_T));
      for (i0 = 0; i0 < calclen; i0++) {
        b_breathing->data[i0].re = 0.0;
        b_breathing->data[i0].im = 0.0;
      }

      if (Fbreathing->size[0] != 0) {
        calclen = 0;
        while ((m > 0) && (calclen <= 0)) {
          for (n = 1; n <= m; n++) {
            b_breathing->data[n - 1].re = 0.0;
            b_breathing->data[n - 1].im = 0.0;
          }

          calclen = m;
        }

        br = 0;
        calclen = 0;
        while ((m > 0) && (calclen <= 0)) {
          calclen = 0;
          i0 = br + k;
          for (sz_idx_0 = br; sz_idx_0 + 1 <= i0; sz_idx_0++) {
            b_b = ((HRentr->data[sz_idx_0].re != 0.0) || (HRentr->data[sz_idx_0]
                    .im != 0.0));
            if (b_b) {
              temp.re = HRentr->data[sz_idx_0].re - 0.0 * HRentr->data[sz_idx_0]
                .im;
              temp.im = HRentr->data[sz_idx_0].im + 0.0 * HRentr->data[sz_idx_0]
                .re;
              nx = calclen;
              for (n = 0; n + 1 <= m; n++) {
                nx++;
                b_mtmp = temp.re * Fbreathing->data[nx - 1].re - temp.im *
                  Fbreathing->data[nx - 1].im;
                ndbl = temp.re * Fbreathing->data[nx - 1].im + temp.im *
                  Fbreathing->data[nx - 1].re;
                b_breathing->data[n].re += b_mtmp;
                b_breathing->data[n].im += ndbl;
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
    i0 = b_breathing->size[0];
    b_breathing->size[0] = Fbreathing->size[0];
    emxEnsureCapacity((emxArray__common *)b_breathing, i0, (int)sizeof(creal_T));
    n = Fbreathing->size[0];
    for (i0 = 0; i0 < n; i0++) {
      b_breathing->data[i0].re = 0.0;
      b_breathing->data[i0].im = 0.0;
      calclen = Fbreathing->size[1];
      for (sz_idx_0 = 0; sz_idx_0 < calclen; sz_idx_0++) {
        b_mtmp = Fbreathing->data[i0 + Fbreathing->size[0] * sz_idx_0].re *
          HRentr->data[sz_idx_0].re - Fbreathing->data[i0 + Fbreathing->size[0] *
          sz_idx_0].im * HRentr->data[sz_idx_0].im;
        ndbl = Fbreathing->data[i0 + Fbreathing->size[0] * sz_idx_0].re *
          HRentr->data[sz_idx_0].im + Fbreathing->data[i0 + Fbreathing->size[0] *
          sz_idx_0].im * HRentr->data[sz_idx_0].re;
        b_breathing->data[i0].re += b_mtmp;
        b_breathing->data[i0].im += ndbl;
      }
    }
  }

  emxFree_creal_T(&Fbreathing);
  emxFree_creal_T(&HRentr);
  sz_idx_0 = 1;
  n = b_breathing->size[0];
  temp = b_breathing->data[0];
  calclen = 0;
  if (b_breathing->size[0] > 1) {
    if (rtIsNaN(b_breathing->data[0].re) || rtIsNaN(b_breathing->data[0].im)) {
      nx = 2;
      exitg1 = false;
      while ((!exitg1) && (nx <= n)) {
        sz_idx_0 = nx;
        if (!(rtIsNaN(b_breathing->data[nx - 1].re) || rtIsNaN(b_breathing->
              data[nx - 1].im))) {
          temp = b_breathing->data[nx - 1];
          calclen = nx - 1;
          exitg1 = true;
        } else {
          nx++;
        }
      }
    }

    if (sz_idx_0 < b_breathing->size[0]) {
      while (sz_idx_0 + 1 <= n) {
        breathing = b_breathing->data[sz_idx_0];
        if (b_relop(breathing, temp)) {
          temp = b_breathing->data[sz_idx_0];
          calclen = sz_idx_0;
        }

        sz_idx_0++;
      }
    }
  }

  emxFree_creal_T(&b_breathing);
  *rr = 60.0 * freq->data[calclen];

  /*  % trust neighborhoods of high weight */
  /*  B = reshape(wrr,size(Bbody(:,:,1))); */
  /*  % B2 = colfilt(B,[2 2],'sliding',@min); */
  /*  % B(1:end-1,1:end-1) = B2(1:end-1,1:end-1); */
  /*  B2 = B; */
  /*  for i = 2:size(B,1)-1 */
  /*    for j = 2:size(B,2)-1 */
  /*      patch = B(i-1:i+1, j-1:j+1); */
  /*      B2(i,j) = min(patch(:)); */
  /*    end */
  /*  end */
  /*  B2([1,2,end-1,end], :) = min(B2(:)); */
  /*  B2(:, [1,2,end-1,end]) = min(B2(:)); */
  /*  wrr = B2(:)./sum(B2(:)); */
  /*  !! */
  /*  wrr = ones(size(wrr))./numel(wrr); */
  /*  sigmam = 1.4e-3; */
  /*  sigmap = 2.3e-5; */
  /*  R = exp(-heartrate)-1; */
  /*  xhat = kalmanfilt(R', sigmap, sigmam); */
  /*  heartrate = -log(xhat+1); */
  /*  [~,maxidx] = max(whr); */
  /*  heartrate_best = Ahead_filt(:,maxidx); */
  /*  R = exp(-heartrate_best)-1; */
  /*  xhat = kalmanfilt(R', sigmap, sigmam); */
  /*  heartrate_best = -log(xhat+1); */
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
  /*  [freq, Fbreathing] = plot_power_spectrum(breathing, fps); */
  /*  [freq, Fheartrate] = plot_power_spectrum(heartrate, fps); */
  /*   */
  /*  % idx1 = find(freq<5/60, 1, 'first'); */
  /*  % idx2 = find(freq>25/60, 1, 'first'); */
  /*  % Fbreathing([1:idx1, idx2:end],:) = 0; */
  /*  Fbreathing = breathing(1:floor(numel(breathing)/2)); */
  /*  Fheartrate(1) = 0; */
  /*   */
  /*  [~,maxidx] = max(Fheartrate); */
  /*  hr = 60*freq(maxidx); */
  /*  [~,maxidx] = max(Fbreathing); */
  /*  rr = 60*freq(maxidx); */
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
