/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: extract_vitals_tk1.c
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 08-Aug-2019 11:00:09
 */

/* Include Files */
#include <math.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "extract_vitals_tk1.h"
#include "interp1.h"
#include "extract_vitals_tk1_emxutil.h"
#include "rdivide.h"
#include "sum.h"
#include "get_spectral_entropy.h"
#include "fft.h"
#include "log.h"
#include "permute.h"
#include "floor.h"
#include "extract_vitals_tk1_rtwutil.h"

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
  int loop_ub;
  int i0;
  emxArray_real_T *x;
  int varargin_1;
  int calclen;
  int idx;
  double Ma;
  int k;
  boolean_T exitg1;
  double kd;
  emxArray_real_T *freq;
  double ndbl;
  double apnd;
  double cdiff;
  emxArray_real_T *Rbody_interp;
  double absa;
  double absb;
  int i;
  emxArray_int32_T *r0;
  emxArray_real_T *y;
  emxArray_real_T *b_x;
  int nm1d2;
  emxArray_real_T *Rhead_interp;
  emxArray_creal_T *Y;
  emxArray_creal_T *Fbreathing;
  emxArray_creal_T *Fheartrate;
  emxArray_boolean_T *c_x;
  int ii_data[1];
  double idx1_data[1];
  int idx2_data[1];
  emxArray_real_T *b_y;
  emxArray_creal_T *HRentr;
  emxArray_creal_T *whr;
  emxArray_creal_T *b_whr;
  double ex_im;
  boolean_T breathing;
  emxArray_creal_T *b_breathing;
  creal_T dc0;
  emxArray_creal_T *heartrate;
  boolean_T SCALEB;
  (void)block_size;

  /*  fprintf('%d,%d',size(frames_head, 1), size(frames_head, 2)); */
  /*  fprintf('%d,%d',size(frames_body, 1), size(frames_body, 2)); */
  /*  fprintf('%d,%d',size(timestamps, 1), size(timestamps, 2)); */
  loop_ub = timestamps->size[0] * timestamps->size[1] - 1;
  i0 = timestamps->size[0] * timestamps->size[1];
  timestamps->size[0] = 1;
  emxEnsureCapacity_real_T(timestamps, i0);
  for (i0 = 0; i0 <= loop_ub; i0++) {
    timestamps->data[i0] *= 1000.0;
  }

  b_floor(timestamps);
  loop_ub = timestamps->size[0] * timestamps->size[1] - 1;
  i0 = timestamps->size[0] * timestamps->size[1];
  timestamps->size[0] = 1;
  emxEnsureCapacity_real_T(timestamps, i0);
  for (i0 = 0; i0 <= loop_ub; i0++) {
    timestamps->data[i0] /= 1000.0;
  }

  emxInit_real_T1(&x, 3);

  /*  ??? */
  /*  Bbody = permute(frames_body, [2 3 1]);  % make it h x w x t */
  /*  Bbody = imresize(Bbody, 1/block_size, 'box'); */
  permute(frames_body, x);
  varargin_1 = timestamps->size[1];
  if (timestamps->size[1] > 0) {
    calclen = div_s32(x->size[0] * x->size[1] * x->size[2], timestamps->size[1]);
  } else {
    calclen = 0;
  }

  /*  Interpolate between uneven timestamped measurements */
  if (timestamps->size[1] <= 2) {
    if (timestamps->size[1] == 1) {
      Ma = timestamps->data[0];
    } else if ((timestamps->data[0] > timestamps->data[1]) || (rtIsNaN
                (timestamps->data[0]) && (!rtIsNaN(timestamps->data[1])))) {
      Ma = timestamps->data[1];
    } else {
      Ma = timestamps->data[0];
    }
  } else {
    if (!rtIsNaN(timestamps->data[0])) {
      idx = 1;
    } else {
      idx = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= timestamps->size[1])) {
        if (!rtIsNaN(timestamps->data[k - 1])) {
          idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }

    if (idx == 0) {
      Ma = timestamps->data[0];
    } else {
      Ma = timestamps->data[idx - 1];
      while (idx + 1 <= timestamps->size[1]) {
        if (Ma > timestamps->data[idx]) {
          Ma = timestamps->data[idx];
        }

        idx++;
      }
    }
  }

  if (timestamps->size[1] <= 2) {
    if (timestamps->size[1] == 1) {
      kd = timestamps->data[0];
    } else if ((timestamps->data[0] < timestamps->data[1]) || (rtIsNaN
                (timestamps->data[0]) && (!rtIsNaN(timestamps->data[1])))) {
      kd = timestamps->data[1];
    } else {
      kd = timestamps->data[0];
    }
  } else {
    if (!rtIsNaN(timestamps->data[0])) {
      idx = 1;
    } else {
      idx = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= timestamps->size[1])) {
        if (!rtIsNaN(timestamps->data[k - 1])) {
          idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }

    if (idx == 0) {
      kd = timestamps->data[0];
    } else {
      kd = timestamps->data[idx - 1];
      while (idx + 1 <= timestamps->size[1]) {
        if (kd < timestamps->data[idx]) {
          kd = timestamps->data[idx];
        }

        idx++;
      }
    }
  }

  emxInit_real_T(&freq, 2);
  if (rtIsNaN(Ma) || rtIsNaN(kd)) {
    i0 = freq->size[0] * freq->size[1];
    freq->size[1] = 1;
    emxEnsureCapacity_real_T(freq, i0);
  } else if (kd < Ma) {
    i0 = freq->size[0] * freq->size[1];
    freq->size[1] = 0;
    emxEnsureCapacity_real_T(freq, i0);
  } else if ((rtIsInf(Ma) || rtIsInf(kd)) && (Ma == kd)) {
    i0 = freq->size[0] * freq->size[1];
    freq->size[1] = 1;
    emxEnsureCapacity_real_T(freq, i0);
  } else {
    ndbl = floor((kd - Ma) / 0.033333333333333333 + 0.5);
    apnd = Ma + ndbl * 0.033333333333333333;
    cdiff = apnd - kd;
    absa = fabs(Ma);
    absb = fabs(kd);
    if ((absa > absb) || rtIsNaN(absb)) {
      absb = absa;
    }

    if (fabs(cdiff) < 4.4408920985006262E-16 * absb) {
      ndbl++;
      apnd = kd;
    } else if (cdiff > 0.0) {
      apnd = Ma + (ndbl - 1.0) * 0.033333333333333333;
    } else {
      ndbl++;
    }

    if (ndbl >= 0.0) {
      idx = (int)ndbl;
    } else {
      idx = 0;
    }

    i0 = freq->size[0] * freq->size[1];
    freq->size[0] = 1;
    freq->size[1] = idx;
    emxEnsureCapacity_real_T(freq, i0);
    if (idx > 0) {
      freq->data[0] = Ma;
      if (idx > 1) {
        freq->data[idx - 1] = apnd;
        nm1d2 = (idx - 1) / 2;
        for (k = 1; k < nm1d2; k++) {
          kd = (double)k * 0.033333333333333333;
          freq->data[k] = Ma + kd;
          freq->data[(idx - k) - 1] = apnd - kd;
        }

        if (nm1d2 << 1 == idx - 1) {
          freq->data[nm1d2] = (Ma + apnd) / 2.0;
        } else {
          kd = (double)nm1d2 * 0.033333333333333333;
          freq->data[nm1d2] = Ma + kd;
          freq->data[nm1d2 + 1] = apnd - kd;
        }
      }
    }
  }

  emxInit_real_T(&Rbody_interp, 2);
  i0 = Rbody_interp->size[0] * Rbody_interp->size[1];
  Rbody_interp->size[0] = freq->size[1];
  emxEnsureCapacity_real_T(Rbody_interp, i0);
  i0 = Rbody_interp->size[0] * Rbody_interp->size[1];
  Rbody_interp->size[1] = calclen;
  emxEnsureCapacity_real_T(Rbody_interp, i0);
  loop_ub = freq->size[1] * calclen;
  for (i0 = 0; i0 < loop_ub; i0++) {
    Rbody_interp->data[i0] = 0.0;
  }

  i = 0;
  emxInit_int32_T(&r0, 1);
  emxInit_real_T(&y, 2);
  emxInit_real_T(&b_x, 2);
  while (i <= calclen - 1) {
    loop_ub = Rbody_interp->size[0];
    i0 = r0->size[0];
    r0->size[0] = loop_ub;
    emxEnsureCapacity_int32_T(r0, i0);
    for (i0 = 0; i0 < loop_ub; i0++) {
      r0->data[i0] = i0;
    }

    if (timestamps->size[1] <= 2) {
      if (timestamps->size[1] == 1) {
        Ma = timestamps->data[0];
      } else if ((timestamps->data[0] > timestamps->data[1]) || (rtIsNaN
                  (timestamps->data[0]) && (!rtIsNaN(timestamps->data[1])))) {
        Ma = timestamps->data[1];
      } else {
        Ma = timestamps->data[0];
      }
    } else {
      if (!rtIsNaN(timestamps->data[0])) {
        idx = 1;
      } else {
        idx = 0;
        k = 2;
        exitg1 = false;
        while ((!exitg1) && (k <= timestamps->size[1])) {
          if (!rtIsNaN(timestamps->data[k - 1])) {
            idx = k;
            exitg1 = true;
          } else {
            k++;
          }
        }
      }

      if (idx == 0) {
        Ma = timestamps->data[0];
      } else {
        Ma = timestamps->data[idx - 1];
        while (idx + 1 <= timestamps->size[1]) {
          if (Ma > timestamps->data[idx]) {
            Ma = timestamps->data[idx];
          }

          idx++;
        }
      }
    }

    if (timestamps->size[1] <= 2) {
      if (timestamps->size[1] == 1) {
        kd = timestamps->data[0];
      } else if ((timestamps->data[0] < timestamps->data[1]) || (rtIsNaN
                  (timestamps->data[0]) && (!rtIsNaN(timestamps->data[1])))) {
        kd = timestamps->data[1];
      } else {
        kd = timestamps->data[0];
      }
    } else {
      if (!rtIsNaN(timestamps->data[0])) {
        idx = 1;
      } else {
        idx = 0;
        k = 2;
        exitg1 = false;
        while ((!exitg1) && (k <= timestamps->size[1])) {
          if (!rtIsNaN(timestamps->data[k - 1])) {
            idx = k;
            exitg1 = true;
          } else {
            k++;
          }
        }
      }

      if (idx == 0) {
        kd = timestamps->data[0];
      } else {
        kd = timestamps->data[idx - 1];
        while (idx + 1 <= timestamps->size[1]) {
          if (kd < timestamps->data[idx]) {
            kd = timestamps->data[idx];
          }

          idx++;
        }
      }
    }

    if (rtIsNaN(Ma) || rtIsNaN(kd)) {
      i0 = y->size[0] * y->size[1];
      y->size[0] = 1;
      y->size[1] = 1;
      emxEnsureCapacity_real_T(y, i0);
      y->data[0] = rtNaN;
    } else if (kd < Ma) {
      i0 = y->size[0] * y->size[1];
      y->size[0] = 1;
      y->size[1] = 0;
      emxEnsureCapacity_real_T(y, i0);
    } else if ((rtIsInf(Ma) || rtIsInf(kd)) && (Ma == kd)) {
      i0 = y->size[0] * y->size[1];
      y->size[0] = 1;
      y->size[1] = 1;
      emxEnsureCapacity_real_T(y, i0);
      y->data[0] = rtNaN;
    } else {
      ndbl = floor((kd - Ma) / 0.033333333333333333 + 0.5);
      apnd = Ma + ndbl * 0.033333333333333333;
      cdiff = apnd - kd;
      absa = fabs(Ma);
      absb = fabs(kd);
      if ((absa > absb) || rtIsNaN(absb)) {
        absb = absa;
      }

      if (fabs(cdiff) < 4.4408920985006262E-16 * absb) {
        ndbl++;
        apnd = kd;
      } else if (cdiff > 0.0) {
        apnd = Ma + (ndbl - 1.0) * 0.033333333333333333;
      } else {
        ndbl++;
      }

      if (ndbl >= 0.0) {
        idx = (int)ndbl;
      } else {
        idx = 0;
      }

      i0 = y->size[0] * y->size[1];
      y->size[0] = 1;
      y->size[1] = idx;
      emxEnsureCapacity_real_T(y, i0);
      if (idx > 0) {
        y->data[0] = Ma;
        if (idx > 1) {
          y->data[idx - 1] = apnd;
          nm1d2 = (idx - 1) / 2;
          for (k = 1; k < nm1d2; k++) {
            kd = (double)k * 0.033333333333333333;
            y->data[k] = Ma + kd;
            y->data[(idx - k) - 1] = apnd - kd;
          }

          if (nm1d2 << 1 == idx - 1) {
            y->data[nm1d2] = (Ma + apnd) / 2.0;
          } else {
            kd = (double)nm1d2 * 0.033333333333333333;
            y->data[nm1d2] = Ma + kd;
            y->data[nm1d2 + 1] = apnd - kd;
          }
        }
      }
    }

    i0 = b_x->size[0] * b_x->size[1];
    b_x->size[0] = 1;
    b_x->size[1] = varargin_1;
    emxEnsureCapacity_real_T(b_x, i0);
    for (i0 = 0; i0 < varargin_1; i0++) {
      b_x->data[b_x->size[0] * i0] = x->data[i0 + varargin_1 * i];
    }

    interp1(timestamps, b_x, y, freq);
    k = r0->size[0];
    for (i0 = 0; i0 < k; i0++) {
      Rbody_interp->data[r0->data[i0] + Rbody_interp->size[0] * i] = freq->
        data[i0];
    }

    i++;
  }

  loop_ub = Rbody_interp->size[0] * Rbody_interp->size[1] - 1;
  i0 = Rbody_interp->size[0] * Rbody_interp->size[1];
  emxEnsureCapacity_real_T(Rbody_interp, i0);
  for (i0 = 0; i0 <= loop_ub; i0++) {
    Rbody_interp->data[i0]++;
  }

  b_log(Rbody_interp);
  loop_ub = Rbody_interp->size[0] * Rbody_interp->size[1] - 1;
  i0 = Rbody_interp->size[0] * Rbody_interp->size[1];
  emxEnsureCapacity_real_T(Rbody_interp, i0);
  for (i0 = 0; i0 <= loop_ub; i0++) {
    Rbody_interp->data[i0] = -Rbody_interp->data[i0];
  }

  /*  Bhead = permute(frames_head, [2 3 1]);  % make it h x w x t */
  /*  Bhead = imresize(Bhead, 1/block_size, 'box'); */
  permute(frames_head, x);
  varargin_1 = timestamps->size[1];
  if (timestamps->size[1] > 0) {
    calclen = div_s32(x->size[0] * x->size[1] * x->size[2], timestamps->size[1]);
  } else {
    calclen = 0;
  }

  /*  Interpolate between uneven timestamped measurements */
  if (timestamps->size[1] <= 2) {
    if (timestamps->size[1] == 1) {
      Ma = timestamps->data[0];
    } else if ((timestamps->data[0] > timestamps->data[1]) || (rtIsNaN
                (timestamps->data[0]) && (!rtIsNaN(timestamps->data[1])))) {
      Ma = timestamps->data[1];
    } else {
      Ma = timestamps->data[0];
    }
  } else {
    if (!rtIsNaN(timestamps->data[0])) {
      idx = 1;
    } else {
      idx = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= timestamps->size[1])) {
        if (!rtIsNaN(timestamps->data[k - 1])) {
          idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }

    if (idx == 0) {
      Ma = timestamps->data[0];
    } else {
      Ma = timestamps->data[idx - 1];
      while (idx + 1 <= timestamps->size[1]) {
        if (Ma > timestamps->data[idx]) {
          Ma = timestamps->data[idx];
        }

        idx++;
      }
    }
  }

  if (timestamps->size[1] <= 2) {
    if (timestamps->size[1] == 1) {
      kd = timestamps->data[0];
    } else if ((timestamps->data[0] < timestamps->data[1]) || (rtIsNaN
                (timestamps->data[0]) && (!rtIsNaN(timestamps->data[1])))) {
      kd = timestamps->data[1];
    } else {
      kd = timestamps->data[0];
    }
  } else {
    if (!rtIsNaN(timestamps->data[0])) {
      idx = 1;
    } else {
      idx = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= timestamps->size[1])) {
        if (!rtIsNaN(timestamps->data[k - 1])) {
          idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }

    if (idx == 0) {
      kd = timestamps->data[0];
    } else {
      kd = timestamps->data[idx - 1];
      while (idx + 1 <= timestamps->size[1]) {
        if (kd < timestamps->data[idx]) {
          kd = timestamps->data[idx];
        }

        idx++;
      }
    }
  }

  if (rtIsNaN(Ma) || rtIsNaN(kd)) {
    i0 = freq->size[0] * freq->size[1];
    freq->size[1] = 1;
    emxEnsureCapacity_real_T(freq, i0);
  } else if (kd < Ma) {
    i0 = freq->size[0] * freq->size[1];
    freq->size[1] = 0;
    emxEnsureCapacity_real_T(freq, i0);
  } else if ((rtIsInf(Ma) || rtIsInf(kd)) && (Ma == kd)) {
    i0 = freq->size[0] * freq->size[1];
    freq->size[1] = 1;
    emxEnsureCapacity_real_T(freq, i0);
  } else {
    ndbl = floor((kd - Ma) / 0.033333333333333333 + 0.5);
    apnd = Ma + ndbl * 0.033333333333333333;
    cdiff = apnd - kd;
    absa = fabs(Ma);
    absb = fabs(kd);
    if ((absa > absb) || rtIsNaN(absb)) {
      absb = absa;
    }

    if (fabs(cdiff) < 4.4408920985006262E-16 * absb) {
      ndbl++;
      apnd = kd;
    } else if (cdiff > 0.0) {
      apnd = Ma + (ndbl - 1.0) * 0.033333333333333333;
    } else {
      ndbl++;
    }

    if (ndbl >= 0.0) {
      idx = (int)ndbl;
    } else {
      idx = 0;
    }

    i0 = freq->size[0] * freq->size[1];
    freq->size[0] = 1;
    freq->size[1] = idx;
    emxEnsureCapacity_real_T(freq, i0);
    if (idx > 0) {
      freq->data[0] = Ma;
      if (idx > 1) {
        freq->data[idx - 1] = apnd;
        nm1d2 = (idx - 1) / 2;
        for (k = 1; k < nm1d2; k++) {
          kd = (double)k * 0.033333333333333333;
          freq->data[k] = Ma + kd;
          freq->data[(idx - k) - 1] = apnd - kd;
        }

        if (nm1d2 << 1 == idx - 1) {
          freq->data[nm1d2] = (Ma + apnd) / 2.0;
        } else {
          kd = (double)nm1d2 * 0.033333333333333333;
          freq->data[nm1d2] = Ma + kd;
          freq->data[nm1d2 + 1] = apnd - kd;
        }
      }
    }
  }

  emxInit_real_T(&Rhead_interp, 2);
  i0 = Rhead_interp->size[0] * Rhead_interp->size[1];
  Rhead_interp->size[0] = freq->size[1];
  emxEnsureCapacity_real_T(Rhead_interp, i0);
  i0 = Rhead_interp->size[0] * Rhead_interp->size[1];
  Rhead_interp->size[1] = calclen;
  emxEnsureCapacity_real_T(Rhead_interp, i0);
  loop_ub = freq->size[1] * calclen;
  for (i0 = 0; i0 < loop_ub; i0++) {
    Rhead_interp->data[i0] = 0.0;
  }

  for (i = 0; i < calclen; i++) {
    loop_ub = Rhead_interp->size[0];
    i0 = r0->size[0];
    r0->size[0] = loop_ub;
    emxEnsureCapacity_int32_T(r0, i0);
    for (i0 = 0; i0 < loop_ub; i0++) {
      r0->data[i0] = i0;
    }

    if (timestamps->size[1] <= 2) {
      if (timestamps->size[1] == 1) {
        Ma = timestamps->data[0];
      } else if ((timestamps->data[0] > timestamps->data[1]) || (rtIsNaN
                  (timestamps->data[0]) && (!rtIsNaN(timestamps->data[1])))) {
        Ma = timestamps->data[1];
      } else {
        Ma = timestamps->data[0];
      }
    } else {
      if (!rtIsNaN(timestamps->data[0])) {
        idx = 1;
      } else {
        idx = 0;
        k = 2;
        exitg1 = false;
        while ((!exitg1) && (k <= timestamps->size[1])) {
          if (!rtIsNaN(timestamps->data[k - 1])) {
            idx = k;
            exitg1 = true;
          } else {
            k++;
          }
        }
      }

      if (idx == 0) {
        Ma = timestamps->data[0];
      } else {
        Ma = timestamps->data[idx - 1];
        while (idx + 1 <= timestamps->size[1]) {
          if (Ma > timestamps->data[idx]) {
            Ma = timestamps->data[idx];
          }

          idx++;
        }
      }
    }

    if (timestamps->size[1] <= 2) {
      if (timestamps->size[1] == 1) {
        kd = timestamps->data[0];
      } else if ((timestamps->data[0] < timestamps->data[1]) || (rtIsNaN
                  (timestamps->data[0]) && (!rtIsNaN(timestamps->data[1])))) {
        kd = timestamps->data[1];
      } else {
        kd = timestamps->data[0];
      }
    } else {
      if (!rtIsNaN(timestamps->data[0])) {
        idx = 1;
      } else {
        idx = 0;
        k = 2;
        exitg1 = false;
        while ((!exitg1) && (k <= timestamps->size[1])) {
          if (!rtIsNaN(timestamps->data[k - 1])) {
            idx = k;
            exitg1 = true;
          } else {
            k++;
          }
        }
      }

      if (idx == 0) {
        kd = timestamps->data[0];
      } else {
        kd = timestamps->data[idx - 1];
        while (idx + 1 <= timestamps->size[1]) {
          if (kd < timestamps->data[idx]) {
            kd = timestamps->data[idx];
          }

          idx++;
        }
      }
    }

    if (rtIsNaN(Ma) || rtIsNaN(kd)) {
      i0 = y->size[0] * y->size[1];
      y->size[0] = 1;
      y->size[1] = 1;
      emxEnsureCapacity_real_T(y, i0);
      y->data[0] = rtNaN;
    } else if (kd < Ma) {
      i0 = y->size[0] * y->size[1];
      y->size[0] = 1;
      y->size[1] = 0;
      emxEnsureCapacity_real_T(y, i0);
    } else if ((rtIsInf(Ma) || rtIsInf(kd)) && (Ma == kd)) {
      i0 = y->size[0] * y->size[1];
      y->size[0] = 1;
      y->size[1] = 1;
      emxEnsureCapacity_real_T(y, i0);
      y->data[0] = rtNaN;
    } else {
      ndbl = floor((kd - Ma) / 0.033333333333333333 + 0.5);
      apnd = Ma + ndbl * 0.033333333333333333;
      cdiff = apnd - kd;
      absa = fabs(Ma);
      absb = fabs(kd);
      if ((absa > absb) || rtIsNaN(absb)) {
        absb = absa;
      }

      if (fabs(cdiff) < 4.4408920985006262E-16 * absb) {
        ndbl++;
        apnd = kd;
      } else if (cdiff > 0.0) {
        apnd = Ma + (ndbl - 1.0) * 0.033333333333333333;
      } else {
        ndbl++;
      }

      if (ndbl >= 0.0) {
        idx = (int)ndbl;
      } else {
        idx = 0;
      }

      i0 = y->size[0] * y->size[1];
      y->size[0] = 1;
      y->size[1] = idx;
      emxEnsureCapacity_real_T(y, i0);
      if (idx > 0) {
        y->data[0] = Ma;
        if (idx > 1) {
          y->data[idx - 1] = apnd;
          nm1d2 = (idx - 1) / 2;
          for (k = 1; k < nm1d2; k++) {
            kd = (double)k * 0.033333333333333333;
            y->data[k] = Ma + kd;
            y->data[(idx - k) - 1] = apnd - kd;
          }

          if (nm1d2 << 1 == idx - 1) {
            y->data[nm1d2] = (Ma + apnd) / 2.0;
          } else {
            kd = (double)nm1d2 * 0.033333333333333333;
            y->data[nm1d2] = Ma + kd;
            y->data[nm1d2 + 1] = apnd - kd;
          }
        }
      }
    }

    i0 = b_x->size[0] * b_x->size[1];
    b_x->size[0] = 1;
    b_x->size[1] = varargin_1;
    emxEnsureCapacity_real_T(b_x, i0);
    for (i0 = 0; i0 < varargin_1; i0++) {
      b_x->data[b_x->size[0] * i0] = x->data[i0 + varargin_1 * i];
    }

    interp1(timestamps, b_x, y, freq);
    k = r0->size[0];
    for (i0 = 0; i0 < k; i0++) {
      Rhead_interp->data[r0->data[i0] + Rhead_interp->size[0] * i] = freq->
        data[i0];
    }
  }

  emxFree_real_T(&b_x);
  emxFree_real_T(&x);
  loop_ub = Rhead_interp->size[0] * Rhead_interp->size[1] - 1;
  i0 = Rhead_interp->size[0] * Rhead_interp->size[1];
  emxEnsureCapacity_real_T(Rhead_interp, i0);
  for (i0 = 0; i0 <= loop_ub; i0++) {
    Rhead_interp->data[i0]++;
  }

  b_log(Rhead_interp);
  loop_ub = Rhead_interp->size[0] * Rhead_interp->size[1] - 1;
  i0 = Rhead_interp->size[0] * Rhead_interp->size[1];
  emxEnsureCapacity_real_T(Rhead_interp, i0);
  for (i0 = 0; i0 <= loop_ub; i0++) {
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
  kd = fps / (double)Rbody_interp->size[0];
  cdiff = (double)Rbody_interp->size[0] / 2.0;
  nm1d2 = (int)floor(cdiff);
  if (nm1d2 - 1 < 0) {
    i0 = freq->size[0] * freq->size[1];
    freq->size[0] = 1;
    freq->size[1] = 0;
    emxEnsureCapacity_real_T(freq, i0);
  } else {
    i0 = freq->size[0] * freq->size[1];
    freq->size[0] = 1;
    freq->size[1] = nm1d2;
    emxEnsureCapacity_real_T(freq, i0);
    loop_ub = nm1d2 - 1;
    for (i0 = 0; i0 <= loop_ub; i0++) {
      freq->data[freq->size[0] * i0] = i0;
    }
  }

  loop_ub = freq->size[0] * freq->size[1] - 1;
  i0 = freq->size[0] * freq->size[1];
  freq->size[0] = 1;
  emxEnsureCapacity_real_T(freq, i0);
  for (i0 = 0; i0 <= loop_ub; i0++) {
    freq->data[i0] *= kd;
  }

  emxInit_creal_T(&Y, 2);
  fft(Rbody_interp, Rbody_interp->size[0], Y);
  loop_ub = Y->size[0] * Y->size[1] - 1;
  i0 = Y->size[0] * Y->size[1];
  emxEnsureCapacity_creal_T(Y, i0);
  nm1d2 = Rbody_interp->size[0];
  for (i0 = 0; i0 <= loop_ub; i0++) {
    cdiff = Y->data[i0].re;
    kd = -Y->data[i0].im;
    ndbl = Y->data[i0].re * cdiff - Y->data[i0].im * kd;
    kd = Y->data[i0].re * kd + Y->data[i0].im * cdiff;
    if (kd == 0.0) {
      Y->data[i0].re = ndbl / (double)nm1d2;
      Y->data[i0].im = 0.0;
    } else if (ndbl == 0.0) {
      Y->data[i0].re = 0.0;
      Y->data[i0].im = kd / (double)nm1d2;
    } else {
      Y->data[i0].re = ndbl / (double)nm1d2;
      Y->data[i0].im = kd / (double)nm1d2;
    }
  }

  i0 = (int)floor((double)Rbody_interp->size[0] / 2.0);
  if (1 > i0) {
    loop_ub = 0;
  } else {
    loop_ub = i0;
  }

  emxInit_creal_T(&Fbreathing, 2);
  nm1d2 = Y->size[1];
  i0 = Fbreathing->size[0] * Fbreathing->size[1];
  Fbreathing->size[0] = loop_ub;
  Fbreathing->size[1] = nm1d2;
  emxEnsureCapacity_creal_T(Fbreathing, i0);
  for (i0 = 0; i0 < nm1d2; i0++) {
    for (idx = 0; idx < loop_ub; idx++) {
      Fbreathing->data[idx + Fbreathing->size[0] * i0] = Y->data[idx + Y->size[0]
        * i0];
    }
  }

  fft(Rhead_interp, Rbody_interp->size[0], Y);
  loop_ub = Y->size[0] * Y->size[1] - 1;
  i0 = Y->size[0] * Y->size[1];
  emxEnsureCapacity_creal_T(Y, i0);
  nm1d2 = Rbody_interp->size[0];
  emxFree_real_T(&Rhead_interp);
  for (i0 = 0; i0 <= loop_ub; i0++) {
    cdiff = Y->data[i0].re;
    kd = -Y->data[i0].im;
    ndbl = Y->data[i0].re * cdiff - Y->data[i0].im * kd;
    kd = Y->data[i0].re * kd + Y->data[i0].im * cdiff;
    if (kd == 0.0) {
      Y->data[i0].re = ndbl / (double)nm1d2;
      Y->data[i0].im = 0.0;
    } else if (ndbl == 0.0) {
      Y->data[i0].re = 0.0;
      Y->data[i0].im = kd / (double)nm1d2;
    } else {
      Y->data[i0].re = ndbl / (double)nm1d2;
      Y->data[i0].im = kd / (double)nm1d2;
    }
  }

  i0 = (int)floor((double)Rbody_interp->size[0] / 2.0);
  emxFree_real_T(&Rbody_interp);
  if (1 > i0) {
    loop_ub = 0;
  } else {
    loop_ub = i0;
  }

  emxInit_creal_T(&Fheartrate, 2);
  nm1d2 = Y->size[1];
  i0 = Fheartrate->size[0] * Fheartrate->size[1];
  Fheartrate->size[0] = loop_ub;
  Fheartrate->size[1] = nm1d2;
  emxEnsureCapacity_creal_T(Fheartrate, i0);
  for (i0 = 0; i0 < nm1d2; i0++) {
    for (idx = 0; idx < loop_ub; idx++) {
      Fheartrate->data[idx + Fheartrate->size[0] * i0] = Y->data[idx + Y->size[0]
        * i0];
    }
  }

  emxInit_boolean_T(&c_x, 2);

  /*  [freq, Fbreathing] = plot_power_spectrum(Abody, 60); */
  /*  [freq, Fheartrate] = plot_power_spectrum(Ahead_filt, 60); */
  i0 = c_x->size[0] * c_x->size[1];
  c_x->size[0] = 1;
  c_x->size[1] = freq->size[1];
  emxEnsureCapacity_boolean_T(c_x, i0);
  nm1d2 = freq->size[0] * freq->size[1];
  for (i0 = 0; i0 < nm1d2; i0++) {
    c_x->data[i0] = (freq->data[i0] > 0.016666666666666666);
  }

  k = (1 <= c_x->size[1]);
  idx = 0;
  nm1d2 = 1;
  exitg1 = false;
  while ((!exitg1) && (nm1d2 <= c_x->size[1])) {
    if (c_x->data[nm1d2 - 1]) {
      idx = 1;
      ii_data[0] = nm1d2;
      exitg1 = true;
    } else {
      nm1d2++;
    }
  }

  if (k == 1) {
    if (idx == 0) {
      k = 0;
    }
  } else {
    k = !(1 > idx);
  }

  for (i0 = 0; i0 < k; i0++) {
    idx1_data[i0] = (double)ii_data[i0] - 1.0;
  }

  i0 = c_x->size[0] * c_x->size[1];
  c_x->size[0] = 1;
  c_x->size[1] = freq->size[1];
  emxEnsureCapacity_boolean_T(c_x, i0);
  nm1d2 = freq->size[0] * freq->size[1];
  for (i0 = 0; i0 < nm1d2; i0++) {
    c_x->data[i0] = (freq->data[i0] > 0.58333333333333337);
  }

  k = (1 <= c_x->size[1]);
  idx = 0;
  nm1d2 = 1;
  exitg1 = false;
  while ((!exitg1) && (nm1d2 <= c_x->size[1])) {
    if (c_x->data[nm1d2 - 1]) {
      idx = 1;
      ii_data[0] = nm1d2;
      exitg1 = true;
    } else {
      nm1d2++;
    }
  }

  if (k == 1) {
    if (idx == 0) {
      k = 0;
    }
  } else {
    k = !(1 > idx);
  }

  if (0 <= k - 1) {
    memcpy(&idx2_data[0], &ii_data[0], (unsigned int)(k * (int)sizeof(int)));
  }

  if (idx1_data[0] < 1.0) {
    i0 = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = 0;
    emxEnsureCapacity_real_T(y, i0);
  } else {
    i0 = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = (int)idx1_data[0];
    emxEnsureCapacity_real_T(y, i0);
    nm1d2 = (int)idx1_data[0] - 1;
    for (i0 = 0; i0 <= nm1d2; i0++) {
      y->data[y->size[0] * i0] = 1.0 + (double)i0;
    }
  }

  emxInit_real_T(&b_y, 2);
  if (Fbreathing->size[0] < idx2_data[0]) {
    i0 = b_y->size[0] * b_y->size[1];
    b_y->size[0] = 1;
    b_y->size[1] = 0;
    emxEnsureCapacity_real_T(b_y, i0);
  } else {
    i0 = Fbreathing->size[0];
    idx = b_y->size[0] * b_y->size[1];
    b_y->size[0] = 1;
    b_y->size[1] = (i0 - idx2_data[0]) + 1;
    emxEnsureCapacity_real_T(b_y, idx);
    nm1d2 = i0 - idx2_data[0];
    for (i0 = 0; i0 <= nm1d2; i0++) {
      b_y->data[b_y->size[0] * i0] = idx2_data[0] + i0;
    }
  }

  i0 = r0->size[0];
  r0->size[0] = y->size[1] + b_y->size[1];
  emxEnsureCapacity_int32_T(r0, i0);
  nm1d2 = y->size[1];
  for (i0 = 0; i0 < nm1d2; i0++) {
    r0->data[i0] = (int)y->data[y->size[0] * i0];
  }

  nm1d2 = b_y->size[1];
  for (i0 = 0; i0 < nm1d2; i0++) {
    r0->data[i0 + y->size[1]] = (int)b_y->data[b_y->size[0] * i0];
  }

  nm1d2 = Fbreathing->size[1];
  k = r0->size[0];
  for (i0 = 0; i0 < nm1d2; i0++) {
    for (idx = 0; idx < k; idx++) {
      Fbreathing->data[(r0->data[idx] + Fbreathing->size[0] * i0) - 1].re = 0.0;
      Fbreathing->data[(r0->data[idx] + Fbreathing->size[0] * i0) - 1].im = 0.0;
    }
  }

  i0 = c_x->size[0] * c_x->size[1];
  c_x->size[0] = 1;
  c_x->size[1] = freq->size[1];
  emxEnsureCapacity_boolean_T(c_x, i0);
  nm1d2 = freq->size[0] * freq->size[1];
  for (i0 = 0; i0 < nm1d2; i0++) {
    c_x->data[i0] = (freq->data[i0] > 0.66666666666666663);
  }

  k = (1 <= c_x->size[1]);
  idx = 0;
  nm1d2 = 1;
  exitg1 = false;
  while ((!exitg1) && (nm1d2 <= c_x->size[1])) {
    if (c_x->data[nm1d2 - 1]) {
      idx = 1;
      ii_data[0] = nm1d2;
      exitg1 = true;
    } else {
      nm1d2++;
    }
  }

  if (k == 1) {
    if (idx == 0) {
      k = 0;
    }
  } else {
    k = !(1 > idx);
  }

  for (i0 = 0; i0 < k; i0++) {
    idx1_data[i0] = (double)ii_data[i0] - 1.0;
  }

  i0 = c_x->size[0] * c_x->size[1];
  c_x->size[0] = 1;
  c_x->size[1] = freq->size[1];
  emxEnsureCapacity_boolean_T(c_x, i0);
  nm1d2 = freq->size[0] * freq->size[1];
  for (i0 = 0; i0 < nm1d2; i0++) {
    c_x->data[i0] = (freq->data[i0] > 1.6666666666666667);
  }

  k = (1 <= c_x->size[1]);
  idx = 0;
  nm1d2 = 1;
  exitg1 = false;
  while ((!exitg1) && (nm1d2 <= c_x->size[1])) {
    if (c_x->data[nm1d2 - 1]) {
      idx = 1;
      ii_data[0] = nm1d2;
      exitg1 = true;
    } else {
      nm1d2++;
    }
  }

  emxFree_boolean_T(&c_x);
  if (k == 1) {
    if (idx == 0) {
      k = 0;
    }
  } else {
    k = !(1 > idx);
  }

  if (0 <= k - 1) {
    memcpy(&idx2_data[0], &ii_data[0], (unsigned int)(k * (int)sizeof(int)));
  }

  /*  Fheartrate(1,:) = 0; */
  if (idx1_data[0] < 1.0) {
    i0 = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = 0;
    emxEnsureCapacity_real_T(y, i0);
  } else {
    i0 = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = (int)idx1_data[0];
    emxEnsureCapacity_real_T(y, i0);
    nm1d2 = (int)idx1_data[0] - 1;
    for (i0 = 0; i0 <= nm1d2; i0++) {
      y->data[y->size[0] * i0] = 1.0 + (double)i0;
    }
  }

  if (loop_ub < idx2_data[0]) {
    i0 = b_y->size[0] * b_y->size[1];
    b_y->size[0] = 1;
    b_y->size[1] = 0;
    emxEnsureCapacity_real_T(b_y, i0);
  } else {
    i0 = b_y->size[0] * b_y->size[1];
    b_y->size[0] = 1;
    b_y->size[1] = (loop_ub - idx2_data[0]) + 1;
    emxEnsureCapacity_real_T(b_y, i0);
    loop_ub -= idx2_data[0];
    for (i0 = 0; i0 <= loop_ub; i0++) {
      b_y->data[b_y->size[0] * i0] = idx2_data[0] + i0;
    }
  }

  i0 = r0->size[0];
  r0->size[0] = y->size[1] + b_y->size[1];
  emxEnsureCapacity_int32_T(r0, i0);
  loop_ub = y->size[1];
  for (i0 = 0; i0 < loop_ub; i0++) {
    r0->data[i0] = (int)y->data[y->size[0] * i0];
  }

  loop_ub = b_y->size[1];
  for (i0 = 0; i0 < loop_ub; i0++) {
    r0->data[i0 + y->size[1]] = (int)b_y->data[b_y->size[0] * i0];
  }

  emxFree_real_T(&b_y);
  emxFree_real_T(&y);
  loop_ub = Y->size[1];
  k = r0->size[0];
  for (i0 = 0; i0 < loop_ub; i0++) {
    for (idx = 0; idx < k; idx++) {
      Fheartrate->data[(r0->data[idx] + Fheartrate->size[0] * i0) - 1].re = 0.0;
      Fheartrate->data[(r0->data[idx] + Fheartrate->size[0] * i0) - 1].im = 0.0;
    }
  }

  emxFree_int32_T(&r0);

  /*  possible error */
  i0 = Y->size[0] * Y->size[1];
  Y->size[0] = Fheartrate->size[0];
  Y->size[1] = Fheartrate->size[1];
  emxEnsureCapacity_creal_T(Y, i0);
  loop_ub = Fheartrate->size[0] * Fheartrate->size[1];
  for (i0 = 0; i0 < loop_ub; i0++) {
    Y->data[i0] = Fheartrate->data[i0];
  }

  emxInit_creal_T(&HRentr, 2);
  get_spectral_entropy(Y, HRentr);
  loop_ub = HRentr->size[0] * HRentr->size[1] - 1;
  i0 = HRentr->size[0] * HRentr->size[1];
  HRentr->size[0] = 1;
  emxEnsureCapacity_creal_T(HRentr, i0);
  for (i0 = 0; i0 <= loop_ub; i0++) {
    HRentr->data[i0].re = (1.0 - HRentr->data[i0].re) - 0.1;
    HRentr->data[i0].im = 0.0 - HRentr->data[i0].im;
  }

  emxInit_creal_T(&whr, 2);
  nm1d2 = HRentr->size[1];
  i0 = whr->size[0] * whr->size[1];
  whr->size[0] = 1;
  whr->size[1] = HRentr->size[1];
  emxEnsureCapacity_creal_T(whr, i0);
  for (k = 0; k < nm1d2; k++) {
    apnd = HRentr->data[k].re;
    ex_im = HRentr->data[k].im;
    if (rtIsNaN(HRentr->data[k].re) || rtIsNaN(HRentr->data[k].im)) {
      breathing = true;
    } else {
      if ((fabs(HRentr->data[k].re) > 8.9884656743115785E+307) || (fabs
           (HRentr->data[k].im) > 8.9884656743115785E+307)) {
        breathing = true;
      } else {
        breathing = false;
      }

      if (breathing) {
        cdiff = rt_hypotd_snf(HRentr->data[k].re / 2.0, HRentr->data[k].im / 2.0);
      } else {
        cdiff = rt_hypotd_snf(HRentr->data[k].re, HRentr->data[k].im);
      }

      if (cdiff == 0.0) {
        absa = fabs(HRentr->data[k].re);
        kd = fabs(HRentr->data[k].im);
        if (absa > kd) {
          Ma = absa;
          absa = kd;
        } else {
          Ma = kd;
        }

        if (Ma > 0.0) {
          cdiff = Ma;
        } else {
          cdiff = absa;
        }

        if (cdiff == 0.0) {
          cdiff = rt_atan2d_snf(HRentr->data[k].im, HRentr->data[k].re);
          if (cdiff == 0.0) {
            cdiff = HRentr->data[k].im;
            if (cdiff == 0.0) {
              cdiff = 0.0;
            }
          }
        }
      }

      breathing = (cdiff < 0.0);
    }

    if (breathing) {
      apnd = 0.0;
      ex_im = 0.0;
    }

    whr->data[k].re = apnd;
    whr->data[k].im = ex_im;
  }

  emxInit_creal_T(&b_whr, 2);
  i0 = b_whr->size[0] * b_whr->size[1];
  b_whr->size[0] = 1;
  b_whr->size[1] = whr->size[1];
  emxEnsureCapacity_creal_T(b_whr, i0);
  loop_ub = whr->size[0] * whr->size[1];
  for (i0 = 0; i0 < loop_ub; i0++) {
    b_whr->data[i0] = whr->data[i0];
  }

  emxInit_creal_T1(&b_breathing, 1);
  dc0 = sum(whr);
  rdivide(b_whr, dc0, whr);
  i0 = b_breathing->size[0];
  b_breathing->size[0] = whr->size[1];
  emxEnsureCapacity_creal_T1(b_breathing, i0);
  loop_ub = whr->size[1];
  for (i0 = 0; i0 < loop_ub; i0++) {
    b_breathing->data[i0].re = whr->data[whr->size[0] * i0].re;
    b_breathing->data[i0].im = -whr->data[whr->size[0] * i0].im;
  }

  emxFree_creal_T(&whr);
  emxInit_creal_T1(&heartrate, 1);
  if ((Fheartrate->size[1] == 1) || (b_breathing->size[0] == 1)) {
    i0 = heartrate->size[0];
    heartrate->size[0] = Fheartrate->size[0];
    emxEnsureCapacity_creal_T1(heartrate, i0);
    loop_ub = Fheartrate->size[0];
    for (i0 = 0; i0 < loop_ub; i0++) {
      heartrate->data[i0].re = 0.0;
      heartrate->data[i0].im = 0.0;
      nm1d2 = Fheartrate->size[1];
      for (idx = 0; idx < nm1d2; idx++) {
        kd = Fheartrate->data[i0 + Fheartrate->size[0] * idx].re *
          b_breathing->data[idx].re - Fheartrate->data[i0 + Fheartrate->size[0] *
          idx].im * b_breathing->data[idx].im;
        cdiff = Fheartrate->data[i0 + Fheartrate->size[0] * idx].re *
          b_breathing->data[idx].im + Fheartrate->data[i0 + Fheartrate->size[0] *
          idx].im * b_breathing->data[idx].re;
        heartrate->data[i0].re += kd;
        heartrate->data[i0].im += cdiff;
      }
    }
  } else {
    idx = Fheartrate->size[0];
    i0 = heartrate->size[0];
    heartrate->size[0] = Fheartrate->size[0];
    emxEnsureCapacity_creal_T1(heartrate, i0);
    for (i = 1; i <= idx; i++) {
      heartrate->data[i - 1].re = 0.0;
      heartrate->data[i - 1].im = 0.0;
    }

    for (k = 0; k < Fheartrate->size[1]; k++) {
      breathing = ((b_breathing->data[k].re != 0.0) || (b_breathing->data[k].im
        != 0.0));
      if (breathing) {
        nm1d2 = k * idx;
        for (i = 0; i < idx; i++) {
          kd = b_breathing->data[k].re * Fheartrate->data[nm1d2 + i].re -
            b_breathing->data[k].im * Fheartrate->data[nm1d2 + i].im;
          cdiff = b_breathing->data[k].re * Fheartrate->data[nm1d2 + i].im +
            b_breathing->data[k].im * Fheartrate->data[nm1d2 + i].re;
          heartrate->data[i].re += kd;
          heartrate->data[i].im += cdiff;
        }
      }
    }
  }

  emxFree_creal_T(&Fheartrate);
  idx = 0;
  apnd = heartrate->data[0].re;
  ex_im = heartrate->data[0].im;
  for (k = 1; k < heartrate->size[0]; k++) {
    if (rtIsNaN(heartrate->data[k].re) || rtIsNaN(heartrate->data[k].im)) {
      breathing = false;
    } else if (rtIsNaN(apnd) || rtIsNaN(ex_im)) {
      breathing = true;
    } else {
      if ((fabs(apnd) > 8.9884656743115785E+307) || (fabs(ex_im) >
           8.9884656743115785E+307)) {
        breathing = true;
      } else {
        breathing = false;
      }

      if ((fabs(heartrate->data[k].re) > 8.9884656743115785E+307) || (fabs
           (heartrate->data[k].im) > 8.9884656743115785E+307)) {
        SCALEB = true;
      } else {
        SCALEB = false;
      }

      if (breathing || SCALEB) {
        cdiff = rt_hypotd_snf(apnd / 2.0, ex_im / 2.0);
        ndbl = rt_hypotd_snf(heartrate->data[k].re / 2.0, heartrate->data[k].im /
                             2.0);
      } else {
        cdiff = rt_hypotd_snf(apnd, ex_im);
        ndbl = rt_hypotd_snf(heartrate->data[k].re, heartrate->data[k].im);
      }

      if (cdiff == ndbl) {
        absa = fabs(apnd);
        kd = fabs(ex_im);
        absb = fabs(heartrate->data[k].re);
        ndbl = fabs(heartrate->data[k].im);
        if (absa > kd) {
          Ma = absa;
          absa = kd;
        } else {
          Ma = kd;
        }

        if (absb > ndbl) {
          kd = absb;
          absb = ndbl;
        } else {
          kd = ndbl;
        }

        if (Ma > kd) {
          if (absa < absb) {
            cdiff = Ma - kd;
            ndbl = (absa / 2.0 + absb / 2.0) / (Ma / 2.0 + kd / 2.0) * (absb -
              absa);
          } else {
            cdiff = Ma;
            ndbl = kd;
          }
        } else if (Ma < kd) {
          if (absa > absb) {
            ndbl = kd - Ma;
            cdiff = (absa / 2.0 + absb / 2.0) / (Ma / 2.0 + kd / 2.0) * (absa -
              absb);
          } else {
            cdiff = Ma;
            ndbl = kd;
          }
        } else {
          cdiff = absa;
          ndbl = absb;
        }

        if (cdiff == ndbl) {
          cdiff = rt_atan2d_snf(ex_im, apnd);
          ndbl = rt_atan2d_snf(heartrate->data[k].im, heartrate->data[k].re);
          if (cdiff == ndbl) {
            ndbl = heartrate->data[k].re;
            kd = heartrate->data[k].im;
            if (cdiff > 0.78539816339744828) {
              if (cdiff > 2.3561944901923448) {
                cdiff = -ex_im;
                ndbl = -kd;
              } else {
                cdiff = -apnd;
                ndbl = -ndbl;
              }
            } else if (cdiff > -0.78539816339744828) {
              cdiff = ex_im;
              ndbl = kd;
            } else if (cdiff > -2.3561944901923448) {
              cdiff = apnd;
            } else {
              cdiff = -ex_im;
              ndbl = -kd;
            }

            if (cdiff == ndbl) {
              cdiff = 0.0;
              ndbl = 0.0;
            }
          }
        }
      }

      breathing = (cdiff < ndbl);
    }

    if (breathing) {
      apnd = heartrate->data[k].re;
      ex_im = heartrate->data[k].im;
      idx = k;
    }
  }

  emxFree_creal_T(&heartrate);
  *hr = 60.0 * freq->data[idx];
  i0 = Y->size[0] * Y->size[1];
  Y->size[0] = Fbreathing->size[0];
  Y->size[1] = Fbreathing->size[1];
  emxEnsureCapacity_creal_T(Y, i0);
  loop_ub = Fbreathing->size[0] * Fbreathing->size[1];
  for (i0 = 0; i0 < loop_ub; i0++) {
    Y->data[i0] = Fbreathing->data[i0];
  }

  get_spectral_entropy(Y, HRentr);
  loop_ub = HRentr->size[0] * HRentr->size[1] - 1;
  i0 = HRentr->size[0] * HRentr->size[1];
  HRentr->size[0] = 1;
  emxEnsureCapacity_creal_T(HRentr, i0);
  emxFree_creal_T(&Y);
  for (i0 = 0; i0 <= loop_ub; i0++) {
    HRentr->data[i0].re = 1.0 - HRentr->data[i0].re;
    HRentr->data[i0].im = 0.0 - HRentr->data[i0].im;
  }

  i0 = b_whr->size[0] * b_whr->size[1];
  b_whr->size[0] = 1;
  b_whr->size[1] = HRentr->size[1];
  emxEnsureCapacity_creal_T(b_whr, i0);
  loop_ub = HRentr->size[0] * HRentr->size[1];
  for (i0 = 0; i0 < loop_ub; i0++) {
    b_whr->data[i0] = HRentr->data[i0];
  }

  dc0 = sum(HRentr);
  rdivide(b_whr, dc0, HRentr);
  emxFree_creal_T(&b_whr);
  if ((Fbreathing->size[1] == 1) || (HRentr->size[1] == 1)) {
    i0 = b_breathing->size[0];
    b_breathing->size[0] = Fbreathing->size[0];
    emxEnsureCapacity_creal_T1(b_breathing, i0);
    loop_ub = Fbreathing->size[0];
    for (i0 = 0; i0 < loop_ub; i0++) {
      b_breathing->data[i0].re = 0.0;
      b_breathing->data[i0].im = 0.0;
      nm1d2 = Fbreathing->size[1];
      for (idx = 0; idx < nm1d2; idx++) {
        cdiff = Fbreathing->data[i0 + Fbreathing->size[0] * idx].re *
          HRentr->data[idx].re - Fbreathing->data[i0 + Fbreathing->size[0] * idx]
          .im * HRentr->data[idx].im;
        kd = Fbreathing->data[i0 + Fbreathing->size[0] * idx].re * HRentr->
          data[idx].im + Fbreathing->data[i0 + Fbreathing->size[0] * idx].im *
          HRentr->data[idx].re;
        b_breathing->data[i0].re += cdiff;
        b_breathing->data[i0].im += kd;
      }
    }
  } else {
    idx = Fbreathing->size[0];
    i0 = b_breathing->size[0];
    b_breathing->size[0] = Fbreathing->size[0];
    emxEnsureCapacity_creal_T1(b_breathing, i0);
    for (i = 1; i <= idx; i++) {
      b_breathing->data[i - 1].re = 0.0;
      b_breathing->data[i - 1].im = 0.0;
    }

    for (k = 0; k < Fbreathing->size[1]; k++) {
      breathing = ((HRentr->data[k].re != 0.0) || (HRentr->data[k].im != 0.0));
      if (breathing) {
        nm1d2 = k * idx;
        for (i = 0; i < idx; i++) {
          cdiff = HRentr->data[k].re * Fbreathing->data[nm1d2 + i].re -
            HRentr->data[k].im * Fbreathing->data[nm1d2 + i].im;
          kd = HRentr->data[k].re * Fbreathing->data[nm1d2 + i].im +
            HRentr->data[k].im * Fbreathing->data[nm1d2 + i].re;
          b_breathing->data[i].re += cdiff;
          b_breathing->data[i].im += kd;
        }
      }
    }
  }

  emxFree_creal_T(&Fbreathing);
  emxFree_creal_T(&HRentr);
  idx = 0;
  apnd = b_breathing->data[0].re;
  ex_im = b_breathing->data[0].im;
  for (k = 1; k < b_breathing->size[0]; k++) {
    if (rtIsNaN(b_breathing->data[k].re) || rtIsNaN(b_breathing->data[k].im)) {
      breathing = false;
    } else if (rtIsNaN(apnd) || rtIsNaN(ex_im)) {
      breathing = true;
    } else {
      if ((fabs(apnd) > 8.9884656743115785E+307) || (fabs(ex_im) >
           8.9884656743115785E+307)) {
        breathing = true;
      } else {
        breathing = false;
      }

      if ((fabs(b_breathing->data[k].re) > 8.9884656743115785E+307) || (fabs
           (b_breathing->data[k].im) > 8.9884656743115785E+307)) {
        SCALEB = true;
      } else {
        SCALEB = false;
      }

      if (breathing || SCALEB) {
        cdiff = rt_hypotd_snf(apnd / 2.0, ex_im / 2.0);
        ndbl = rt_hypotd_snf(b_breathing->data[k].re / 2.0, b_breathing->data[k]
                             .im / 2.0);
      } else {
        cdiff = rt_hypotd_snf(apnd, ex_im);
        ndbl = rt_hypotd_snf(b_breathing->data[k].re, b_breathing->data[k].im);
      }

      if (cdiff == ndbl) {
        absa = fabs(apnd);
        kd = fabs(ex_im);
        absb = fabs(b_breathing->data[k].re);
        ndbl = fabs(b_breathing->data[k].im);
        if (absa > kd) {
          Ma = absa;
          absa = kd;
        } else {
          Ma = kd;
        }

        if (absb > ndbl) {
          kd = absb;
          absb = ndbl;
        } else {
          kd = ndbl;
        }

        if (Ma > kd) {
          if (absa < absb) {
            cdiff = Ma - kd;
            ndbl = (absa / 2.0 + absb / 2.0) / (Ma / 2.0 + kd / 2.0) * (absb -
              absa);
          } else {
            cdiff = Ma;
            ndbl = kd;
          }
        } else if (Ma < kd) {
          if (absa > absb) {
            ndbl = kd - Ma;
            cdiff = (absa / 2.0 + absb / 2.0) / (Ma / 2.0 + kd / 2.0) * (absa -
              absb);
          } else {
            cdiff = Ma;
            ndbl = kd;
          }
        } else {
          cdiff = absa;
          ndbl = absb;
        }

        if (cdiff == ndbl) {
          cdiff = rt_atan2d_snf(ex_im, apnd);
          ndbl = rt_atan2d_snf(b_breathing->data[k].im, b_breathing->data[k].re);
          if (cdiff == ndbl) {
            ndbl = b_breathing->data[k].re;
            kd = b_breathing->data[k].im;
            if (cdiff > 0.78539816339744828) {
              if (cdiff > 2.3561944901923448) {
                cdiff = -ex_im;
                ndbl = -kd;
              } else {
                cdiff = -apnd;
                ndbl = -ndbl;
              }
            } else if (cdiff > -0.78539816339744828) {
              cdiff = ex_im;
              ndbl = kd;
            } else if (cdiff > -2.3561944901923448) {
              cdiff = apnd;
            } else {
              cdiff = -ex_im;
              ndbl = -kd;
            }

            if (cdiff == ndbl) {
              cdiff = 0.0;
              ndbl = 0.0;
            }
          }
        }
      }

      breathing = (cdiff < ndbl);
    }

    if (breathing) {
      apnd = b_breathing->data[k].re;
      ex_im = b_breathing->data[k].im;
      idx = k;
    }
  }

  emxFree_creal_T(&b_breathing);
  *rr = 60.0 * freq->data[idx];

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
  /*   */
  /*  subplot(2,2,2), plot(t,breathing), title('Breathing') */
  /*  xlabel('time (s)') */
  /*  ylabel('breathing amplitude (a.u.)') */
  /*   */
  /*  subplot(2,2,3), plot(60*freq,Fheartrate), title(sprintf('HR=%u bpm',round(hr*60))) */
  /*  [~,maxidx] = max(Fheartrate); */
  /*  hold on, plot(60*freq(maxidx),Fheartrate(maxidx),'or') */
  /*  xlim([0 300]) */
  /*  xlabel('frequency (1/min)') */
  /*  ylabel('spectral power (dB)') */
  /*   */
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
