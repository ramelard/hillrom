/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: permute.c
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 08-Aug-2019 11:00:09
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "extract_vitals_tk1.h"
#include "permute.h"
#include "extract_vitals_tk1_emxutil.h"

/* Function Definitions */

/*
 * Arguments    : const emxArray_real_T *a
 *                emxArray_real_T *b
 * Return Type  : void
 */
void permute(const emxArray_real_T *a, emxArray_real_T *b)
{
  boolean_T b_b;
  int plast;
  int k;
  int b_k;
  boolean_T exitg1;
  static const signed char iv0[3] = { 3, 1, 2 };

  int subsa_idx_1;
  int subsa_idx_2;
  b_b = true;
  if (!((a->size[0] == 0) || (a->size[1] == 0) || (a->size[2] == 0))) {
    plast = 0;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k + 1 < 4)) {
      if (a->size[iv0[k] - 1] != 1) {
        if (plast > iv0[k]) {
          b_b = false;
          exitg1 = true;
        } else {
          plast = iv0[k];
          k++;
        }
      } else {
        k++;
      }
    }
  }

  if (b_b) {
    b_k = b->size[0] * b->size[1] * b->size[2];
    b->size[0] = a->size[2];
    b->size[1] = a->size[0];
    b->size[2] = a->size[1];
    emxEnsureCapacity_real_T1(b, b_k);
    plast = a->size[2] * a->size[0] * a->size[1];
    for (b_k = 0; b_k < plast; b_k++) {
      b->data[b_k] = a->data[b_k];
    }
  } else {
    b_k = b->size[0] * b->size[1] * b->size[2];
    b->size[0] = a->size[2];
    b->size[1] = a->size[0];
    b->size[2] = a->size[1];
    emxEnsureCapacity_real_T1(b, b_k);
    for (k = 1; k <= a->size[2]; k++) {
      for (plast = 1; plast <= a->size[1]; plast++) {
        if (1 <= a->size[0]) {
          subsa_idx_1 = plast;
          subsa_idx_2 = k;
        }

        for (b_k = 1; b_k <= a->size[0]; b_k++) {
          b->data[((subsa_idx_2 + b->size[0] * (b_k - 1)) + b->size[0] * b->
                   size[1] * (subsa_idx_1 - 1)) - 1] = a->data[((b_k + a->size[0]
            * (subsa_idx_1 - 1)) + a->size[0] * a->size[1] * (subsa_idx_2 - 1))
            - 1];
        }
      }
    }
  }
}

/*
 * File trailer for permute.c
 *
 * [EOF]
 */
