/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: permute.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 18-Apr-2018 17:29:29
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
  unsigned int insz[3];
  int plast;
  unsigned int outsz[3];
  boolean_T b_b;
  int k;
  int iwork[3];
  boolean_T exitg2;
  boolean_T guard1 = false;
  static const signed char iv1[3] = { 3, 1, 2 };

  int inc[3];
  static const signed char iv2[3] = { 2, 0, 1 };

  int isrc;
  int exitg1;
  for (plast = 0; plast < 3; plast++) {
    insz[plast] = (unsigned int)a->size[plast];
  }

  outsz[0] = insz[2];
  outsz[1] = insz[0];
  outsz[2] = insz[1];
  plast = b->size[0] * b->size[1] * b->size[2];
  b->size[0] = (int)insz[2];
  b->size[1] = (int)insz[0];
  b->size[2] = (int)insz[1];
  emxEnsureCapacity((emxArray__common *)b, plast, (int)sizeof(double));
  b_b = true;
  if (!((a->size[0] == 0) || (a->size[1] == 0) || (a->size[2] == 0))) {
    plast = 0;
    k = 0;
    exitg2 = false;
    while ((!exitg2) && (k + 1 < 4)) {
      guard1 = false;
      if (a->size[iv1[k] - 1] != 1) {
        if (plast > iv1[k]) {
          b_b = false;
          exitg2 = true;
        } else {
          plast = iv1[k];
          guard1 = true;
        }
      } else {
        guard1 = true;
      }

      if (guard1) {
        k++;
      }
    }
  }

  if (b_b) {
    plast = a->size[0] * a->size[1] * a->size[2];
    for (k = 0; k + 1 <= plast; k++) {
      b->data[k] = a->data[k];
    }
  } else {
    for (plast = 0; plast < 3; plast++) {
      iwork[plast] = 1;
    }

    for (k = 0; k < 2; k++) {
      iwork[k + 1] = iwork[k] * (int)insz[k];
    }

    for (plast = 0; plast < 3; plast++) {
      inc[plast] = iwork[iv2[plast]];
    }

    for (plast = 0; plast < 3; plast++) {
      iwork[plast] = 0;
    }

    plast = 0;
    do {
      isrc = 0;
      for (k = 0; k < 2; k++) {
        isrc += iwork[k + 1] * inc[k + 1];
      }

      for (k = 1; k <= (int)outsz[0]; k++) {
        b->data[plast] = a->data[isrc];
        plast++;
        isrc += inc[0];
      }

      k = 1;
      do {
        exitg1 = 0;
        iwork[k]++;
        if (iwork[k] < (int)outsz[k]) {
          exitg1 = 2;
        } else if (k + 1 == 3) {
          exitg1 = 1;
        } else {
          iwork[1] = 0;
          k = 2;
        }
      } while (exitg1 == 0);
    } while (!(exitg1 == 1));
  }
}

/*
 * File trailer for permute.c
 *
 * [EOF]
 */
