/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: permute.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 06-Sep-2018 13:47:30
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "extract_vitals_tk1.h"
#include "permute.h"
#include "extract_vitals_tk1_emxutil.h"

/* Function Declarations */
static boolean_T nomovement(const double p[3], const emxArray_real_T *a);

/* Function Definitions */

/*
 * Arguments    : const double p[3]
 *                const emxArray_real_T *a
 * Return Type  : boolean_T
 */
static boolean_T nomovement(const double p[3], const emxArray_real_T *a)
{
  boolean_T b;
  double plast;
  int k;
  boolean_T exitg1;
  boolean_T guard1 = false;
  b = true;
  if (!((a->size[0] == 0) || (a->size[1] == 0) || (a->size[2] == 0))) {
    plast = 0.0;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k + 1 < 4)) {
      guard1 = false;
      if (a->size[(int)p[k] - 1] != 1) {
        if (plast > p[k]) {
          b = false;
          exitg1 = true;
        } else {
          plast = p[k];
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

  return b;
}

/*
 * Arguments    : const emxArray_real_T *a
 *                emxArray_real_T *b
 * Return Type  : void
 */
void b_permute(const emxArray_real_T *a, emxArray_real_T *b)
{
  unsigned int insz[3];
  int idest;
  unsigned int outsz[3];
  static const double dv1[3] = { 3.0, 1.0, 2.0 };

  int iwork[3];
  int k;
  int inc[3];
  static const signed char iv2[3] = { 2, 0, 1 };

  int isrc;
  int exitg1;
  for (idest = 0; idest < 3; idest++) {
    insz[idest] = (unsigned int)a->size[idest];
  }

  outsz[0] = insz[2];
  outsz[1] = insz[0];
  outsz[2] = insz[1];
  idest = b->size[0] * b->size[1] * b->size[2];
  b->size[0] = (int)insz[2];
  b->size[1] = (int)insz[0];
  b->size[2] = (int)insz[1];
  emxEnsureCapacity((emxArray__common *)b, idest, (int)sizeof(double));
  if (nomovement(dv1, a)) {
    idest = a->size[0] * a->size[1] * a->size[2];
    for (k = 0; k + 1 <= idest; k++) {
      b->data[k] = a->data[k];
    }
  } else {
    for (idest = 0; idest < 3; idest++) {
      iwork[idest] = 1;
    }

    for (k = 0; k < 2; k++) {
      iwork[k + 1] = iwork[k] * (int)insz[k];
    }

    for (idest = 0; idest < 3; idest++) {
      inc[idest] = iwork[iv2[idest]];
    }

    for (idest = 0; idest < 3; idest++) {
      iwork[idest] = 0;
    }

    idest = 0;
    do {
      isrc = 0;
      for (k = 0; k < 2; k++) {
        isrc += iwork[k + 1] * inc[k + 1];
      }

      for (k = 1; k <= (int)outsz[0]; k++) {
        b->data[idest] = a->data[isrc];
        idest++;
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
 * Arguments    : const emxArray_real_T *a
 *                emxArray_real_T *b
 * Return Type  : void
 */
void permute(const emxArray_real_T *a, emxArray_real_T *b)
{
  unsigned int insz[3];
  int idest;
  unsigned int outsz[3];
  static const double dv0[3] = { 2.0, 3.0, 1.0 };

  int iwork[3];
  int k;
  int inc[3];
  static const signed char iv1[3] = { 1, 2, 0 };

  int isrc;
  int exitg1;
  for (idest = 0; idest < 3; idest++) {
    insz[idest] = (unsigned int)a->size[idest];
  }

  outsz[0] = insz[1];
  outsz[1] = insz[2];
  outsz[2] = insz[0];
  idest = b->size[0] * b->size[1] * b->size[2];
  b->size[0] = (int)insz[1];
  b->size[1] = (int)insz[2];
  b->size[2] = (int)insz[0];
  emxEnsureCapacity((emxArray__common *)b, idest, (int)sizeof(double));
  if (nomovement(dv0, a)) {
    idest = a->size[0] * a->size[1] * a->size[2];
    for (k = 0; k + 1 <= idest; k++) {
      b->data[k] = a->data[k];
    }
  } else {
    for (idest = 0; idest < 3; idest++) {
      iwork[idest] = 1;
    }

    for (k = 0; k < 2; k++) {
      iwork[k + 1] = iwork[k] * (int)insz[k];
    }

    for (idest = 0; idest < 3; idest++) {
      inc[idest] = iwork[iv1[idest]];
    }

    for (idest = 0; idest < 3; idest++) {
      iwork[idest] = 0;
    }

    idest = 0;
    do {
      isrc = 0;
      for (k = 0; k < 2; k++) {
        isrc += iwork[k + 1] * inc[k + 1];
      }

      for (k = 1; k <= (int)outsz[0]; k++) {
        b->data[idest] = a->data[isrc];
        idest++;
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
