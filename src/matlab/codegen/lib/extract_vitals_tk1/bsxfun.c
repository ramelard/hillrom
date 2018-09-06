/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: bsxfun.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 06-Sep-2018 16:44:03
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "extract_vitals_tk1.h"
#include "bsxfun.h"
#include "extract_vitals_tk1_emxutil.h"

/* Function Definitions */

/*
 * Arguments    : const emxArray_real_T *a
 *                const emxArray_real_T *b
 *                emxArray_real_T *c
 * Return Type  : void
 */
void b_bsxfun(const emxArray_real_T *a, const emxArray_real_T *b,
              emxArray_real_T *c)
{
  int nb1;
  int b_c;
  int csz_idx_1;
  int i4;
  emxArray_real_T *av;
  emxArray_real_T *bv;
  unsigned int a_idx_0;
  int bsub;
  int bk;
  int nc1;
  int ck;
  emxArray_real_T *cv;
  nb1 = b->size[0];
  if (a->size[0] <= b->size[0]) {
    b_c = a->size[0];
  } else {
    b_c = b->size[0];
  }

  csz_idx_1 = b->size[1];
  i4 = c->size[0] * c->size[1];
  c->size[0] = b_c;
  c->size[1] = csz_idx_1;
  emxEnsureCapacity((emxArray__common *)c, i4, (int)sizeof(double));
  if (!((c->size[0] == 0) || (c->size[1] == 0))) {
    emxInit_real_T2(&av, 1);
    emxInit_real_T2(&bv, 1);
    a_idx_0 = (unsigned int)a->size[0];
    i4 = av->size[0];
    av->size[0] = (int)a_idx_0;
    emxEnsureCapacity((emxArray__common *)av, i4, (int)sizeof(double));
    a_idx_0 = (unsigned int)b->size[0];
    i4 = bv->size[0];
    bv->size[0] = (int)a_idx_0;
    emxEnsureCapacity((emxArray__common *)bv, i4, (int)sizeof(double));
    bsub = 1;
    bk = 0;
    nc1 = c->size[0];
    i4 = c->size[0] * c->size[1] - c->size[0];
    ck = 0;
    emxInit_real_T2(&cv, 1);
    while (ck <= i4) {
      for (b_c = 0; b_c + 1 <= a->size[0]; b_c++) {
        av->data[b_c] = a->data[b_c];
      }

      for (b_c = 0; b_c + 1 <= nb1; b_c++) {
        bv->data[b_c] = b->data[bk + b_c];
      }

      b_c = cv->size[0];
      cv->size[0] = av->size[0];
      emxEnsureCapacity((emxArray__common *)cv, b_c, (int)sizeof(double));
      csz_idx_1 = av->size[0];
      for (b_c = 0; b_c < csz_idx_1; b_c++) {
        cv->data[b_c] = av->data[b_c] - bv->data[b_c];
      }

      for (b_c = 0; b_c + 1 <= nc1; b_c++) {
        c->data[ck + b_c] = cv->data[b_c];
      }

      if (bsub < b->size[1]) {
        bk += nb1;
        bsub++;
      } else {
        bsub = 1;
      }

      ck += nc1;
    }

    emxFree_real_T(&cv);
    emxFree_real_T(&bv);
    emxFree_real_T(&av);
  }
}

/*
 * Arguments    : const emxArray_int32_T *a
 *                const emxArray_int32_T *b
 *                emxArray_int32_T *c
 * Return Type  : void
 */
void bsxfun(const emxArray_int32_T *a, const emxArray_int32_T *b,
            emxArray_int32_T *c)
{
  int csz_idx_0;
  int csz_idx_1;
  int i3;
  emxArray_int32_T *av;
  unsigned int a_idx_0;
  int bsub;
  int bk;
  int nc1;
  int ck;
  emxArray_int32_T *cv;
  int loop_ub;
  csz_idx_0 = a->size[0];
  csz_idx_1 = b->size[1];
  i3 = c->size[0] * c->size[1];
  c->size[0] = csz_idx_0;
  c->size[1] = csz_idx_1;
  emxEnsureCapacity((emxArray__common *)c, i3, (int)sizeof(int));
  if (!((c->size[0] == 0) || (c->size[1] == 0))) {
    emxInit_int32_T(&av, 1);
    a_idx_0 = (unsigned int)a->size[0];
    i3 = av->size[0];
    av->size[0] = (int)a_idx_0;
    emxEnsureCapacity((emxArray__common *)av, i3, (int)sizeof(int));
    bsub = 1;
    bk = 0;
    nc1 = c->size[0];
    i3 = c->size[0] * c->size[1] - c->size[0];
    ck = 0;
    emxInit_int32_T(&cv, 1);
    while (ck <= i3) {
      for (csz_idx_0 = 0; csz_idx_0 + 1 <= a->size[0]; csz_idx_0++) {
        av->data[csz_idx_0] = a->data[csz_idx_0];
      }

      csz_idx_0 = b->data[bk];
      csz_idx_1 = cv->size[0];
      cv->size[0] = av->size[0];
      emxEnsureCapacity((emxArray__common *)cv, csz_idx_1, (int)sizeof(int));
      loop_ub = av->size[0];
      for (csz_idx_1 = 0; csz_idx_1 < loop_ub; csz_idx_1++) {
        cv->data[csz_idx_1] = av->data[csz_idx_1] + csz_idx_0;
      }

      for (csz_idx_0 = 0; csz_idx_0 + 1 <= nc1; csz_idx_0++) {
        c->data[ck + csz_idx_0] = cv->data[csz_idx_0];
      }

      if (bsub < b->size[1]) {
        bk++;
        bsub++;
      } else {
        bsub = 1;
      }

      ck += nc1;
    }

    emxFree_int32_T(&cv);
    emxFree_int32_T(&av);
  }
}

/*
 * File trailer for bsxfun.c
 *
 * [EOF]
 */
