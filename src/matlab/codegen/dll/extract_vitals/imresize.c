/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: imresize.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 01-Mar-2018 16:31:19
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "extract_vitals.h"
#include "imresize.h"
#include "extract_vitals_emxutil.h"
#include "bsxfun.h"
#include "sort1.h"

/* Function Declarations */
static void b_resizeAlongDim(const emxArray_real_T *in_, double dim, const
  emxArray_real_T *weights, const emxArray_int32_T *indices, emxArray_real_T
  *out);
static void contributions(int in_length, double out_length, double scale,
  emxArray_real_T *weights, emxArray_int32_T *indices);
static void resizeAlongDim(const emxArray_real_T *in_, double dim, const
  emxArray_real_T *weights, const emxArray_int32_T *indices, emxArray_real_T
  *out);

/* Function Definitions */

/*
 * Arguments    : const emxArray_real_T *in_
 *                double dim
 *                const emxArray_real_T *weights
 *                const emxArray_int32_T *indices
 *                emxArray_real_T *out
 * Return Type  : void
 */
static void b_resizeAlongDim(const emxArray_real_T *in_, double dim, const
  emxArray_real_T *weights, const emxArray_int32_T *indices, emxArray_real_T
  *out)
{
  int out_length;
  unsigned int outSize[3];
  int idx;
  int pInd;
  double d2;
  int outCInd;
  int inRInd;
  int c;
  int siz[3];
  double sumVal1;
  int linearInds;
  int k;
  out_length = weights->size[1] - 1;
  for (idx = 0; idx < 3; idx++) {
    outSize[idx] = (unsigned int)in_->size[idx];
  }

  outSize[(int)dim - 1] = (unsigned int)weights->size[1];
  idx = out->size[0] * out->size[1] * out->size[2];
  out->size[0] = (int)outSize[0];
  out->size[1] = (int)outSize[1];
  out->size[2] = 3;
  emxEnsureCapacity((emxArray__common *)out, idx, (int)sizeof(double));
  if (dim == 1.0) {
    d2 = (double)(in_->size[0] * in_->size[1] * 3) / (double)in_->size[0];
    for (outCInd = 0; outCInd < (int)d2; outCInd++) {
      for (c = 0; c <= out_length; c++) {
        sumVal1 = 0.0;
        linearInds = weights->size[0] * c;

        /*  Core - first dimension */
        for (k = 0; k < weights->size[0]; k++) {
          idx = in_->size[0];
          sumVal1 += weights->data[linearInds] * in_->data[(indices->
            data[linearInds] + idx * outCInd) - 1];
          linearInds++;
        }

        idx = out->size[0];
        out->data[c + idx * outCInd] = sumVal1;
      }
    }
  } else {
    for (pInd = 0; pInd < 3; pInd++) {
      for (inRInd = 0; inRInd < in_->size[0]; inRInd++) {
        for (idx = 0; idx < 3; idx++) {
          siz[idx] = in_->size[idx];
        }

        idx = inRInd + siz[0] * siz[1] * pInd;
        for (outCInd = 0; outCInd <= out_length; outCInd++) {
          sumVal1 = 0.0;

          /*  Core - second dimension */
          linearInds = weights->size[0] * outCInd;
          for (k = 0; k < weights->size[0]; k++) {
            c = (indices->data[linearInds] - 1) * in_->size[0];
            sumVal1 += weights->data[linearInds] * in_->data[idx + c];
            linearInds++;
          }

          out->data[(inRInd + out->size[0] * outCInd) + out->size[0] * out->
            size[1] * pInd] = sumVal1;
        }
      }
    }
  }
}

/*
 * Arguments    : int in_length
 *                double out_length
 *                double scale
 *                emxArray_real_T *weights
 *                emxArray_int32_T *indices
 * Return Type  : void
 */
static void contributions(int in_length, double out_length, double scale,
  emxArray_real_T *weights, emxArray_int32_T *indices)
{
  double kernel_width;
  emxArray_real_T *y;
  int i5;
  int loop_ub;
  emxArray_real_T *u;
  double s;
  emxArray_real_T *av;
  emxArray_real_T *bv;
  int P;
  int n;
  emxArray_int32_T *aux;
  emxArray_int32_T *b_bv;
  emxArray_real_T *b_indices;
  emxArray_real_T *a;
  int na1;
  unsigned int u_idx_0;
  int nc1;
  int ck;
  emxArray_real_T *cv;
  unsigned int outsize[2];
  int i6;
  emxArray_boolean_T *copyCols;
  boolean_T exitg1;
  boolean_T b0;
  emxArray_real_T *b_weights;
  emxArray_int32_T *c_indices;

  /*  Contributions, using pixel indices */
  if (scale < 1.0) {
    kernel_width = 1.0 / scale;
  } else {
    kernel_width = 1.0;
  }

  emxInit_real_T(&y, 2);
  if (rtIsNaN(out_length)) {
    i5 = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = 1;
    emxEnsureCapacity((emxArray__common *)y, i5, (int)sizeof(double));
    y->data[0] = rtNaN;
  } else if (out_length < 1.0) {
    i5 = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)y, i5, (int)sizeof(double));
  } else if (rtIsInf(out_length) && (1.0 == out_length)) {
    i5 = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = 1;
    emxEnsureCapacity((emxArray__common *)y, i5, (int)sizeof(double));
    y->data[0] = rtNaN;
  } else {
    i5 = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = (int)floor(out_length - 1.0) + 1;
    emxEnsureCapacity((emxArray__common *)y, i5, (int)sizeof(double));
    loop_ub = (int)floor(out_length - 1.0);
    for (i5 = 0; i5 <= loop_ub; i5++) {
      y->data[y->size[0] * i5] = 1.0 + (double)i5;
    }
  }

  emxInit_real_T2(&u, 1);
  s = 0.5 * (1.0 - 1.0 / scale);
  i5 = u->size[0];
  u->size[0] = y->size[1];
  emxEnsureCapacity((emxArray__common *)u, i5, (int)sizeof(double));
  loop_ub = y->size[1];
  for (i5 = 0; i5 < loop_ub; i5++) {
    u->data[i5] = y->data[y->size[0] * i5] / scale + s;
  }

  emxFree_real_T(&y);
  emxInit_real_T2(&av, 1);
  s = kernel_width / 2.0;
  i5 = av->size[0];
  av->size[0] = u->size[0];
  emxEnsureCapacity((emxArray__common *)av, i5, (int)sizeof(double));
  loop_ub = u->size[0];
  for (i5 = 0; i5 < loop_ub; i5++) {
    av->data[i5] = u->data[i5] - s;
  }

  emxInit_real_T2(&bv, 1);
  i5 = bv->size[0];
  bv->size[0] = av->size[0];
  emxEnsureCapacity((emxArray__common *)bv, i5, (int)sizeof(double));
  loop_ub = av->size[0];
  for (i5 = 0; i5 < loop_ub; i5++) {
    bv->data[i5] = av->data[i5];
  }

  for (loop_ub = 0; loop_ub + 1 <= av->size[0]; loop_ub++) {
    bv->data[loop_ub] = floor(bv->data[loop_ub]);
  }

  P = (int)(ceil(kernel_width) + 2.0);
  if (P - 1 < 0) {
    n = 0;
  } else {
    n = P;
  }

  emxInit_int32_T(&aux, 2);
  i5 = aux->size[0] * aux->size[1];
  aux->size[0] = 1;
  aux->size[1] = n;
  emxEnsureCapacity((emxArray__common *)aux, i5, (int)sizeof(int));
  if (n > 0) {
    aux->data[0] = 0;
    P = 0;
    for (loop_ub = 2; loop_ub <= n; loop_ub++) {
      P++;
      aux->data[loop_ub - 1] = P;
    }
  }

  emxInit_int32_T1(&b_bv, 1);
  i5 = b_bv->size[0];
  b_bv->size[0] = bv->size[0];
  emxEnsureCapacity((emxArray__common *)b_bv, i5, (int)sizeof(int));
  loop_ub = bv->size[0];
  for (i5 = 0; i5 < loop_ub; i5++) {
    b_bv->data[i5] = (int)bv->data[i5];
  }

  emxInit_real_T(&b_indices, 2);
  bsxfun(b_bv, aux, indices);
  i5 = b_indices->size[0] * b_indices->size[1];
  b_indices->size[0] = indices->size[0];
  b_indices->size[1] = indices->size[1];
  emxEnsureCapacity((emxArray__common *)b_indices, i5, (int)sizeof(double));
  loop_ub = indices->size[0] * indices->size[1];
  emxFree_int32_T(&b_bv);
  for (i5 = 0; i5 < loop_ub; i5++) {
    b_indices->data[i5] = indices->data[i5];
  }

  b_bsxfun(u, b_indices, weights);
  emxFree_real_T(&b_indices);
  if (scale < 1.0) {
    i5 = weights->size[0] * weights->size[1];
    emxEnsureCapacity((emxArray__common *)weights, i5, (int)sizeof(double));
    n = weights->size[0];
    P = weights->size[1];
    loop_ub = n * P;
    for (i5 = 0; i5 < loop_ub; i5++) {
      weights->data[i5] *= scale;
    }
  }

  i5 = weights->size[0] * weights->size[1];
  emxEnsureCapacity((emxArray__common *)weights, i5, (int)sizeof(double));
  n = weights->size[0];
  P = weights->size[1];
  loop_ub = n * P;
  for (i5 = 0; i5 < loop_ub; i5++) {
    weights->data[i5] = ((-0.5 <= weights->data[i5]) && (weights->data[i5] < 0.5));
  }

  if (scale < 1.0) {
    i5 = weights->size[0] * weights->size[1];
    emxEnsureCapacity((emxArray__common *)weights, i5, (int)sizeof(double));
    n = weights->size[0];
    P = weights->size[1];
    loop_ub = n * P;
    for (i5 = 0; i5 < loop_ub; i5++) {
      weights->data[i5] *= scale;
    }
  }

  i5 = u->size[0];
  u->size[0] = weights->size[0];
  emxEnsureCapacity((emxArray__common *)u, i5, (int)sizeof(double));
  if ((weights->size[0] == 0) || (weights->size[1] == 0)) {
    P = u->size[0];
    i5 = u->size[0];
    u->size[0] = P;
    emxEnsureCapacity((emxArray__common *)u, i5, (int)sizeof(double));
    for (i5 = 0; i5 < P; i5++) {
      u->data[i5] = 0.0;
    }
  } else {
    P = weights->size[0];
    for (n = 0; n + 1 <= P; n++) {
      s = weights->data[n];
      for (loop_ub = 2; loop_ub <= weights->size[1]; loop_ub++) {
        s += weights->data[n + (loop_ub - 1) * P];
      }

      u->data[n] = s;
    }
  }

  emxInit_real_T(&a, 2);
  i5 = a->size[0] * a->size[1];
  a->size[0] = weights->size[0];
  a->size[1] = weights->size[1];
  emxEnsureCapacity((emxArray__common *)a, i5, (int)sizeof(double));
  loop_ub = weights->size[0] * weights->size[1];
  for (i5 = 0; i5 < loop_ub; i5++) {
    a->data[i5] = weights->data[i5];
  }

  na1 = weights->size[0];
  if (weights->size[0] <= u->size[0]) {
    P = weights->size[0];
  } else {
    P = u->size[0];
  }

  n = weights->size[1];
  i5 = weights->size[0] * weights->size[1];
  weights->size[0] = P;
  weights->size[1] = n;
  emxEnsureCapacity((emxArray__common *)weights, i5, (int)sizeof(double));
  if (!((weights->size[0] == 0) || (weights->size[1] == 0))) {
    i5 = av->size[0];
    av->size[0] = na1;
    emxEnsureCapacity((emxArray__common *)av, i5, (int)sizeof(double));
    u_idx_0 = (unsigned int)u->size[0];
    i5 = bv->size[0];
    bv->size[0] = (int)u_idx_0;
    emxEnsureCapacity((emxArray__common *)bv, i5, (int)sizeof(double));
    P = 1;
    n = 0;
    nc1 = weights->size[0];
    i5 = weights->size[0] * weights->size[1] - weights->size[0];
    ck = 0;
    emxInit_real_T2(&cv, 1);
    while (ck <= i5) {
      for (loop_ub = 0; loop_ub + 1 <= na1; loop_ub++) {
        av->data[loop_ub] = a->data[n + loop_ub];
      }

      for (loop_ub = 0; loop_ub + 1 <= u->size[0]; loop_ub++) {
        bv->data[loop_ub] = u->data[loop_ub];
      }

      i6 = cv->size[0];
      cv->size[0] = av->size[0];
      emxEnsureCapacity((emxArray__common *)cv, i6, (int)sizeof(double));
      loop_ub = av->size[0];
      for (i6 = 0; i6 < loop_ub; i6++) {
        cv->data[i6] = av->data[i6] / bv->data[i6];
      }

      for (loop_ub = 0; loop_ub + 1 <= nc1; loop_ub++) {
        weights->data[ck + loop_ub] = cv->data[loop_ub];
      }

      if (P < a->size[1]) {
        n += na1;
        P++;
      } else {
        P = 1;
      }

      ck += nc1;
    }

    emxFree_real_T(&cv);
  }

  emxFree_real_T(&bv);
  emxFree_real_T(&av);
  emxFree_real_T(&a);
  emxFree_real_T(&u);

  /*  Create the auxiliary matrix: */
  P = in_length << 1;
  i5 = aux->size[0] * aux->size[1];
  aux->size[0] = 1;
  aux->size[1] = P;
  emxEnsureCapacity((emxArray__common *)aux, i5, (int)sizeof(int));
  aux->data[0] = 1;
  aux->data[in_length] = in_length;
  for (ck = 1; ck + 1 <= in_length; ck++) {
    aux->data[ck] = aux->data[ck - 1] + 1;
    aux->data[in_length + ck] = aux->data[(in_length + ck) - 1] - 1;
  }

  /*  Mirror the out-of-bounds indices using mod: */
  i5 = indices->size[0] * indices->size[1];
  for (ck = 0; ck < i5; ck++) {
    if (P == 0) {
      s = (double)indices->data[ck] - 1.0;
    } else {
      s = ((double)indices->data[ck] - 1.0) - floor(((double)indices->data[ck] -
        1.0) / (double)P) * (double)P;
    }

    indices->data[ck] = aux->data[(int)s];
  }

  for (i5 = 0; i5 < 2; i5++) {
    outsize[i5] = (unsigned int)weights->size[i5];
  }

  emxInit_boolean_T(&copyCols, 2);
  i5 = copyCols->size[0] * copyCols->size[1];
  copyCols->size[0] = 1;
  copyCols->size[1] = (int)outsize[1];
  emxEnsureCapacity((emxArray__common *)copyCols, i5, (int)sizeof(boolean_T));
  loop_ub = (int)outsize[1];
  for (i5 = 0; i5 < loop_ub; i5++) {
    copyCols->data[i5] = false;
  }

  P = 0;
  n = -1;
  for (ck = 1; ck <= weights->size[1]; ck++) {
    nc1 = P;
    P += weights->size[0];
    n++;
    exitg1 = false;
    while ((!exitg1) && (nc1 + 1 <= P)) {
      if ((weights->data[nc1] == 0.0) || rtIsNaN(weights->data[nc1])) {
        b0 = true;
      } else {
        b0 = false;
      }

      if (!b0) {
        copyCols->data[n] = true;
        exitg1 = true;
      } else {
        nc1++;
      }
    }
  }

  n = copyCols->size[1] - 1;
  P = 0;
  for (ck = 0; ck <= n; ck++) {
    if (copyCols->data[ck]) {
      P++;
    }
  }

  i5 = aux->size[0] * aux->size[1];
  aux->size[0] = 1;
  aux->size[1] = P;
  emxEnsureCapacity((emxArray__common *)aux, i5, (int)sizeof(int));
  P = 0;
  for (ck = 0; ck <= n; ck++) {
    if (copyCols->data[ck]) {
      aux->data[P] = ck + 1;
      P++;
    }
  }

  emxInit_real_T(&b_weights, 2);
  n = weights->size[0];
  i5 = b_weights->size[0] * b_weights->size[1];
  b_weights->size[0] = aux->size[1];
  b_weights->size[1] = n;
  emxEnsureCapacity((emxArray__common *)b_weights, i5, (int)sizeof(double));
  for (i5 = 0; i5 < n; i5++) {
    loop_ub = aux->size[1];
    for (i6 = 0; i6 < loop_ub; i6++) {
      b_weights->data[i6 + b_weights->size[0] * i5] = weights->data[i5 +
        weights->size[0] * (aux->data[aux->size[0] * i6] - 1)];
    }
  }

  i5 = weights->size[0] * weights->size[1];
  weights->size[0] = b_weights->size[0];
  weights->size[1] = b_weights->size[1];
  emxEnsureCapacity((emxArray__common *)weights, i5, (int)sizeof(double));
  loop_ub = b_weights->size[1];
  for (i5 = 0; i5 < loop_ub; i5++) {
    P = b_weights->size[0];
    for (i6 = 0; i6 < P; i6++) {
      weights->data[i6 + weights->size[0] * i5] = b_weights->data[i6 +
        b_weights->size[0] * i5];
    }
  }

  emxFree_real_T(&b_weights);
  n = copyCols->size[1] - 1;
  P = 0;
  for (ck = 0; ck <= n; ck++) {
    if (copyCols->data[ck]) {
      P++;
    }
  }

  i5 = aux->size[0] * aux->size[1];
  aux->size[0] = 1;
  aux->size[1] = P;
  emxEnsureCapacity((emxArray__common *)aux, i5, (int)sizeof(int));
  P = 0;
  for (ck = 0; ck <= n; ck++) {
    if (copyCols->data[ck]) {
      aux->data[P] = ck + 1;
      P++;
    }
  }

  emxFree_boolean_T(&copyCols);
  emxInit_int32_T(&c_indices, 2);
  P = indices->size[0];
  i5 = c_indices->size[0] * c_indices->size[1];
  c_indices->size[0] = aux->size[1];
  c_indices->size[1] = P;
  emxEnsureCapacity((emxArray__common *)c_indices, i5, (int)sizeof(int));
  for (i5 = 0; i5 < P; i5++) {
    loop_ub = aux->size[1];
    for (i6 = 0; i6 < loop_ub; i6++) {
      c_indices->data[i6 + c_indices->size[0] * i5] = indices->data[i5 +
        indices->size[0] * (aux->data[aux->size[0] * i6] - 1)];
    }
  }

  emxFree_int32_T(&aux);
  i5 = indices->size[0] * indices->size[1];
  indices->size[0] = c_indices->size[0];
  indices->size[1] = c_indices->size[1];
  emxEnsureCapacity((emxArray__common *)indices, i5, (int)sizeof(int));
  loop_ub = c_indices->size[1];
  for (i5 = 0; i5 < loop_ub; i5++) {
    P = c_indices->size[0];
    for (i6 = 0; i6 < P; i6++) {
      indices->data[i6 + indices->size[0] * i5] = c_indices->data[i6 +
        c_indices->size[0] * i5];
    }
  }

  emxFree_int32_T(&c_indices);
}

/*
 * Arguments    : const emxArray_real_T *in_
 *                double dim
 *                const emxArray_real_T *weights
 *                const emxArray_int32_T *indices
 *                emxArray_real_T *out
 * Return Type  : void
 */
static void resizeAlongDim(const emxArray_real_T *in_, double dim, const
  emxArray_real_T *weights, const emxArray_int32_T *indices, emxArray_real_T
  *out)
{
  int out_length;
  unsigned int outSize[3];
  int idx;
  int pInd;
  double d1;
  int outCInd;
  int inRInd;
  int c;
  int siz[3];
  double sumVal1;
  int linearInds;
  int k;
  out_length = weights->size[1] - 1;
  for (idx = 0; idx < 3; idx++) {
    outSize[idx] = (unsigned int)in_->size[idx];
  }

  outSize[(int)dim - 1] = (unsigned int)weights->size[1];
  idx = out->size[0] * out->size[1] * out->size[2];
  out->size[0] = (int)outSize[0];
  out->size[1] = (int)outSize[1];
  out->size[2] = 3;
  emxEnsureCapacity((emxArray__common *)out, idx, (int)sizeof(double));
  if (dim == 1.0) {
    d1 = (double)(in_->size[0] * in_->size[1] * 3) / (double)in_->size[0];
    for (outCInd = 0; outCInd < (int)d1; outCInd++) {
      for (c = 0; c <= out_length; c++) {
        sumVal1 = 0.0;
        linearInds = weights->size[0] * c;

        /*  Core - first dimension */
        for (k = 0; k < weights->size[0]; k++) {
          idx = in_->size[0];
          sumVal1 += weights->data[linearInds] * in_->data[(indices->
            data[linearInds] + idx * outCInd) - 1];
          linearInds++;
        }

        idx = out->size[0];
        out->data[c + idx * outCInd] = sumVal1;
      }
    }
  } else {
    for (pInd = 0; pInd < 3; pInd++) {
      for (inRInd = 0; inRInd < in_->size[0]; inRInd++) {
        for (idx = 0; idx < 3; idx++) {
          siz[idx] = in_->size[idx];
        }

        idx = inRInd + siz[0] * siz[1] * pInd;
        for (outCInd = 0; outCInd <= out_length; outCInd++) {
          sumVal1 = 0.0;

          /*  Core - second dimension */
          linearInds = weights->size[0] * outCInd;
          for (k = 0; k < weights->size[0]; k++) {
            c = (indices->data[linearInds] - 1) * in_->size[0];
            sumVal1 += weights->data[linearInds] * in_->data[idx + c];
            linearInds++;
          }

          out->data[(inRInd + out->size[0] * outCInd) + out->size[0] * out->
            size[1] * pInd] = sumVal1;
        }
      }
    }
  }
}

/*
 * Arguments    : const emxArray_real_T *Ain
 *                double varargin_1
 *                emxArray_real_T *Bout
 * Return Type  : void
 */
void imresize(const emxArray_real_T *Ain, double varargin_1, emxArray_real_T
              *Bout)
{
  double scale[2];
  int k;
  double scaleOrSize[2];
  double outputSize[2];
  int iidx[2];
  emxArray_real_T *APartialResize;
  emxArray_real_T *weights;
  emxArray_int32_T *indices;
  scale[0] = varargin_1;
  scale[1] = varargin_1;
  for (k = 0; k < 2; k++) {
    outputSize[k] = (double)(unsigned int)Ain->size[k] * scale[k];
  }

  for (k = 0; k < 2; k++) {
    scaleOrSize[k] = scale[k];
    outputSize[k] = ceil(outputSize[k]);
  }

  sort(scaleOrSize, iidx);
  for (k = 0; k < 2; k++) {
    scaleOrSize[k] = iidx[k];
  }

  emxInit_real_T1(&APartialResize, 3);
  emxInit_real_T(&weights, 2);
  emxInit_int32_T(&indices, 2);

  /*  Resize first dimension */
  contributions(Ain->size[(int)scaleOrSize[0] - 1], outputSize[(int)scaleOrSize
                [0] - 1], scale[(int)scaleOrSize[0] - 1], weights, indices);
  resizeAlongDim(Ain, scaleOrSize[0], weights, indices, APartialResize);

  /*  Resize second dimension */
  contributions(Ain->size[(int)scaleOrSize[1] - 1], outputSize[(int)scaleOrSize
                [1] - 1], scale[(int)scaleOrSize[1] - 1], weights, indices);
  b_resizeAlongDim(APartialResize, scaleOrSize[1], weights, indices, Bout);
  emxFree_int32_T(&indices);
  emxFree_real_T(&weights);
  emxFree_real_T(&APartialResize);
}

/*
 * File trailer for imresize.c
 *
 * [EOF]
 */
