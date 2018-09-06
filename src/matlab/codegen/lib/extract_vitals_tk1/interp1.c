/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: interp1.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 06-Sep-2018 13:47:30
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "extract_vitals_tk1.h"
#include "interp1.h"
#include "extract_vitals_tk1_emxutil.h"

/* Function Definitions */

/*
 * Arguments    : const emxArray_real_T *varargin_1
 *                const emxArray_real_T *varargin_2
 *                const emxArray_real_T *varargin_3
 *                emxArray_real_T *Vq
 * Return Type  : void
 */
void interp1(const emxArray_real_T *varargin_1, const emxArray_real_T
             *varargin_2, const emxArray_real_T *varargin_3, emxArray_real_T *Vq)
{
  emxArray_real_T *y;
  int low_ip1;
  int nd2;
  emxArray_real_T *x;
  int nx;
  unsigned int outsize[2];
  int k;
  int exitg1;
  int mid_i;
  double r;
  emxInit_real_T1(&y, 2);
  low_ip1 = y->size[0] * y->size[1];
  y->size[0] = 1;
  y->size[1] = varargin_2->size[1];
  emxEnsureCapacity((emxArray__common *)y, low_ip1, (int)sizeof(double));
  nd2 = varargin_2->size[0] * varargin_2->size[1];
  for (low_ip1 = 0; low_ip1 < nd2; low_ip1++) {
    y->data[low_ip1] = varargin_2->data[low_ip1];
  }

  emxInit_real_T1(&x, 2);
  low_ip1 = x->size[0] * x->size[1];
  x->size[0] = 1;
  x->size[1] = varargin_1->size[1];
  emxEnsureCapacity((emxArray__common *)x, low_ip1, (int)sizeof(double));
  nd2 = varargin_1->size[0] * varargin_1->size[1];
  for (low_ip1 = 0; low_ip1 < nd2; low_ip1++) {
    x->data[low_ip1] = varargin_1->data[low_ip1];
  }

  nx = varargin_1->size[1];
  for (low_ip1 = 0; low_ip1 < 2; low_ip1++) {
    outsize[low_ip1] = (unsigned int)varargin_3->size[low_ip1];
  }

  low_ip1 = Vq->size[0] * Vq->size[1];
  Vq->size[0] = 1;
  Vq->size[1] = (int)outsize[1];
  emxEnsureCapacity((emxArray__common *)Vq, low_ip1, (int)sizeof(double));
  nd2 = (int)outsize[1];
  for (low_ip1 = 0; low_ip1 < nd2; low_ip1++) {
    Vq->data[low_ip1] = rtNaN;
  }

  if (varargin_3->size[1] != 0) {
    k = 1;
    do {
      exitg1 = 0;
      if (k <= nx) {
        if (rtIsNaN(varargin_1->data[k - 1])) {
          exitg1 = 1;
        } else {
          k++;
        }
      } else {
        if (varargin_1->data[1] < varargin_1->data[0]) {
          low_ip1 = nx >> 1;
          for (mid_i = 1; mid_i <= low_ip1; mid_i++) {
            r = x->data[mid_i - 1];
            x->data[mid_i - 1] = x->data[nx - mid_i];
            x->data[nx - mid_i] = r;
          }

          nd2 = varargin_2->size[1] >> 1;
          for (mid_i = 1; mid_i <= nd2; mid_i++) {
            low_ip1 = varargin_2->size[1] - mid_i;
            r = y->data[mid_i - 1];
            y->data[mid_i - 1] = y->data[low_ip1];
            y->data[low_ip1] = r;
          }
        }

        for (k = 0; k + 1 <= varargin_3->size[1]; k++) {
          r = Vq->data[k];
          if (rtIsNaN(varargin_3->data[k])) {
            r = rtNaN;
          } else {
            if ((!(varargin_3->data[k] > x->data[x->size[1] - 1])) &&
                (!(varargin_3->data[k] < x->data[0]))) {
              nd2 = 1;
              low_ip1 = 2;
              nx = x->size[1];
              while (nx > low_ip1) {
                mid_i = (nd2 >> 1) + (nx >> 1);
                if (((nd2 & 1) == 1) && ((nx & 1) == 1)) {
                  mid_i++;
                }

                if (varargin_3->data[k] >= x->data[mid_i - 1]) {
                  nd2 = mid_i;
                  low_ip1 = mid_i + 1;
                } else {
                  nx = mid_i;
                }
              }

              r = (varargin_3->data[k] - x->data[nd2 - 1]) / (x->data[nd2] -
                x->data[nd2 - 1]);
              if (r == 0.0) {
                r = y->data[nd2 - 1];
              } else if (r == 1.0) {
                r = y->data[nd2];
              } else if (y->data[nd2 - 1] == y->data[nd2]) {
                r = y->data[nd2 - 1];
              } else {
                r = (1.0 - r) * y->data[nd2 - 1] + r * y->data[nd2];
              }
            }
          }

          Vq->data[k] = r;
        }

        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  emxFree_real_T(&x);
  emxFree_real_T(&y);
}

/*
 * File trailer for interp1.c
 *
 * [EOF]
 */
