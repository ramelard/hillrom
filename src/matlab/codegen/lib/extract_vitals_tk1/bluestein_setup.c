/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: bluestein_setup.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 18-Apr-2018 17:29:29
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "extract_vitals_tk1.h"
#include "bluestein_setup.h"
#include "extract_vitals_tk1_emxutil.h"

/* Function Definitions */

/*
 * Arguments    : int nRows
 *                emxArray_creal_T *wwc
 * Return Type  : void
 */
void bluestein_setup(int nRows, emxArray_creal_T *wwc)
{
  int nInt2m1;
  int idx;
  int rt;
  int nInt2;
  int k;
  int y;
  double nt_im;
  double nt_re;
  nInt2m1 = (nRows + nRows) - 1;
  idx = wwc->size[0];
  wwc->size[0] = nInt2m1;
  emxEnsureCapacity((emxArray__common *)wwc, idx, (int)sizeof(creal_T));
  idx = nRows;
  rt = 0;
  wwc->data[nRows - 1].re = 1.0;
  wwc->data[nRows - 1].im = 0.0;
  nInt2 = nRows << 1;
  for (k = 1; k < nRows; k++) {
    y = (k << 1) - 1;
    if (nInt2 - rt <= y) {
      rt += y - nInt2;
    } else {
      rt += y;
    }

    nt_im = -3.1415926535897931 * (double)rt / (double)nRows;
    if (nt_im == 0.0) {
      nt_re = 1.0;
      nt_im = 0.0;
    } else {
      nt_re = cos(nt_im);
      nt_im = sin(nt_im);
    }

    wwc->data[idx - 2].re = nt_re;
    wwc->data[idx - 2].im = -nt_im;
    idx--;
  }

  idx = 0;
  for (k = nInt2m1 - 1; k >= nRows; k--) {
    wwc->data[k] = wwc->data[idx];
    idx++;
  }
}

/*
 * File trailer for bluestein_setup.c
 *
 * [EOF]
 */
