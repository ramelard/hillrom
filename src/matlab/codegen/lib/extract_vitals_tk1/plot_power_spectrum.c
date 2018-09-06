/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: plot_power_spectrum.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 06-Sep-2018 14:56:40
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "extract_vitals_tk1.h"
#include "plot_power_spectrum.h"
#include "extract_vitals_tk1_emxutil.h"
#include "fft.h"

/* Function Definitions */

/*
 * Arguments    : const emxArray_real_T *y
 *                double sampling_rate
 *                emxArray_real_T *freq
 *                emxArray_creal_T *power
 * Return Type  : void
 */
void plot_power_spectrum(const emxArray_real_T *y, double sampling_rate,
  emxArray_real_T *freq, emxArray_creal_T *power)
{
  emxArray_creal_T *Y;
  int b_y[1];
  int y_idx_0;
  emxArray_real_T c_y;
  int i11;
  int loop_ub;
  double d_y;
  double Y_im;
  double Y_re;
  emxInit_creal_T1(&Y, 1);

  /* PLOT_POWER_SPECTRUM Plot frequency power spectrum */
  /*  */
  /*  [freq,fpower] = plot_power_spectrum(y, sampling_rate) */
  /*  */
  /*  From http://www.mathworks.com/products/matlab/examples.html?file=/products/demos/shipping/matlab/fftdemo.html */
  /*  y - signal (columns are observations) */
  /*  if size(y,2) > size(y,1) */
  /*    warning(sprintf('Are you sure columns are observations? (r%u,c%u)', size(y,1), size(y,2))) */
  /*  end */
  /*  npts = numel(y); */
  b_y[0] = y->size[1];
  y_idx_0 = y->size[1];
  c_y = *y;
  c_y.size = (int *)&b_y;
  c_y.numDimensions = 1;
  c_fft(&c_y, y_idx_0, Y);

  /*  power = (a^2+b^2)/N */
  /*  Should be same as 1/N*abs(Y)^2 */
  y_idx_0 = y->size[1];
  i11 = Y->size[0];
  emxEnsureCapacity((emxArray__common *)Y, i11, (int)sizeof(creal_T));
  loop_ub = Y->size[0];
  for (i11 = 0; i11 < loop_ub; i11++) {
    d_y = Y->data[i11].re;
    Y_im = -Y->data[i11].im;
    Y_re = Y->data[i11].re * d_y - Y->data[i11].im * Y_im;
    Y_im = Y->data[i11].re * Y_im + Y->data[i11].im * d_y;
    if (Y_im == 0.0) {
      Y->data[i11].re = Y_re / (double)y_idx_0;
      Y->data[i11].im = 0.0;
    } else if (Y_re == 0.0) {
      Y->data[i11].re = 0.0;
      Y->data[i11].im = Y_im / (double)y_idx_0;
    } else {
      Y->data[i11].re = Y_re / (double)y_idx_0;
      Y->data[i11].im = Y_im / (double)y_idx_0;
    }
  }

  y_idx_0 = y->size[1];
  d_y = sampling_rate / (double)y_idx_0;
  y_idx_0 = y->size[1];
  y_idx_0 = (int)floor((double)y_idx_0 / 2.0);
  if (y_idx_0 - 1 < 0) {
    i11 = freq->size[0] * freq->size[1];
    freq->size[0] = 1;
    freq->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)freq, i11, (int)sizeof(double));
  } else {
    i11 = freq->size[0] * freq->size[1];
    freq->size[0] = 1;
    freq->size[1] = y_idx_0;
    emxEnsureCapacity((emxArray__common *)freq, i11, (int)sizeof(double));
    loop_ub = y_idx_0 - 1;
    for (i11 = 0; i11 <= loop_ub; i11++) {
      freq->data[freq->size[0] * i11] = i11;
    }
  }

  /*  nth bin is n*Fs/N, +1 for index offset */
  i11 = freq->size[0] * freq->size[1];
  freq->size[0] = 1;
  emxEnsureCapacity((emxArray__common *)freq, i11, (int)sizeof(double));
  y_idx_0 = freq->size[0];
  loop_ub = freq->size[1];
  loop_ub *= y_idx_0;
  for (i11 = 0; i11 < loop_ub; i11++) {
    freq->data[i11] *= d_y;
  }

  y_idx_0 = y->size[1];
  i11 = (int)floor((double)y_idx_0 / 2.0);
  if (1 > i11) {
    loop_ub = 0;
  } else {
    loop_ub = i11;
  }

  i11 = power->size[0];
  power->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)power, i11, (int)sizeof(creal_T));
  for (i11 = 0; i11 < loop_ub; i11++) {
    power->data[i11] = Y->data[i11];
  }

  emxFree_creal_T(&Y);
}

/*
 * File trailer for plot_power_spectrum.c
 *
 * [EOF]
 */
