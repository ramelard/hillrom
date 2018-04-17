/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: plot_power_spectrum.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 16-Apr-2018 17:05:50
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "extract_vitals.h"
#include "plot_power_spectrum.h"
#include "extract_vitals_emxutil.h"
#include "fft.h"

/* Function Definitions */

/*
 * Arguments    : const emxArray_creal_T *y
 *                emxArray_real_T *freq
 *                emxArray_creal_T *power
 * Return Type  : void
 */
void plot_power_spectrum(const emxArray_creal_T *y, emxArray_real_T *freq,
  emxArray_creal_T *power)
{
  emxArray_creal_T *sigs;
  int i12;
  int loop_ub;
  emxArray_creal_T *Y;
  int sigs_re;
  double b_y;
  double c_y;
  double Y_re;

  /* PLOT_POWER_SPECTRUM Plot frequency power spectrum */
  /*  */
  /*  [freq,fpower] = plot_power_spectrum(y, sampling_rate) */
  /*  */
  /*  From http://www.mathworks.com/products/matlab/examples.html?file=/products/demos/shipping/matlab/fftdemo.html */
  /*  y - signal (columns are observations) */
  emxInit_creal_T1(&sigs, 1);
  if (y->size[0] == 1) {
    i12 = sigs->size[0];
    sigs->size[0] = 1;
    emxEnsureCapacity((emxArray__common *)sigs, i12, (int)sizeof(creal_T));
    sigs->data[0] = y->data[0];
  } else {
    i12 = sigs->size[0];
    sigs->size[0] = y->size[0];
    emxEnsureCapacity((emxArray__common *)sigs, i12, (int)sizeof(creal_T));
    loop_ub = y->size[0];
    for (i12 = 0; i12 < loop_ub; i12++) {
      sigs->data[i12] = y->data[i12];
    }
  }

  emxInit_creal_T1(&Y, 1);

  /*  if size(y,2) > size(y,1) */
  /*    warning(sprintf('Are you sure columns are observations? (r%u,c%u)', size(y,1), size(y,2))) */
  /*  end */
  /*  npts = numel(y); */
  c_fft(sigs, sigs->size[0], Y);

  /*  power = (a^2+b^2)/N */
  /*  Should be same as 1/N*abs(Y)^2 */
  sigs_re = sigs->size[0];
  i12 = Y->size[0];
  emxEnsureCapacity((emxArray__common *)Y, i12, (int)sizeof(creal_T));
  loop_ub = Y->size[0];
  for (i12 = 0; i12 < loop_ub; i12++) {
    b_y = Y->data[i12].re;
    c_y = -Y->data[i12].im;
    Y_re = Y->data[i12].re * b_y - Y->data[i12].im * c_y;
    c_y = Y->data[i12].re * c_y + Y->data[i12].im * b_y;
    if (c_y == 0.0) {
      Y->data[i12].re = Y_re / (double)sigs_re;
      Y->data[i12].im = 0.0;
    } else if (Y_re == 0.0) {
      Y->data[i12].re = 0.0;
      Y->data[i12].im = c_y / (double)sigs_re;
    } else {
      Y->data[i12].re = Y_re / (double)sigs_re;
      Y->data[i12].im = c_y / (double)sigs_re;
    }
  }

  b_y = 60.0 / (double)sigs->size[0];
  c_y = (double)sigs->size[0] / 2.0;
  loop_ub = (int)floor(c_y);
  if (loop_ub - 1 < 0) {
    i12 = freq->size[0] * freq->size[1];
    freq->size[0] = 1;
    freq->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)freq, i12, (int)sizeof(double));
  } else {
    i12 = freq->size[0] * freq->size[1];
    freq->size[0] = 1;
    freq->size[1] = loop_ub;
    emxEnsureCapacity((emxArray__common *)freq, i12, (int)sizeof(double));
    loop_ub--;
    for (i12 = 0; i12 <= loop_ub; i12++) {
      freq->data[freq->size[0] * i12] = i12;
    }
  }

  /*  nth bin is n*Fs/N, +1 for index offset */
  i12 = freq->size[0] * freq->size[1];
  freq->size[0] = 1;
  emxEnsureCapacity((emxArray__common *)freq, i12, (int)sizeof(double));
  loop_ub = freq->size[0];
  sigs_re = freq->size[1];
  loop_ub *= sigs_re;
  for (i12 = 0; i12 < loop_ub; i12++) {
    freq->data[i12] *= b_y;
  }

  i12 = (int)floor((double)sigs->size[0] / 2.0);
  emxFree_creal_T(&sigs);
  if (1 > i12) {
    loop_ub = 0;
  } else {
    loop_ub = i12;
  }

  i12 = power->size[0];
  power->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)power, i12, (int)sizeof(creal_T));
  for (i12 = 0; i12 < loop_ub; i12++) {
    power->data[i12] = Y->data[i12];
  }

  emxFree_creal_T(&Y);
}

/*
 * File trailer for plot_power_spectrum.c
 *
 * [EOF]
 */
