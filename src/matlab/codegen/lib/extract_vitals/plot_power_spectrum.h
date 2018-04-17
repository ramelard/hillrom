/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: plot_power_spectrum.h
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 16-Apr-2018 17:05:50
 */

#ifndef PLOT_POWER_SPECTRUM_H
#define PLOT_POWER_SPECTRUM_H

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "extract_vitals_types.h"

/* Function Declarations */
extern void plot_power_spectrum(const emxArray_creal_T *y, emxArray_real_T *freq,
  emxArray_creal_T *power);

#endif

/*
 * File trailer for plot_power_spectrum.h
 *
 * [EOF]
 */
