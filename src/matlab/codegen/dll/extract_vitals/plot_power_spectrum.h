/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: plot_power_spectrum.h
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 01-Mar-2018 16:31:19
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
#ifdef __cplusplus

extern "C" {

#endif

  extern void b_r2br_r2dit_trig(const emxArray_creal_T *x, int n1_unsigned,
    const emxArray_real_T *costab, const emxArray_real_T *sintab,
    emxArray_creal_T *y);
  extern void get_algo_sizes(int n1, boolean_T useRadix2, int *N2blue, int
    *nRows);
  extern void plot_power_spectrum(const double y_data[], double freq_data[], int
    freq_size[2], creal_T power_data[], int power_size[1]);
  extern void r2br_r2dit_trig(const emxArray_creal_T *x, int n1_unsigned, const
    emxArray_real_T *costab, const emxArray_real_T *sintab, emxArray_creal_T *y);
  extern void r2br_r2dit_trig_impl(const emxArray_creal_T *x, int unsigned_nRows,
    const emxArray_real_T *costab, const emxArray_real_T *sintab,
    emxArray_creal_T *y);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for plot_power_spectrum.h
 *
 * [EOF]
 */
