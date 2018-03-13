/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: get_spectral_entropy.h
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 01-Mar-2018 16:31:19
 */

#ifndef GET_SPECTRAL_ENTROPY_H
#define GET_SPECTRAL_ENTROPY_H

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

  extern void get_spectral_entropy(emxArray_creal_T *power_range,
    emxArray_creal_T *entr);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for get_spectral_entropy.h
 *
 * [EOF]
 */
