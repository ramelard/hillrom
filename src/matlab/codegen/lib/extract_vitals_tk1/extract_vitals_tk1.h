/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: extract_vitals_tk1.h
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 08-Aug-2019 11:00:09
 */

#ifndef EXTRACT_VITALS_TK1_H
#define EXTRACT_VITALS_TK1_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "extract_vitals_tk1_types.h"

/* Function Declarations */
extern void extract_vitals_tk1(const emxArray_real_T *frames_head, const
  emxArray_real_T *frames_body, emxArray_real_T *timestamps, double fps, double
  block_size, double *hr, double *rr);

#endif

/*
 * File trailer for extract_vitals_tk1.h
 *
 * [EOF]
 */
