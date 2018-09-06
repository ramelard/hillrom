/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_extract_vitals_tk1_api.h
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 06-Sep-2018 13:47:30
 */

#ifndef _CODER_EXTRACT_VITALS_TK1_API_H
#define _CODER_EXTRACT_VITALS_TK1_API_H

/* Include Files */
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_extract_vitals_tk1_api.h"

/* Type Definitions */
#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T

struct emxArray_real_T
{
  real_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray_real_T*/

#ifndef typedef_emxArray_real_T
#define typedef_emxArray_real_T

typedef struct emxArray_real_T emxArray_real_T;

#endif                                 /*typedef_emxArray_real_T*/

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void extract_vitals_tk1(emxArray_real_T *frames_head, emxArray_real_T
  *frames_body, emxArray_real_T *timestamps, real_T fps, real_T block_size,
  real_T *hr, real_T *rr);
extern void extract_vitals_tk1_api(const mxArray *prhs[5], const mxArray *plhs[2]);
extern void extract_vitals_tk1_atexit(void);
extern void extract_vitals_tk1_initialize(void);
extern void extract_vitals_tk1_terminate(void);
extern void extract_vitals_tk1_xil_terminate(void);

#endif

/*
 * File trailer for _coder_extract_vitals_tk1_api.h
 *
 * [EOF]
 */
