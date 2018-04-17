/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_extract_vitals_api.h
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 16-Apr-2018 17:05:50
 */

#ifndef _CODER_EXTRACT_VITALS_API_H
#define _CODER_EXTRACT_VITALS_API_H

/* Include Files */
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_extract_vitals_api.h"

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
extern void extract_vitals(emxArray_real_T *frames, real_T face_rect[4], real_T
  block_size, real_T *hr, real_T *rr);
extern void extract_vitals_api(const mxArray *prhs[3], const mxArray *plhs[2]);
extern void extract_vitals_atexit(void);
extern void extract_vitals_initialize(void);
extern void extract_vitals_terminate(void);
extern void extract_vitals_xil_terminate(void);

#endif

/*
 * File trailer for _coder_extract_vitals_api.h
 *
 * [EOF]
 */
