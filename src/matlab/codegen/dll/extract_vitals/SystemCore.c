/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: SystemCore.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 01-Mar-2018 16:31:19
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "extract_vitals.h"
#include "SystemCore.h"
#include "extract_vitals_emxutil.h"
#include "CascadeObjectDetector.h"
#include "libmwgrayto8.h"
#include "CascadeClassifierCore_api.hpp"

/* Function Declarations */
static double rt_roundd_snf(double u);

/* Function Definitions */

/*
 * Arguments    : double u
 * Return Type  : double
 */
static double rt_roundd_snf(double u)
{
  double y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

/*
 * Arguments    : vision_CascadeObjectDetector *obj
 *                const emxArray_real_T *varargin_1
 *                emxArray_real_T *varargout_1
 * Return Type  : void
 */
void SystemCore_step(vision_CascadeObjectDetector *obj, const emxArray_real_T
                     *varargin_1, emxArray_real_T *varargout_1)
{
  vision_CascadeObjectDetector *b_obj;
  int i3;
  cell_wrap_2 varSizes[1];
  unsigned int inSize[8];
  int num_bboxes;
  boolean_T exitg1;
  emxArray_uint8_T *Iu8;
  emxArray_uint8_T *Iu8_grayT;
  int i4;
  double obj_MinSize[2];
  int loop_ub;
  void * ptrObj;
  double ScaleFactor;
  double d0;
  unsigned int MergeThreshold;
  int MinSize_[2];
  int MaxSize_[2];
  void * ptrDetectedObj;
  emxArray_int32_T *bboxes_;
  if (obj->isInitialized != 1) {
    b_obj = obj;
    b_obj->isInitialized = 1;
    for (i3 = 0; i3 < 2; i3++) {
      varSizes[0].f1[i3] = (unsigned int)varargin_1->size[i3];
    }

    for (i3 = 0; i3 < 6; i3++) {
      varSizes[0].f1[i3 + 2] = 1U;
    }

    b_obj->inputVarSize[0] = varSizes[0];
    c_CascadeObjectDetector_validat(b_obj);
    b_obj->TunablePropsChanged = false;
  }

  b_obj = obj;
  if (b_obj->TunablePropsChanged) {
    c_CascadeObjectDetector_validat(b_obj);
    b_obj->TunablePropsChanged = false;
  }

  b_obj = obj;
  for (i3 = 0; i3 < 2; i3++) {
    inSize[i3] = (unsigned int)varargin_1->size[i3];
  }

  for (i3 = 0; i3 < 6; i3++) {
    inSize[i3 + 2] = 1U;
  }

  num_bboxes = 0;
  exitg1 = false;
  while ((!exitg1) && (num_bboxes < 8)) {
    if (b_obj->inputVarSize[0].f1[num_bboxes] != inSize[num_bboxes]) {
      for (i3 = 0; i3 < 8; i3++) {
        b_obj->inputVarSize[0].f1[i3] = inSize[i3];
      }

      exitg1 = true;
    } else {
      num_bboxes++;
    }
  }

  emxInit_uint8_T(&Iu8, 2);
  b_obj = obj;
  for (i3 = 0; i3 < 2; i3++) {
    i4 = Iu8->size[0] * Iu8->size[1];
    Iu8->size[i3] = varargin_1->size[i3];
    emxEnsureCapacity((emxArray__common *)Iu8, i4, (int)sizeof(unsigned char));
  }

  emxInit_uint8_T(&Iu8_grayT, 2);
  grayto8_real64(&varargin_1->data[0], &Iu8->data[0], (double)(varargin_1->size
    [0] * varargin_1->size[1]));
  i3 = Iu8_grayT->size[0] * Iu8_grayT->size[1];
  Iu8_grayT->size[0] = Iu8->size[1];
  Iu8_grayT->size[1] = Iu8->size[0];
  emxEnsureCapacity((emxArray__common *)Iu8_grayT, i3, (int)sizeof(unsigned char));
  num_bboxes = Iu8->size[0];
  for (i3 = 0; i3 < num_bboxes; i3++) {
    loop_ub = Iu8->size[1];
    for (i4 = 0; i4 < loop_ub; i4++) {
      Iu8_grayT->data[i4 + Iu8_grayT->size[0] * i3] = Iu8->data[i3 + Iu8->size[0]
        * i4];
    }
  }

  for (i3 = 0; i3 < 2; i3++) {
    obj_MinSize[i3] = b_obj->MinSize[i3];
  }

  ptrObj = b_obj->pCascadeClassifier;
  ScaleFactor = b_obj->ScaleFactor;
  d0 = rt_roundd_snf(b_obj->MergeThreshold);
  if (d0 < 4.294967296E+9) {
    if (d0 >= 0.0) {
      MergeThreshold = (unsigned int)d0;
    } else {
      MergeThreshold = 0U;
    }
  } else if (d0 >= 4.294967296E+9) {
    MergeThreshold = MAX_uint32_T;
  } else {
    MergeThreshold = 0U;
  }

  if ((Iu8_grayT->size[0] == 0) || (Iu8_grayT->size[1] == 0)) {
    i3 = varargout_1->size[0] * varargout_1->size[1];
    varargout_1->size[0] = 0;
    varargout_1->size[1] = 4;
    emxEnsureCapacity((emxArray__common *)varargout_1, i3, (int)sizeof(double));
  } else {
    for (i3 = 0; i3 < 2; i3++) {
      d0 = rt_roundd_snf(obj_MinSize[i3]);
      if (d0 < 2.147483648E+9) {
        if (d0 >= -2.147483648E+9) {
          i4 = (int)d0;
        } else {
          i4 = MIN_int32_T;
        }
      } else if (d0 >= 2.147483648E+9) {
        i4 = MAX_int32_T;
      } else {
        i4 = 0;
      }

      MinSize_[i3] = i4;
      MaxSize_[i3] = 0;
    }

    ptrDetectedObj = NULL;
    i3 = Iu8->size[0] * Iu8->size[1];
    Iu8->size[0] = Iu8_grayT->size[0];
    Iu8->size[1] = Iu8_grayT->size[1];
    emxEnsureCapacity((emxArray__common *)Iu8, i3, (int)sizeof(unsigned char));
    num_bboxes = Iu8_grayT->size[0] * Iu8_grayT->size[1];
    for (i3 = 0; i3 < num_bboxes; i3++) {
      Iu8->data[i3] = Iu8_grayT->data[i3];
    }

    emxInit_int32_T(&bboxes_, 2);
    num_bboxes = cascadeClassifier_detectMultiScale(ptrObj, &ptrDetectedObj,
      &Iu8->data[0], Iu8_grayT->size[1], Iu8_grayT->size[0], ScaleFactor,
      MergeThreshold, MinSize_, MaxSize_);
    i3 = bboxes_->size[0] * bboxes_->size[1];
    bboxes_->size[0] = num_bboxes;
    bboxes_->size[1] = 4;
    emxEnsureCapacity((emxArray__common *)bboxes_, i3, (int)sizeof(int));
    cascadeClassifier_assignOutputDeleteBbox(ptrDetectedObj, &bboxes_->data[0]);
    i3 = varargout_1->size[0] * varargout_1->size[1];
    varargout_1->size[0] = bboxes_->size[0];
    varargout_1->size[1] = bboxes_->size[1];
    emxEnsureCapacity((emxArray__common *)varargout_1, i3, (int)sizeof(double));
    num_bboxes = bboxes_->size[0] * bboxes_->size[1];
    for (i3 = 0; i3 < num_bboxes; i3++) {
      varargout_1->data[i3] = bboxes_->data[i3];
    }

    emxFree_int32_T(&bboxes_);
  }

  emxFree_uint8_T(&Iu8_grayT);
  emxFree_uint8_T(&Iu8);
}

/*
 * File trailer for SystemCore.c
 *
 * [EOF]
 */
