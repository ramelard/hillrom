/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: CascadeObjectDetector.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 01-Mar-2018 16:31:19
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "extract_vitals.h"
#include "CascadeObjectDetector.h"
#include "CascadeClassifierCore_api.hpp"

/* Function Declarations */
static void d_CascadeObjectDetector_Cascade(vision_CascadeObjectDetector **obj);

/* Function Definitions */

/*
 * Arguments    : vision_CascadeObjectDetector **obj
 * Return Type  : void
 */
static void d_CascadeObjectDetector_Cascade(vision_CascadeObjectDetector **obj)
{
  vision_CascadeObjectDetector *b_obj;
  void * ptrObj;
  boolean_T flag;
  int i12;
  char ClassificationModel[123];
  static const char b_ClassificationModel[123] = { 'C', ':', '\\', 'P', 'r', 'o',
    'g', 'r', 'a', 'm', ' ', 'F', 'i', 'l', 'e', 's', '\\', 'M', 'A', 'T', 'L',
    'A', 'B', '\\', 'R', '2', '0', '1', '6', 'b', '\\', 't', 'o', 'o', 'l', 'b',
    'o', 'x', '\\', 'v', 'i', 's', 'i', 'o', 'n', '\\', 'v', 'i', 's', 'i', 'o',
    'n', 'u', 't', 'i', 'l', 'i', 't', 'i', 'e', 's', '\\', 'c', 'l', 'a', 's',
    's', 'i', 'f', 'i', 'e', 'r', 'd', 'a', 't', 'a', '\\', 'c', 'a', 's', 'c',
    'a', 'd', 'e', '\\', 'h', 'a', 'a', 'r', '\\', 'h', 'a', 'a', 'r', 'c', 'a',
    's', 'c', 'a', 'd', 'e', '_', 'f', 'r', 'o', 'n', 't', 'a', 'l', 'f', 'a',
    'c', 'e', '_', 'a', 'l', 't', '2', '.', 'x', 'm', 'l', '\x00' };

  (*obj)->ScaleFactor = 1.1;
  (*obj)->MergeThreshold = 4.0;
  b_obj = *obj;
  b_obj->isInitialized = 0;
  ptrObj = NULL;
  cascadeClassifier_construct(&ptrObj);
  (*obj)->pCascadeClassifier = ptrObj;
  b_obj = *obj;
  flag = (b_obj->isInitialized == 1);
  if (flag) {
    b_obj->TunablePropsChanged = true;
  }

  for (i12 = 0; i12 < 2; i12++) {
    b_obj->MinSize[i12] = 75.0;
  }

  b_obj = *obj;
  ptrObj = b_obj->pCascadeClassifier;
  memcpy(&ClassificationModel[0], &b_ClassificationModel[0], 123U * sizeof(char));
  cascadeClassifier_load(ptrObj, ClassificationModel);
  c_CascadeObjectDetector_validat(*obj);
}

/*
 * Arguments    : vision_CascadeObjectDetector *obj
 * Return Type  : vision_CascadeObjectDetector *
 */
vision_CascadeObjectDetector *c_CascadeObjectDetector_Cascade
  (vision_CascadeObjectDetector *obj)
{
  vision_CascadeObjectDetector *b_obj;
  b_obj = obj;
  d_CascadeObjectDetector_Cascade(&b_obj);
  return b_obj;
}

/*
 * Arguments    : vision_CascadeObjectDetector *obj
 * Return Type  : void
 */
void c_CascadeObjectDetector_validat(vision_CascadeObjectDetector *obj)
{
  vision_CascadeObjectDetector *b_obj;
  void * ptrObj;
  unsigned int originalWindowSize[2];
  int k;
  unsigned int unusedU0;
  boolean_T x[2];
  boolean_T y;
  boolean_T exitg1;
  b_obj = obj;
  ptrObj = b_obj->pCascadeClassifier;
  for (k = 0; k < 2; k++) {
    originalWindowSize[k] = 0U;
  }

  unusedU0 = 0U;
  cascadeClassifier_getClassifierInfo(ptrObj, originalWindowSize, &unusedU0);
  for (k = 0; k < 2; k++) {
    x[k] = (obj->MinSize[k] < originalWindowSize[k]);
  }

  y = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 2)) {
    if (!!x[k]) {
      y = true;
      exitg1 = true;
    } else {
      k++;
    }
  }

  if (y) {
    b_obj = obj;
    ptrObj = b_obj->pCascadeClassifier;
    for (k = 0; k < 2; k++) {
      originalWindowSize[k] = 0U;
    }

    unusedU0 = 0U;
    cascadeClassifier_getClassifierInfo(ptrObj, originalWindowSize, &unusedU0);
    b_obj = obj;
    ptrObj = b_obj->pCascadeClassifier;
    for (k = 0; k < 2; k++) {
      originalWindowSize[k] = 0U;
    }

    unusedU0 = 0U;
    cascadeClassifier_getClassifierInfo(ptrObj, originalWindowSize, &unusedU0);
  }
}

/*
 * File trailer for CascadeObjectDetector.c
 *
 * [EOF]
 */
