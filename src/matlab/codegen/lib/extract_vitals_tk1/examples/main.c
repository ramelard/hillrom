/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: main.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 18-Apr-2018 17:29:29
 */

/*************************************************************************/
/* This automatically generated example C main file shows how to call    */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/
/* Include Files */
#include "rt_nonfinite.h"
#include "extract_vitals_tk1.h"
#include "main.h"
#include "extract_vitals_tk1_terminate.h"
#include "extract_vitals_tk1_emxAPI.h"
#include "extract_vitals_tk1_initialize.h"

/* Function Declarations */
static double argInit_real_T(void);
static emxArray_real_T *c_argInit_UnboundedxUnboundedxU(void);
static void main_extract_vitals_tk1(void);

/* Function Definitions */

/*
 * Arguments    : void
 * Return Type  : double
 */
static double argInit_real_T(void)
{
  return 0.0;
}

/*
 * Arguments    : void
 * Return Type  : emxArray_real_T *
 */
static emxArray_real_T *c_argInit_UnboundedxUnboundedxU(void)
{
  emxArray_real_T *result;
  static int iv9[3] = { 2, 2, 2 };

  int idx0;
  int idx1;
  int idx2;

  /* Set the size of the array.
     Change this size to the value that the application requires. */
  result = emxCreateND_real_T(3, iv9);

  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < result->size[0U]; idx0++) {
    for (idx1 = 0; idx1 < result->size[1U]; idx1++) {
      for (idx2 = 0; idx2 < result->size[2U]; idx2++) {
        /* Set the value of the array element.
           Change this value to the value that the application requires. */
        result->data[(idx0 + result->size[0] * idx1) + result->size[0] *
          result->size[1] * idx2] = argInit_real_T();
      }
    }
  }

  return result;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
static void main_extract_vitals_tk1(void)
{
  emxArray_real_T *frames_head;
  emxArray_real_T *frames_body;
  double hr;
  double rr;

  /* Initialize function 'extract_vitals_tk1' input arguments. */
  /* Initialize function input argument 'frames_head'. */
  frames_head = c_argInit_UnboundedxUnboundedxU();

  /* Initialize function input argument 'frames_body'. */
  frames_body = c_argInit_UnboundedxUnboundedxU();

  /* Call the entry-point 'extract_vitals_tk1'. */
  extract_vitals_tk1(frames_head, frames_body, argInit_real_T(), argInit_real_T(),
                     &hr, &rr);
  emxDestroyArray_real_T(frames_body);
  emxDestroyArray_real_T(frames_head);
}

/*
 * Arguments    : int argc
 *                const char * const argv[]
 * Return Type  : int
 */
int main(int argc, const char * const argv[])
{
  (void)argc;
  (void)argv;

  /* Initialize the application.
     You do not need to do this more than one time. */
  extract_vitals_tk1_initialize();

  /* Invoke the entry-point functions.
     You can call entry-point functions multiple times. */
  main_extract_vitals_tk1();

  /* Terminate the application.
     You do not need to do this more than one time. */
  extract_vitals_tk1_terminate();
  return 0;
}

/*
 * File trailer for main.c
 *
 * [EOF]
 */
