/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: kalmanfilt.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 16-Apr-2018 17:05:50
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "extract_vitals.h"
#include "kalmanfilt.h"
#include "pinv.h"
#include "extract_vitals_emxutil.h"

/* Function Definitions */

/*
 * Arguments    : const emxArray_real_T *z
 *                emxArray_real_T *xest
 * Return Type  : void
 */
void b_kalmanfilt(const emxArray_real_T *z, emxArray_real_T *xest)
{
  double xprior[2];
  double Pprior[4];
  int ar;
  emxArray_real_T *xpost;
  static const signed char iv6[4] = { 1, 0, 0, 1 };

  int loop_ub;
  int k;
  double b_Pprior[2];
  double d1;
  double b;
  double K[2];
  double I[4];
  double a[4];
  double b_a[4];
  double Ppost_data[4];
  int br;
  static const signed char c_a[4] = { 1, 0, 1, 1 };

  int ic;
  double y_data[4];
  int ib;
  int ia;
  static const double Q[4] = { 1.3225E-10, 2.645E-10, 2.645E-10, 5.29E-10 };

  static const signed char iv7[4] = { 1, 0, 1, 1 };

  static const signed char iv8[4] = { 1, 1, 0, 1 };

  /*  KALMANFILT   Kalman filter */
  /*    xest = kalmanfilt(xhat, sigmap, sigmam) */
  /*      For now, assumes 2D position, velocity setup. */
  /*  For codegen:  */
  /*    codegen -args {coder.typeof(double(0), [1 Inf]) double(0) double(0)} kalmanfilt.m */
  /*  process noise covariance */
  /*  measurement noise variance */
  /*  state transition matrix */
  /*  observation matrix */
  xprior[0] = z->data[0];
  xprior[1] = 0.0;

  /*  x,v initially */
  for (ar = 0; ar < 4; ar++) {
    Pprior[ar] = iv6[ar];
  }

  emxInit_real_T1(&xpost, 2);

  /*  we are certain, thus 0 stddev */
  /*  Posterior estimate of signal x */
  ar = xpost->size[0] * xpost->size[1];
  xpost->size[0] = 2;
  xpost->size[1] = z->size[1];
  emxEnsureCapacity((emxArray__common *)xpost, ar, (int)sizeof(double));
  loop_ub = z->size[1] << 1;
  for (ar = 0; ar < loop_ub; ar++) {
    xpost->data[ar] = 0.0;
  }

  for (k = 0; k < z->size[1]; k++) {
    /*  Correct/Update (supports multiple observations) */
    /*  number of observations */
    d1 = 0.0;
    for (ar = 0; ar < 2; ar++) {
      b_Pprior[ar] = 0.0;
      for (loop_ub = 0; loop_ub < 2; loop_ub++) {
        b_Pprior[ar] += (1.0 - (double)loop_ub) * Pprior[loop_ub + (ar << 1)];
      }

      d1 += b_Pprior[ar] * (1.0 - (double)ar);
    }

    b = pinv(d1 + 1.96E-6);

    /*  Kalman gain */
    d1 = 0.0;
    for (ar = 0; ar < 2; ar++) {
      b_Pprior[ar] = 0.0;
      for (loop_ub = 0; loop_ub < 2; loop_ub++) {
        b_Pprior[ar] += Pprior[ar + (loop_ub << 1)] * (1.0 - (double)loop_ub);
      }

      K[ar] = b_Pprior[ar] * b;
      d1 += (1.0 - (double)ar) * xprior[ar];
    }

    b = z->data[z->size[0] * k] - d1;
    for (ar = 0; ar < 2; ar++) {
      xpost->data[ar + xpost->size[0] * k] = xprior[ar] + K[ar] * b;
    }

    /*  estimate x using msmt data */
    for (ar = 0; ar < 4; ar++) {
      I[ar] = 0.0;
    }

    for (loop_ub = 0; loop_ub < 2; loop_ub++) {
      I[loop_ub + (loop_ub << 1)] = 1.0;
    }

    for (ar = 0; ar < 2; ar++) {
      for (loop_ub = 0; loop_ub < 2; loop_ub++) {
        b_a[ar + (loop_ub << 1)] = I[ar + (loop_ub << 1)] - K[ar] * (1.0 -
          (double)loop_ub);
      }

      for (loop_ub = 0; loop_ub < 2; loop_ub++) {
        I[ar + (loop_ub << 1)] = 0.0;
        a[ar + (loop_ub << 1)] = 0.0;
        for (br = 0; br < 2; br++) {
          I[ar + (loop_ub << 1)] += b_a[ar + (br << 1)] * Pprior[br + (loop_ub <<
            1)];
          a[ar + (loop_ub << 1)] += b_a[ar + (br << 1)] * Pprior[br + (loop_ub <<
            1)];
        }
      }
    }

    for (ar = 0; ar < 2; ar++) {
      for (loop_ub = 0; loop_ub < 2; loop_ub++) {
        Ppost_data[loop_ub + (ar << 1)] = a[loop_ub + (ar << 1)];
      }
    }

    /*  update posterior error covariance */
    /*  Predict */
    for (ar = 0; ar < 2; ar++) {
      d1 = 0.0;
      for (loop_ub = 0; loop_ub < 2; loop_ub++) {
        d1 += (double)c_a[ar + (loop_ub << 1)] * xpost->data[loop_ub +
          xpost->size[0] * k];
      }

      xprior[ar] = d1;
    }

    /*  build prior based on previous position */
    for (ar = 0; ar < 2; ar++) {
      for (loop_ub = 0; loop_ub < 2; loop_ub++) {
        y_data[loop_ub + (ar << 1)] = 0.0;
      }
    }

    for (loop_ub = 0; loop_ub <= 2; loop_ub += 2) {
      for (ic = loop_ub; ic + 1 <= loop_ub + 2; ic++) {
        y_data[ic] = 0.0;
      }
    }

    br = 0;
    for (loop_ub = 0; loop_ub <= 2; loop_ub += 2) {
      ar = -1;
      for (ib = br; ib + 1 <= br + 2; ib++) {
        if (Ppost_data[ib] != 0.0) {
          ia = ar;
          for (ic = loop_ub; ic + 1 <= loop_ub + 2; ic++) {
            ia++;
            y_data[ic] += I[ib] * (double)iv7[ia];
          }
        }

        ar += 2;
      }

      br += 2;
    }

    for (ar = 0; ar < 4; ar++) {
      Pprior[ar] = 0.0;
    }

    for (loop_ub = 0; loop_ub <= 3; loop_ub += 2) {
      for (ic = loop_ub; ic + 1 <= loop_ub + 2; ic++) {
        Pprior[ic] = 0.0;
      }
    }

    br = 0;
    for (loop_ub = 0; loop_ub <= 3; loop_ub += 2) {
      ar = -1;
      for (ib = br; ib + 1 <= br + 2; ib++) {
        if (iv8[ib] != 0) {
          ia = ar;
          for (ic = loop_ub; ic + 1 <= loop_ub + 2; ic++) {
            ia++;
            Pprior[ic] += (double)iv8[ib] * y_data[ia];
          }
        }

        ar += 2;
      }

      br += 2;
    }

    for (ar = 0; ar < 4; ar++) {
      Pprior[ar] += Q[ar];
    }

    /*  build prior error covariance */
  }

  /*  figure, hold on */
  /*  plot(x,'--k') */
  /*  plot(xhat,'-r') */
  /*  plot(z,'-g') */
  /*  plot(xpost(1,:),'b','LineWidth',2) */
  /*  legend('x','xhat','z','xest') */
  loop_ub = xpost->size[1];
  ar = xest->size[0] * xest->size[1];
  xest->size[0] = 1;
  xest->size[1] = loop_ub;
  emxEnsureCapacity((emxArray__common *)xest, ar, (int)sizeof(double));
  for (ar = 0; ar < loop_ub; ar++) {
    xest->data[xest->size[0] * ar] = xpost->data[xpost->size[0] * ar];
  }

  emxFree_real_T(&xpost);
}

/*
 * Arguments    : const emxArray_creal_T *z
 *                emxArray_real_T *xest
 * Return Type  : void
 */
void kalmanfilt(const emxArray_creal_T *z, emxArray_real_T *xest)
{
  creal_T xprior[2];
  double Pprior[4];
  int ar;
  emxArray_real_T *xpost;
  static const signed char iv3[4] = { 1, 0, 0, 1 };

  int loop_ub;
  int k;
  double b_Pprior[2];
  double b;
  double re;
  double K[2];
  double I[4];
  double a[4];
  double b_a[4];
  double Ppost_data[4];
  int br;
  static const signed char c_a[4] = { 1, 0, 1, 1 };

  int ic;
  double y_data[4];
  int ib;
  int ia;
  static const double Q[4] = { 1.3225E-10, 2.645E-10, 2.645E-10, 5.29E-10 };

  static const signed char iv4[4] = { 1, 0, 1, 1 };

  static const signed char iv5[4] = { 1, 1, 0, 1 };

  /*  KALMANFILT   Kalman filter */
  /*    xest = kalmanfilt(xhat, sigmap, sigmam) */
  /*      For now, assumes 2D position, velocity setup. */
  /*  For codegen:  */
  /*    codegen -args {coder.typeof(double(0), [1 Inf]) double(0) double(0)} kalmanfilt.m */
  /*  process noise covariance */
  /*  measurement noise variance */
  /*  state transition matrix */
  /*  observation matrix */
  xprior[0] = z->data[0];
  xprior[1].re = 0.0;
  xprior[1].im = 0.0;

  /*  x,v initially */
  for (ar = 0; ar < 4; ar++) {
    Pprior[ar] = iv3[ar];
  }

  emxInit_real_T1(&xpost, 2);

  /*  we are certain, thus 0 stddev */
  /*  Posterior estimate of signal x */
  ar = xpost->size[0] * xpost->size[1];
  xpost->size[0] = 2;
  xpost->size[1] = z->size[1];
  emxEnsureCapacity((emxArray__common *)xpost, ar, (int)sizeof(double));
  loop_ub = z->size[1] << 1;
  for (ar = 0; ar < loop_ub; ar++) {
    xpost->data[ar] = 0.0;
  }

  for (k = 0; k < z->size[1]; k++) {
    /*  Correct/Update (supports multiple observations) */
    /*  number of observations */
    b = 0.0;
    for (ar = 0; ar < 2; ar++) {
      b_Pprior[ar] = 0.0;
      for (loop_ub = 0; loop_ub < 2; loop_ub++) {
        b_Pprior[ar] += (1.0 - (double)loop_ub) * Pprior[loop_ub + (ar << 1)];
      }

      b += b_Pprior[ar] * (1.0 - (double)ar);
    }

    b = pinv(b + 1.96E-6);

    /*  Kalman gain */
    re = 0.0;
    for (ar = 0; ar < 2; ar++) {
      b_Pprior[ar] = 0.0;
      for (loop_ub = 0; loop_ub < 2; loop_ub++) {
        b_Pprior[ar] += Pprior[ar + (loop_ub << 1)] * (1.0 - (double)loop_ub);
      }

      K[ar] = b_Pprior[ar] * b;
      re += (1.0 - (double)ar) * xprior[ar].re - 0.0 * xprior[ar].im;
    }

    b = z->data[z->size[0] * k].re - re;
    for (ar = 0; ar < 2; ar++) {
      xpost->data[ar + xpost->size[0] * k] = xprior[ar].re + K[ar] * b;
    }

    /*  estimate x using msmt data */
    for (ar = 0; ar < 4; ar++) {
      I[ar] = 0.0;
    }

    for (loop_ub = 0; loop_ub < 2; loop_ub++) {
      I[loop_ub + (loop_ub << 1)] = 1.0;
    }

    for (ar = 0; ar < 2; ar++) {
      for (loop_ub = 0; loop_ub < 2; loop_ub++) {
        b_a[ar + (loop_ub << 1)] = I[ar + (loop_ub << 1)] - K[ar] * (1.0 -
          (double)loop_ub);
      }

      for (loop_ub = 0; loop_ub < 2; loop_ub++) {
        I[ar + (loop_ub << 1)] = 0.0;
        a[ar + (loop_ub << 1)] = 0.0;
        for (br = 0; br < 2; br++) {
          I[ar + (loop_ub << 1)] += b_a[ar + (br << 1)] * Pprior[br + (loop_ub <<
            1)];
          a[ar + (loop_ub << 1)] += b_a[ar + (br << 1)] * Pprior[br + (loop_ub <<
            1)];
        }
      }
    }

    for (ar = 0; ar < 2; ar++) {
      for (loop_ub = 0; loop_ub < 2; loop_ub++) {
        Ppost_data[loop_ub + (ar << 1)] = a[loop_ub + (ar << 1)];
      }
    }

    /*  update posterior error covariance */
    /*  Predict */
    for (ar = 0; ar < 2; ar++) {
      b = 0.0;
      for (loop_ub = 0; loop_ub < 2; loop_ub++) {
        b += (double)c_a[ar + (loop_ub << 1)] * xpost->data[loop_ub +
          xpost->size[0] * k];
      }

      xprior[ar].re = b;
      xprior[ar].im = 0.0;
    }

    /*  build prior based on previous position */
    for (ar = 0; ar < 2; ar++) {
      for (loop_ub = 0; loop_ub < 2; loop_ub++) {
        y_data[loop_ub + (ar << 1)] = 0.0;
      }
    }

    for (loop_ub = 0; loop_ub <= 2; loop_ub += 2) {
      for (ic = loop_ub; ic + 1 <= loop_ub + 2; ic++) {
        y_data[ic] = 0.0;
      }
    }

    br = 0;
    for (loop_ub = 0; loop_ub <= 2; loop_ub += 2) {
      ar = -1;
      for (ib = br; ib + 1 <= br + 2; ib++) {
        if (Ppost_data[ib] != 0.0) {
          ia = ar;
          for (ic = loop_ub; ic + 1 <= loop_ub + 2; ic++) {
            ia++;
            y_data[ic] += I[ib] * (double)iv4[ia];
          }
        }

        ar += 2;
      }

      br += 2;
    }

    for (ar = 0; ar < 4; ar++) {
      Pprior[ar] = 0.0;
    }

    for (loop_ub = 0; loop_ub <= 3; loop_ub += 2) {
      for (ic = loop_ub; ic + 1 <= loop_ub + 2; ic++) {
        Pprior[ic] = 0.0;
      }
    }

    br = 0;
    for (loop_ub = 0; loop_ub <= 3; loop_ub += 2) {
      ar = -1;
      for (ib = br; ib + 1 <= br + 2; ib++) {
        if (iv5[ib] != 0) {
          ia = ar;
          for (ic = loop_ub; ic + 1 <= loop_ub + 2; ic++) {
            ia++;
            Pprior[ic] += (double)iv5[ib] * y_data[ia];
          }
        }

        ar += 2;
      }

      br += 2;
    }

    for (ar = 0; ar < 4; ar++) {
      Pprior[ar] += Q[ar];
    }

    /*  build prior error covariance */
  }

  /*  figure, hold on */
  /*  plot(x,'--k') */
  /*  plot(xhat,'-r') */
  /*  plot(z,'-g') */
  /*  plot(xpost(1,:),'b','LineWidth',2) */
  /*  legend('x','xhat','z','xest') */
  loop_ub = xpost->size[1];
  ar = xest->size[0] * xest->size[1];
  xest->size[0] = 1;
  xest->size[1] = loop_ub;
  emxEnsureCapacity((emxArray__common *)xest, ar, (int)sizeof(double));
  for (ar = 0; ar < loop_ub; ar++) {
    xest->data[xest->size[0] * ar] = xpost->data[xpost->size[0] * ar];
  }

  emxFree_real_T(&xpost);
}

/*
 * File trailer for kalmanfilt.c
 *
 * [EOF]
 */
