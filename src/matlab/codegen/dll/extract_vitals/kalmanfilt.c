/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: kalmanfilt.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 01-Mar-2018 16:31:19
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "extract_vitals.h"
#include "kalmanfilt.h"
#include "pinv.h"

/* Function Definitions */

/*
 * Arguments    : const creal_T z_data[]
 *                const int z_size[2]
 *                double xest_data[]
 *                int xest_size[2]
 * Return Type  : void
 */
void kalmanfilt(const creal_T z_data[], const int z_size[2], double xest_data[],
                int xest_size[2])
{
  creal_T xprior[2];
  double Pprior[4];
  int k;
  static const signed char iv2[4] = { 1, 0, 0, 1 };

  double xpost_data[6];
  double a[4];
  int b_k;
  double b_Pprior[2];
  double b;
  int br;
  double re;
  double K[2];
  double I[4];
  double b_a[4];
  int ar;
  int ic;
  double Ppost_data[4];
  static const signed char c_a[4] = { 1, 0, 1, 1 };

  double y_data[4];
  int ib;
  int ia;
  static const double Q[4] = { 1.3225E-10, 2.645E-10, 2.645E-10, 5.29E-10 };

  static const signed char iv3[4] = { 1, 0, 1, 1 };

  static const signed char iv4[4] = { 1, 1, 0, 1 };

  /*  KALMANFILT   Kalman filter */
  /*    xest = kalmanfilt(xhat, sigmap, sigmam) */
  /*      For now, assumes 2D position, velocity setup. */
  /*  For codegen:  */
  /*    codegen -args {coder.typeof(double(0), [1 Inf]) double(0) double(0)} kalmanfilt.m */
  /*  process noise covariance */
  /*  measurement noise variance */
  /*  state transition matrix */
  /*  observation matrix */
  xprior[0] = z_data[0];
  xprior[1].re = 0.0;
  xprior[1].im = 0.0;

  /*  x,v initially */
  for (k = 0; k < 4; k++) {
    Pprior[k] = iv2[k];
  }

  /*  we are certain, thus 0 stddev */
  /*  Posterior estimate of signal x */
  for (b_k = 0; b_k < 3; b_k++) {
    /*  Correct/Update (supports multiple observations) */
    /*  number of observations */
    b = 0.0;
    for (k = 0; k < 2; k++) {
      b_Pprior[k] = 0.0;
      for (br = 0; br < 2; br++) {
        b_Pprior[k] += (1.0 - (double)br) * Pprior[br + (k << 1)];
      }

      b += b_Pprior[k] * (1.0 - (double)k);
    }

    b = pinv(b + 1.96E-6);

    /*  Kalman gain */
    re = 0.0;
    for (k = 0; k < 2; k++) {
      b_Pprior[k] = 0.0;
      for (br = 0; br < 2; br++) {
        b_Pprior[k] += Pprior[k + (br << 1)] * (1.0 - (double)br);
      }

      K[k] = b_Pprior[k] * b;
      re += (1.0 - (double)k) * xprior[k].re - 0.0 * xprior[k].im;
    }

    b = z_data[z_size[0] * b_k].re - re;
    for (k = 0; k < 2; k++) {
      xpost_data[k + (b_k << 1)] = xprior[k].re + K[k] * b;
    }

    /*  estimate x using msmt data */
    for (k = 0; k < 4; k++) {
      I[k] = 0.0;
    }

    for (k = 0; k < 2; k++) {
      I[k + (k << 1)] = 1.0;
    }

    for (k = 0; k < 2; k++) {
      for (br = 0; br < 2; br++) {
        a[k + (br << 1)] = I[k + (br << 1)] - K[k] * (1.0 - (double)br);
      }

      for (br = 0; br < 2; br++) {
        I[k + (br << 1)] = 0.0;
        b_a[k + (br << 1)] = 0.0;
        for (ar = 0; ar < 2; ar++) {
          I[k + (br << 1)] += a[k + (ar << 1)] * Pprior[ar + (br << 1)];
          b_a[k + (br << 1)] += a[k + (ar << 1)] * Pprior[ar + (br << 1)];
        }
      }
    }

    /*  update posterior error covariance */
    /*  Predict */
    /*  build prior based on previous position */
    for (k = 0; k < 2; k++) {
      b = 0.0;
      for (br = 0; br < 2; br++) {
        Ppost_data[br + (k << 1)] = b_a[br + (k << 1)];
        b += (double)c_a[k + (br << 1)] * xpost_data[br + (b_k << 1)];
        y_data[br + (k << 1)] = 0.0;
      }

      xprior[k].re = b;
      xprior[k].im = 0.0;
    }

    for (k = 0; k <= 2; k += 2) {
      for (ic = k; ic + 1 <= k + 2; ic++) {
        y_data[ic] = 0.0;
      }
    }

    br = 0;
    for (k = 0; k <= 2; k += 2) {
      ar = -1;
      for (ib = br; ib + 1 <= br + 2; ib++) {
        if (Ppost_data[ib] != 0.0) {
          ia = ar;
          for (ic = k; ic + 1 <= k + 2; ic++) {
            ia++;
            y_data[ic] += I[ib] * (double)iv3[ia];
          }
        }

        ar += 2;
      }

      br += 2;
    }

    for (k = 0; k < 4; k++) {
      Pprior[k] = 0.0;
    }

    for (k = 0; k <= 3; k += 2) {
      for (ic = k; ic + 1 <= k + 2; ic++) {
        Pprior[ic] = 0.0;
      }
    }

    br = 0;
    for (k = 0; k <= 3; k += 2) {
      ar = -1;
      for (ib = br; ib + 1 <= br + 2; ib++) {
        if (iv4[ib] != 0) {
          ia = ar;
          for (ic = k; ic + 1 <= k + 2; ic++) {
            ia++;
            Pprior[ic] += (double)iv4[ib] * y_data[ia];
          }
        }

        ar += 2;
      }

      br += 2;
    }

    for (k = 0; k < 4; k++) {
      Pprior[k] += Q[k];
    }

    /*  build prior error covariance */
  }

  /*  figure, hold on */
  /*  plot(x,'--k') */
  /*  plot(xhat,'-r') */
  /*  plot(z,'-g') */
  /*  plot(xpost(1,:),'b','LineWidth',2) */
  /*  legend('x','xhat','z','xest') */
  xest_size[0] = 1;
  xest_size[1] = 3;
  for (k = 0; k < 3; k++) {
    xest_data[k] = xpost_data[k << 1];
  }
}

/*
 * File trailer for kalmanfilt.c
 *
 * [EOF]
 */
