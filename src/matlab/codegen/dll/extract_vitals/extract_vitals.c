/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: extract_vitals.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 01-Mar-2018 16:31:19
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "extract_vitals.h"
#include "filter.h"
#include "plot_power_spectrum.h"
#include "kalmanfilt.h"
#include "extract_vitals_emxutil.h"
#include "rdivide.h"
#include "sum.h"
#include "get_spectral_entropy.h"
#include "fft.h"
#include "log.h"
#include "permute.h"
#include "imresize.h"
#include "fprintf.h"
#include "SystemCore.h"
#include "CascadeObjectDetector.h"
#include "flipud.h"

/* Function Definitions */

/*
 * assert(isa(frames,'double'))
 *  assert(isa(block_size,'double'))
 * Arguments    : const emxArray_real_T *frames
 *                double block_size
 *                double *hr
 *                double *rr
 * Return Type  : void
 */
void extract_vitals(const emxArray_real_T *frames, double block_size, double *hr,
                    double *rr)
{
  emxArray_real_T *frame;
  int loop_ub;
  int b_loop_ub;
  int i0;
  emxArray_real_T *face_rect;
  int nx;
  vision_CascadeObjectDetector faceDetector;
  emxArray_real_T *body;
  emxArray_real_T *head;
  double y;
  double mtmp_re;
  int Rbody_re;
  double b_face_rect;
  emxArray_real_T *Bbody;
  emxArray_real_T *Rbody;
  emxArray_real_T *x;
  int i1;
  int i2;
  emxArray_real_T *Rhead;
  int xhat_size[2];
  emxArray_real_T *Ahead_filt;
  emxArray_creal_T *Y;
  double xhat_data[3];
  double b_xhat_data[3];
  int Fheartrate_size[1];
  emxArray_creal_T *Fbreathing;
  emxArray_creal_T *Fheartrate;
  emxArray_creal_T *HRentr;
  emxArray_creal_T *b_HRentr;
  emxArray_creal_T *b_Ahead_filt;
  creal_T dc0;
  creal_T x_data[3];
  double im;
  creal_T b_x_data[3];
  int x_size[2];
  double Ahead_filt_re;
  double freq_data[1];
  creal_T Fheartrate_data[1];
  emxInit_real_T(&frame, 2);

  /*  tic */
  /*  frames = imrotate(frames,180); */
  /*  toc */
  cfprintf();

  /*  Need to make face pointing up so it works with Viola-Jones. */
  loop_ub = frames->size[0];
  b_loop_ub = frames->size[1];
  i0 = frame->size[0] * frame->size[1];
  frame->size[0] = loop_ub;
  frame->size[1] = b_loop_ub;
  emxEnsureCapacity((emxArray__common *)frame, i0, (int)sizeof(double));
  for (i0 = 0; i0 < b_loop_ub; i0++) {
    for (nx = 0; nx < loop_ub; nx++) {
      frame->data[nx + frame->size[0] * i0] = frames->data[nx + frames->size[0] *
        i0];
    }
  }

  emxInit_real_T(&face_rect, 2);
  flipud(frame);

  /*  Viola-Jones cascade face detector. Min size chosen empirically based on */
  /*  optical setup. */
  c_CascadeObjectDetector_Cascade(&faceDetector);

  /*  faceDetector = vision.CascadeObjectDetector('ProfileFace','MinSize',[75 75]); */
  SystemCore_step(&faceDetector, frame, face_rect);
  emxFree_real_T(&frame);
  emxInit_real_T1(&body, 3);
  emxInit_real_T1(&head, 3);
  if (face_rect->size[0] == 0) {
    /*    warning('No face detected. Using coarse ROIs.') */
    i0 = body->size[0] * body->size[1] * body->size[2];
    body->size[0] = 303;
    body->size[1] = 677;
    body->size[2] = 3;
    emxEnsureCapacity((emxArray__common *)body, i0, (int)sizeof(double));
    for (i0 = 0; i0 < 3; i0++) {
      for (nx = 0; nx < 677; nx++) {
        for (Rbody_re = 0; Rbody_re < 303; Rbody_re++) {
          body->data[(Rbody_re + body->size[0] * nx) + body->size[0] *
            body->size[1] * i0] = frames->data[((Rbody_re + frames->size[0] * (1
            + nx)) + frames->size[0] * frames->size[1] * i0) + 1];
        }
      }
    }

    i0 = head->size[0] * head->size[1] * head->size[2];
    head->size[0] = 409;
    head->size[1] = 412;
    head->size[2] = 3;
    emxEnsureCapacity((emxArray__common *)head, i0, (int)sizeof(double));
    for (i0 = 0; i0 < 3; i0++) {
      for (nx = 0; nx < 412; nx++) {
        for (Rbody_re = 0; Rbody_re < 409; Rbody_re++) {
          head->data[(Rbody_re + head->size[0] * nx) + head->size[0] *
            head->size[1] * i0] = frames->data[((Rbody_re + frames->size[0] *
            (109 + nx)) + frames->size[0] * frames->size[1] * i0) + 442];
        }
      }
    }
  } else {
    y = (double)frames->size[0] - face_rect->data[1];
    b_cfprintf();

    /*  Draw the returned bounding box around the detected face. */
    /*    videoOut = insertObjectAnnotation(frame,'rectangle',face_rect,'Face'); */
    /*    figure, imshow(videoOut), title('Detected face'); */
    /*  Expand the head ROI so we get the neck as well. */
    /*    head = frames(y-h*1.5 : y+h*.10, x:x+w, :); */
    mtmp_re = y - face_rect->data[3];
    if (mtmp_re > y) {
      i0 = 0;
      nx = 0;
    } else {
      i0 = (int)mtmp_re - 1;
      nx = (int)y;
    }

    b_face_rect = face_rect->data[0];
    mtmp_re = face_rect->data[2];
    Rbody_re = head->size[0] * head->size[1] * head->size[2];
    head->size[0] = nx - i0;
    head->size[1] = (int)floor(mtmp_re) + 1;
    head->size[2] = 3;
    emxEnsureCapacity((emxArray__common *)head, Rbody_re, (int)sizeof(double));
    for (Rbody_re = 0; Rbody_re < 3; Rbody_re++) {
      loop_ub = (int)floor(mtmp_re);
      for (i1 = 0; i1 <= loop_ub; i1++) {
        b_loop_ub = nx - i0;
        for (i2 = 0; i2 < b_loop_ub; i2++) {
          head->data[(i2 + head->size[0] * i1) + head->size[0] * head->size[1] *
            Rbody_re] = frames->data[((i0 + i2) + frames->size[0] * ((int)
            (b_face_rect + (double)i1) - 1)) + frames->size[0] * frames->size[1]
            * Rbody_re];
        }
      }
    }

    /*  The rest is body. */
    mtmp_re = y - face_rect->data[3] * 2.0;
    if (1.0 > mtmp_re) {
      loop_ub = 0;
    } else {
      loop_ub = (int)mtmp_re;
    }

    if (1 > frames->size[1]) {
      b_loop_ub = 0;
    } else {
      b_loop_ub = frames->size[1];
    }

    i0 = body->size[0] * body->size[1] * body->size[2];
    body->size[0] = loop_ub;
    body->size[1] = b_loop_ub;
    body->size[2] = 3;
    emxEnsureCapacity((emxArray__common *)body, i0, (int)sizeof(double));
    for (i0 = 0; i0 < 3; i0++) {
      for (nx = 0; nx < b_loop_ub; nx++) {
        for (Rbody_re = 0; Rbody_re < loop_ub; Rbody_re++) {
          body->data[(Rbody_re + body->size[0] * nx) + body->size[0] *
            body->size[1] * i0] = frames->data[(Rbody_re + frames->size[0] * nx)
            + frames->size[0] * frames->size[1] * i0];
        }
      }
    }
  }

  emxFree_real_T(&face_rect);
  emxInit_real_T1(&Bbody, 3);
  emxInit_real_T(&Rbody, 2);
  emxInit_real_T1(&x, 3);
  imresize(body, 1.0 / block_size, Bbody);
  permute(Bbody, x);
  nx = 3 * x->size[1] * x->size[2];
  i0 = Rbody->size[0] * Rbody->size[1];
  Rbody->size[0] = 3;
  Rbody->size[1] = nx / 3;
  emxEnsureCapacity((emxArray__common *)Rbody, i0, (int)sizeof(double));
  b_loop_ub = 0;
  emxFree_real_T(&Bbody);
  while (b_loop_ub + 1 <= nx) {
    Rbody->data[b_loop_ub] = x->data[b_loop_ub];
    b_loop_ub++;
  }

  i0 = Rbody->size[0] * Rbody->size[1];
  Rbody->size[0] = 3;
  emxEnsureCapacity((emxArray__common *)Rbody, i0, (int)sizeof(double));
  b_loop_ub = Rbody->size[0];
  nx = Rbody->size[1];
  loop_ub = b_loop_ub * nx;
  for (i0 = 0; i0 < loop_ub; i0++) {
    Rbody->data[i0]++;
  }

  b_log(Rbody);
  i0 = Rbody->size[0] * Rbody->size[1];
  Rbody->size[0] = 3;
  emxEnsureCapacity((emxArray__common *)Rbody, i0, (int)sizeof(double));
  b_loop_ub = Rbody->size[0];
  nx = Rbody->size[1];
  loop_ub = b_loop_ub * nx;
  for (i0 = 0; i0 < loop_ub; i0++) {
    Rbody->data[i0] = -Rbody->data[i0];
  }

  emxInit_real_T(&Rhead, 2);
  imresize(head, 1.0 / block_size, body);
  permute(body, x);
  nx = 3 * x->size[1] * x->size[2];
  i0 = Rhead->size[0] * Rhead->size[1];
  Rhead->size[0] = 3;
  Rhead->size[1] = nx / 3;
  emxEnsureCapacity((emxArray__common *)Rhead, i0, (int)sizeof(double));
  b_loop_ub = 0;
  emxFree_real_T(&head);
  emxFree_real_T(&body);
  while (b_loop_ub + 1 <= nx) {
    Rhead->data[b_loop_ub] = x->data[b_loop_ub];
    b_loop_ub++;
  }

  emxFree_real_T(&x);
  i0 = Rhead->size[0] * Rhead->size[1];
  Rhead->size[0] = 3;
  emxEnsureCapacity((emxArray__common *)Rhead, i0, (int)sizeof(double));
  b_loop_ub = Rhead->size[0];
  nx = Rhead->size[1];
  loop_ub = b_loop_ub * nx;
  for (i0 = 0; i0 < loop_ub; i0++) {
    Rhead->data[i0]++;
  }

  b_log(Rhead);
  i0 = Rhead->size[0] * Rhead->size[1];
  Rhead->size[0] = 3;
  emxEnsureCapacity((emxArray__common *)Rhead, i0, (int)sizeof(double));
  b_loop_ub = Rhead->size[0];
  nx = Rhead->size[1];
  loop_ub = b_loop_ub * nx;
  for (i0 = 0; i0 < loop_ub; i0++) {
    Rhead->data[i0] = -Rhead->data[i0];
  }

  /*  */
  /*  breathing = mean(Abody,2); */
  /*  heartrate = mean(Ahead,2); */
  /*  Get rid of breathing component of heartrate signal. */
  /*  x_ts = timeseries(Ahead, [0:T-1]./60); */
  /*  x_filt = idealfilter(x_ts, [low_freq high_freq], 'pass'); */
  /*  Ahead_filt = x_filt.Data; */
  for (i0 = 0; i0 < 2; i0++) {
    xhat_size[i0] = Rhead->size[i0];
  }

  emxInit_real_T(&Ahead_filt, 2);
  i0 = Ahead_filt->size[0] * Ahead_filt->size[1];
  Ahead_filt->size[0] = 3;
  Ahead_filt->size[1] = xhat_size[1];
  emxEnsureCapacity((emxArray__common *)Ahead_filt, i0, (int)sizeof(double));
  loop_ub = 3 * xhat_size[1];
  for (i0 = 0; i0 < loop_ub; i0++) {
    Ahead_filt->data[i0] = 0.0;
  }

  for (b_loop_ub = 0; b_loop_ub < xhat_size[1]; b_loop_ub++) {
    for (i0 = 0; i0 < 3; i0++) {
      xhat_data[i0] = Rhead->data[i0 + Rhead->size[0] * b_loop_ub];
    }

    filter(xhat_data, b_xhat_data, Fheartrate_size);
    for (i0 = 0; i0 < 3; i0++) {
      Ahead_filt->data[i0 + Ahead_filt->size[0] * b_loop_ub] = b_xhat_data[i0];
    }
  }

  emxInit_creal_T(&Y, 2);

  /*  Do weighted average of signals based on (inverse) entropy to yield a */
  /*  signal (heartrate and breathing). */
  /*  power = (a^2+b^2)/N */
  /*  Should be same as 1/N*abs(Y)^2 */
  fft(Rbody, Rbody->size[1], Y);
  i0 = Y->size[0] * Y->size[1];
  emxEnsureCapacity((emxArray__common *)Y, i0, (int)sizeof(creal_T));
  b_loop_ub = Y->size[0];
  nx = Y->size[1];
  Rbody_re = Rbody->size[1];
  loop_ub = b_loop_ub * nx;
  for (i0 = 0; i0 < loop_ub; i0++) {
    y = Y->data[i0].re;
    b_face_rect = -Y->data[i0].im;
    mtmp_re = Y->data[i0].re * y - Y->data[i0].im * b_face_rect;
    b_face_rect = Y->data[i0].re * b_face_rect + Y->data[i0].im * y;
    if (b_face_rect == 0.0) {
      Y->data[i0].re = mtmp_re / (double)Rbody_re;
      Y->data[i0].im = 0.0;
    } else if (mtmp_re == 0.0) {
      Y->data[i0].re = 0.0;
      Y->data[i0].im = b_face_rect / (double)Rbody_re;
    } else {
      Y->data[i0].re = mtmp_re / (double)Rbody_re;
      Y->data[i0].im = b_face_rect / (double)Rbody_re;
    }
  }

  i0 = (int)floor((double)Rbody->size[1] / 2.0);
  if (1 > i0) {
    loop_ub = 0;
  } else {
    loop_ub = i0;
  }

  emxInit_creal_T(&Fbreathing, 2);
  b_loop_ub = Y->size[1];
  i0 = Fbreathing->size[0] * Fbreathing->size[1];
  Fbreathing->size[0] = loop_ub;
  Fbreathing->size[1] = b_loop_ub;
  emxEnsureCapacity((emxArray__common *)Fbreathing, i0, (int)sizeof(creal_T));
  for (i0 = 0; i0 < b_loop_ub; i0++) {
    for (nx = 0; nx < loop_ub; nx++) {
      Fbreathing->data[nx + Fbreathing->size[0] * i0] = Y->data[nx + Y->size[0] *
        i0];
    }
  }

  fft(Rhead, Rbody->size[1], Y);
  i0 = Y->size[0] * Y->size[1];
  emxEnsureCapacity((emxArray__common *)Y, i0, (int)sizeof(creal_T));
  b_loop_ub = Y->size[0];
  nx = Y->size[1];
  Rbody_re = Rbody->size[1];
  loop_ub = b_loop_ub * nx;
  emxFree_real_T(&Rhead);
  for (i0 = 0; i0 < loop_ub; i0++) {
    y = Y->data[i0].re;
    b_face_rect = -Y->data[i0].im;
    mtmp_re = Y->data[i0].re * y - Y->data[i0].im * b_face_rect;
    b_face_rect = Y->data[i0].re * b_face_rect + Y->data[i0].im * y;
    if (b_face_rect == 0.0) {
      Y->data[i0].re = mtmp_re / (double)Rbody_re;
      Y->data[i0].im = 0.0;
    } else if (mtmp_re == 0.0) {
      Y->data[i0].re = 0.0;
      Y->data[i0].im = b_face_rect / (double)Rbody_re;
    } else {
      Y->data[i0].re = mtmp_re / (double)Rbody_re;
      Y->data[i0].im = b_face_rect / (double)Rbody_re;
    }
  }

  i0 = (int)floor((double)Rbody->size[1] / 2.0);
  emxFree_real_T(&Rbody);
  if (1 > i0) {
    loop_ub = 0;
  } else {
    loop_ub = i0;
  }

  emxInit_creal_T(&Fheartrate, 2);
  b_loop_ub = Y->size[1];
  i0 = Fheartrate->size[0] * Fheartrate->size[1];
  Fheartrate->size[0] = loop_ub;
  Fheartrate->size[1] = b_loop_ub;
  emxEnsureCapacity((emxArray__common *)Fheartrate, i0, (int)sizeof(creal_T));
  for (i0 = 0; i0 < b_loop_ub; i0++) {
    for (nx = 0; nx < loop_ub; nx++) {
      Fheartrate->data[nx + Fheartrate->size[0] * i0] = Y->data[nx + Y->size[0] *
        i0];
    }
  }

  emxFree_creal_T(&Y);

  /*  [freq, Fbreathing] = plot_power_spectrum(Abody, 60); */
  /*  [freq, Fheartrate] = plot_power_spectrum(Ahead_filt, 60); */
  i0 = Fbreathing->size[1];
  b_loop_ub = 0;
  emxFree_creal_T(&Fbreathing);
  while (b_loop_ub <= i0 - 1) {
    Fheartrate->data[Fheartrate->size[0] * b_loop_ub].re = 0.0;
    Fheartrate->data[Fheartrate->size[0] * b_loop_ub].im = 0.0;
    b_loop_ub++;
  }

  emxInit_creal_T(&HRentr, 2);

  /*  Fbreathing(1,:) = zeros(1, size(Fbreathing,2)); */
  /*  Fheartrate(1,:) = zeros(1, size(Fheartrate,2)); */
  get_spectral_entropy(Fheartrate, HRentr);
  i0 = HRentr->size[0] * HRentr->size[1];
  HRentr->size[0] = 1;
  emxEnsureCapacity((emxArray__common *)HRentr, i0, (int)sizeof(creal_T));
  b_loop_ub = HRentr->size[0];
  nx = HRentr->size[1];
  loop_ub = b_loop_ub * nx;
  emxFree_creal_T(&Fheartrate);
  for (i0 = 0; i0 < loop_ub; i0++) {
    HRentr->data[i0].re = 1.0 - HRentr->data[i0].re;
    HRentr->data[i0].im = 0.0 - HRentr->data[i0].im;
  }

  emxInit_creal_T(&b_HRentr, 2);
  i0 = b_HRentr->size[0] * b_HRentr->size[1];
  b_HRentr->size[0] = 1;
  b_HRentr->size[1] = HRentr->size[1];
  emxEnsureCapacity((emxArray__common *)b_HRentr, i0, (int)sizeof(creal_T));
  loop_ub = HRentr->size[0] * HRentr->size[1];
  for (i0 = 0; i0 < loop_ub; i0++) {
    b_HRentr->data[i0] = HRentr->data[i0];
  }

  emxInit_creal_T(&b_Ahead_filt, 2);
  dc0 = sum(HRentr);
  rdivide(b_HRentr, dc0, HRentr);

  /*  Use fft to get rid of phase differences between signals. */
  i0 = b_Ahead_filt->size[0] * b_Ahead_filt->size[1];
  b_Ahead_filt->size[0] = 3;
  b_Ahead_filt->size[1] = Ahead_filt->size[1];
  emxEnsureCapacity((emxArray__common *)b_Ahead_filt, i0, (int)sizeof(creal_T));
  loop_ub = Ahead_filt->size[1];
  emxFree_creal_T(&b_HRentr);
  for (i0 = 0; i0 < loop_ub; i0++) {
    for (nx = 0; nx < 3; nx++) {
      b_Ahead_filt->data[nx + b_Ahead_filt->size[0] * i0].re = Ahead_filt->
        data[nx + Ahead_filt->size[0] * i0];
      b_Ahead_filt->data[nx + b_Ahead_filt->size[0] * i0].im = 0.0;
    }
  }

  emxFree_real_T(&Ahead_filt);
  for (i0 = 0; i0 < 3; i0++) {
    mtmp_re = 0.0;
    im = 0.0;
    loop_ub = b_Ahead_filt->size[1];
    for (nx = 0; nx < loop_ub; nx++) {
      y = HRentr->data[HRentr->size[0] * nx].re;
      b_face_rect = -HRentr->data[HRentr->size[0] * nx].im;
      Ahead_filt_re = b_Ahead_filt->data[i0 + b_Ahead_filt->size[0] * nx].re * y
        - b_Ahead_filt->data[i0 + b_Ahead_filt->size[0] * nx].im * b_face_rect;
      y = b_Ahead_filt->data[i0 + b_Ahead_filt->size[0] * nx].re * b_face_rect +
        b_Ahead_filt->data[i0 + b_Ahead_filt->size[0] * nx].im * y;
      mtmp_re += Ahead_filt_re;
      im += y;
    }

    x_data[i0].re = -mtmp_re;
    x_data[i0].im = -im;
  }

  emxFree_creal_T(&b_Ahead_filt);
  emxFree_creal_T(&HRentr);
  x_size[0] = 1;
  x_size[1] = 3;
  for (b_loop_ub = 0; b_loop_ub < 3; b_loop_ub++) {
    if (x_data[b_loop_ub].im == 0.0) {
      mtmp_re = exp(x_data[b_loop_ub].re);
      y = 0.0;
    } else if (rtIsInf(x_data[b_loop_ub].im) && rtIsInf(x_data[b_loop_ub].re) &&
               (x_data[b_loop_ub].re < 0.0)) {
      mtmp_re = 0.0;
      y = 0.0;
    } else {
      y = exp(x_data[b_loop_ub].re / 2.0);
      mtmp_re = y * (y * cos(x_data[b_loop_ub].im));
      y *= y * sin(x_data[b_loop_ub].im);
    }

    x_data[b_loop_ub].re = mtmp_re;
    x_data[b_loop_ub].im = y;
    b_x_data[b_loop_ub].re = x_data[b_loop_ub].re - 1.0;
    b_x_data[b_loop_ub].im = -x_data[b_loop_ub].im;
  }

  kalmanfilt(b_x_data, x_size, xhat_data, xhat_size);

  /*   */
  /*  figure; */
  /*  subplot(1,2,1), */
  /*  [H,W,~] = size(Bhead); */
  /*  imshow(flipud((reshape(whr,[H W]))),[]) */
  /*  title('Pulse weights') */
  /*  subplot(1,2,2) */
  /*  [H,W,~] = size(Bbody); */
  /*  imshow(flipud(reshape(wrr,[H W])),[]); */
  /*  title('Breathing weights') */
  /*  Find vitals */
  /* PLOT_POWER_SPECTRUM Plot frequency power spectrum */
  /*  */
  /*  [freq,fpower] = plot_power_spectrum(y, sampling_rate) */
  /*  */
  /*  From http://www.mathworks.com/products/matlab/examples.html?file=/products/demos/shipping/matlab/fftdemo.html */
  /*  y - signal (columns are observations) */
  /*  if size(y,2) > size(y,1) */
  /*    warning(sprintf('Are you sure columns are observations? (r%u,c%u)', size(y,1), size(y,2))) */
  /*  end */
  /*  npts = numel(y); */
  /*  power = (a^2+b^2)/N */
  /*  Should be same as 1/N*abs(Y)^2 */
  /*  nth bin is n*Fs/N, +1 for index offset */
  for (b_loop_ub = 0; b_loop_ub < 3; b_loop_ub++) {
    b_xhat_data[b_loop_ub] = -log(xhat_data[b_loop_ub] + 1.0);
  }

  plot_power_spectrum(b_xhat_data, freq_data, xhat_size, Fheartrate_data,
                      Fheartrate_size);
  *hr = freq_data[0];
  *rr = freq_data[0];

  /*  t = [0:numel(heartrate_best)-1] ./ 60; */
  /*  figure; */
  /*  subplot(2,2,1), plot(t,heartrate_best), title('Heart rate') */
  /*  hold on, plot(t,heartrate,'--k') */
  /*  xlabel('time (s)') */
  /*  ylabel('blood volume (a.u.)') */
  /*  legend('strongest signal','weighted average') */
  /*   */
  /*  subplot(2,2,2), plot(t,breathing), title('Breathing') */
  /*  xlabel('time (s)') */
  /*  ylabel('breathing amplitude (a.u.)') */
  /*   */
  /*  subplot(2,2,3), plot(60*freq,Fheartrate), title(sprintf('HR=%u bpm',round(hr*60))) */
  /*  [~,maxidx] = max(Fheartrate); */
  /*  hold on, plot(60*freq(maxidx),Fheartrate(maxidx),'or') */
  /*  xlim([0 300]) */
  /*  xlabel('frequency (1/min)') */
  /*  ylabel('spectral power (dB)') */
  /*   */
  /*  subplot(2,2,4), plot(60*freq,Fbreathing), title(sprintf('RR=%u bpm',round(rr*60))) */
  /*  [~,maxidx] = max(Fbreathing); */
  /*  hold on, plot(60*freq(maxidx),Fbreathing(maxidx),'or') */
  /*  xlim([0 300]) */
  /*  xlabel('frequency (1/min)') */
  /*  ylabel('spectral power (dB)') */
}

/*
 * File trailer for extract_vitals.c
 *
 * [EOF]
 */
