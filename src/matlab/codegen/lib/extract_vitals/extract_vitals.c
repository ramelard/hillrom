/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: extract_vitals.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 16-Apr-2018 17:05:50
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "extract_vitals.h"
#include "filter.h"
#include "extract_vitals_emxutil.h"
#include "relop.h"
#include "fft.h"
#include "plot_power_spectrum.h"
#include "log.h"
#include "kalmanfilt.h"
#include "exp.h"
#include "ifft.h"
#include "abs1.h"
#include "rdivide.h"
#include "sum.h"
#include "get_spectral_entropy.h"
#include "permute.h"
#include "imresize.h"

/* Function Declarations */
static int div_s32(int numerator, int denominator);

/* Function Definitions */

/*
 * Arguments    : int numerator
 *                int denominator
 * Return Type  : int
 */
static int div_s32(int numerator, int denominator)
{
  int quotient;
  unsigned int absNumerator;
  unsigned int absDenominator;
  boolean_T quotientNeedsNegation;
  if (denominator == 0) {
    if (numerator >= 0) {
      quotient = MAX_int32_T;
    } else {
      quotient = MIN_int32_T;
    }
  } else {
    if (numerator < 0) {
      absNumerator = ~(unsigned int)numerator + 1U;
    } else {
      absNumerator = (unsigned int)numerator;
    }

    if (denominator < 0) {
      absDenominator = ~(unsigned int)denominator + 1U;
    } else {
      absDenominator = (unsigned int)denominator;
    }

    quotientNeedsNegation = ((numerator < 0) != (denominator < 0));
    absNumerator /= absDenominator;
    if (quotientNeedsNegation) {
      quotient = -(int)absNumerator;
    } else {
      quotient = (int)absNumerator;
    }
  }

  return quotient;
}

/*
 * assert(isa(frames,'double'))
 *  assert(isa(block_size,'double'))
 * Arguments    : const emxArray_real_T *frames
 *                const double face_rect[4]
 *                double block_size
 *                double *hr
 *                double *rr
 * Return Type  : void
 */
void extract_vitals(const emxArray_real_T *frames, const double face_rect[4],
                    double block_size, double *hr, double *rr)
{
  double y;
  double R_im;
  int i0;
  int calclen;
  emxArray_real_T *head;
  int loop_ub;
  int sz_idx_0;
  int nx;
  int i1;
  int b_loop_ub;
  int i2;
  emxArray_real_T *b_frames;
  emxArray_real_T *Bbody;
  emxArray_real_T *x;
  emxArray_real_T *Rbody;
  emxArray_real_T *Rhead;
  int iv0[2];
  emxArray_real_T *Ahead_filt;
  emxArray_real_T *b_Rhead;
  emxArray_real_T *r0;
  emxArray_creal_T *Y;
  double R_re;
  emxArray_creal_T *Fbreathing;
  emxArray_creal_T *Fheartrate;
  emxArray_creal_T *b_Fheartrate;
  emxArray_creal_T *HRentr;
  emxArray_creal_T *b_HRentr;
  emxArray_creal_T *b_Fbreathing;
  creal_T c_Fbreathing;
  emxArray_creal_T *RRentr;
  emxArray_creal_T *b_RRentr;
  emxArray_creal_T *b_Rbody;
  emxArray_creal_T *c_Rbody;
  emxArray_creal_T *breathing;
  emxArray_creal_T *b_Ahead_filt;
  emxArray_creal_T *R;
  double im;
  emxArray_creal_T *b_R;
  double Ahead_filt_re;
  emxArray_real_T *xhat;
  emxArray_real_T *heartrate;
  creal_T mtmp;
  emxArray_real_T *b_x;
  boolean_T exitg3;
  emxArray_real_T *c_x;
  emxArray_real_T *d_x;
  emxArray_real_T *b_xhat;
  emxArray_creal_T *d_Fbreathing;
  emxArray_real_T *c_xhat;
  int b_heartrate[1];
  emxArray_real_T c_heartrate;
  boolean_T exitg2;
  boolean_T exitg1;

  /*  tic */
  /*  frames = imrotate(frames,180); */
  /*  toc */
  /*  fprintf(1,'Finding face...') */
  /*  % Need to make face pointing up so it works with Viola-Jones. */
  /*  frame = flipud(frames(:,:,1)); */
  /*  % Viola-Jones cascade face detector. Min size chosen empirically based on */
  /*  % optical setup. */
  /*  faceDetector = vision.CascadeObjectDetector('MinSize',[75 75]); */
  /*  % faceDetector = vision.CascadeObjectDetector('ProfileFace','MinSize',[75 75]); */
  /*  face_rect = step(faceDetector, frame); */
  /*   */
  /*  if isempty(face_rect) */
  /*    warning('No face detected. Using coarse ROIs.') */
  /*    body = frames(2:304,2:678,:); */
  /*    head = frames(443:851, 110:521, :); */
  /*  else */
  y = (double)frames->size[0] - face_rect[1];

  /*  Draw the returned bounding box around the detected face. */
  /*    videoOut = insertObjectAnnotation(frame,'rectangle',face_rect,'Face'); */
  /*    figure, imshow(videoOut), title('Detected face'); */
  /*  Expand the head ROI so we get the neck as well. */
  /*    head = frames(y-h*1.5 : y+h*.10, x:x+w, :); */
  R_im = y - face_rect[3];
  if (R_im > y) {
    i0 = 0;
    calclen = 0;
  } else {
    i0 = (int)R_im - 1;
    calclen = (int)y;
  }

  emxInit_real_T(&head, 3);
  loop_ub = frames->size[2];
  sz_idx_0 = head->size[0] * head->size[1] * head->size[2];
  head->size[0] = calclen - i0;
  head->size[1] = (int)floor(face_rect[2]) + 1;
  head->size[2] = loop_ub;
  emxEnsureCapacity((emxArray__common *)head, sz_idx_0, (int)sizeof(double));
  for (sz_idx_0 = 0; sz_idx_0 < loop_ub; sz_idx_0++) {
    nx = (int)floor(face_rect[2]);
    for (i1 = 0; i1 <= nx; i1++) {
      b_loop_ub = calclen - i0;
      for (i2 = 0; i2 < b_loop_ub; i2++) {
        head->data[(i2 + head->size[0] * i1) + head->size[0] * head->size[1] *
          sz_idx_0] = frames->data[((i0 + i2) + frames->size[0] * ((int)
          (face_rect[0] + (double)i1) - 1)) + frames->size[0] * frames->size[1] *
          sz_idx_0];
      }
    }
  }

  /*  The rest is body. */
  R_im = y - face_rect[3] * 2.0;
  if (1.0 > R_im) {
    loop_ub = 0;
  } else {
    loop_ub = (int)R_im;
  }

  if (1 > frames->size[1]) {
    nx = 0;
  } else {
    nx = frames->size[1];
  }

  emxInit_real_T(&b_frames, 3);

  /*  end */
  b_loop_ub = frames->size[2];
  i0 = b_frames->size[0] * b_frames->size[1] * b_frames->size[2];
  b_frames->size[0] = loop_ub;
  b_frames->size[1] = nx;
  b_frames->size[2] = b_loop_ub;
  emxEnsureCapacity((emxArray__common *)b_frames, i0, (int)sizeof(double));
  for (i0 = 0; i0 < b_loop_ub; i0++) {
    for (calclen = 0; calclen < nx; calclen++) {
      for (sz_idx_0 = 0; sz_idx_0 < loop_ub; sz_idx_0++) {
        b_frames->data[(sz_idx_0 + b_frames->size[0] * calclen) + b_frames->
          size[0] * b_frames->size[1] * i0] = frames->data[(sz_idx_0 +
          frames->size[0] * calclen) + frames->size[0] * frames->size[1] * i0];
      }
    }
  }

  emxInit_real_T(&Bbody, 3);
  emxInit_real_T(&x, 3);
  imresize(b_frames, 1.0 / block_size, Bbody);
  permute(Bbody, x);
  nx = x->size[0] * x->size[1] * x->size[2];
  emxFree_real_T(&b_frames);
  if (Bbody->size[2] > 0) {
    calclen = div_s32(nx, Bbody->size[2]);
  } else {
    calclen = 0;
  }

  emxInit_real_T1(&Rbody, 2);
  sz_idx_0 = Bbody->size[2];
  i0 = Rbody->size[0] * Rbody->size[1];
  Rbody->size[0] = sz_idx_0;
  Rbody->size[1] = calclen;
  emxEnsureCapacity((emxArray__common *)Rbody, i0, (int)sizeof(double));
  for (calclen = 0; calclen + 1 <= nx; calclen++) {
    Rbody->data[calclen] = x->data[calclen];
  }

  i0 = Rbody->size[0] * Rbody->size[1];
  emxEnsureCapacity((emxArray__common *)Rbody, i0, (int)sizeof(double));
  calclen = Rbody->size[0];
  sz_idx_0 = Rbody->size[1];
  loop_ub = calclen * sz_idx_0;
  for (i0 = 0; i0 < loop_ub; i0++) {
    Rbody->data[i0]++;
  }

  c_log(Rbody);
  i0 = Rbody->size[0] * Rbody->size[1];
  emxEnsureCapacity((emxArray__common *)Rbody, i0, (int)sizeof(double));
  calclen = Rbody->size[0];
  sz_idx_0 = Rbody->size[1];
  loop_ub = calclen * sz_idx_0;
  for (i0 = 0; i0 < loop_ub; i0++) {
    Rbody->data[i0] = -Rbody->data[i0];
  }

  imresize(head, 1.0 / block_size, Bbody);
  permute(Bbody, x);
  nx = x->size[0] * x->size[1] * x->size[2];
  emxFree_real_T(&head);
  if (Bbody->size[2] > 0) {
    calclen = div_s32(nx, Bbody->size[2]);
  } else {
    calclen = 0;
  }

  emxInit_real_T1(&Rhead, 2);
  sz_idx_0 = Bbody->size[2];
  i0 = Rhead->size[0] * Rhead->size[1];
  Rhead->size[0] = sz_idx_0;
  Rhead->size[1] = calclen;
  emxEnsureCapacity((emxArray__common *)Rhead, i0, (int)sizeof(double));
  calclen = 0;
  emxFree_real_T(&Bbody);
  while (calclen + 1 <= nx) {
    Rhead->data[calclen] = x->data[calclen];
    calclen++;
  }

  emxFree_real_T(&x);
  i0 = Rhead->size[0] * Rhead->size[1];
  emxEnsureCapacity((emxArray__common *)Rhead, i0, (int)sizeof(double));
  calclen = Rhead->size[0];
  sz_idx_0 = Rhead->size[1];
  loop_ub = calclen * sz_idx_0;
  for (i0 = 0; i0 < loop_ub; i0++) {
    Rhead->data[i0]++;
  }

  c_log(Rhead);
  i0 = Rhead->size[0] * Rhead->size[1];
  emxEnsureCapacity((emxArray__common *)Rhead, i0, (int)sizeof(double));
  calclen = Rhead->size[0];
  sz_idx_0 = Rhead->size[1];
  loop_ub = calclen * sz_idx_0;
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
    iv0[i0] = Rhead->size[i0];
  }

  emxInit_real_T1(&Ahead_filt, 2);
  i0 = Ahead_filt->size[0] * Ahead_filt->size[1];
  Ahead_filt->size[0] = iv0[0];
  Ahead_filt->size[1] = iv0[1];
  emxEnsureCapacity((emxArray__common *)Ahead_filt, i0, (int)sizeof(double));
  loop_ub = iv0[0] * iv0[1];
  for (i0 = 0; i0 < loop_ub; i0++) {
    Ahead_filt->data[i0] = 0.0;
  }

  calclen = 0;
  emxInit_real_T2(&b_Rhead, 1);
  emxInit_real_T2(&r0, 1);
  while (calclen <= iv0[1] - 1) {
    loop_ub = Rhead->size[0];
    i0 = b_Rhead->size[0];
    b_Rhead->size[0] = loop_ub;
    emxEnsureCapacity((emxArray__common *)b_Rhead, i0, (int)sizeof(double));
    for (i0 = 0; i0 < loop_ub; i0++) {
      b_Rhead->data[i0] = Rhead->data[i0 + Rhead->size[0] * calclen];
    }

    filter(b_Rhead, r0);
    loop_ub = r0->size[0];
    for (i0 = 0; i0 < loop_ub; i0++) {
      Ahead_filt->data[i0 + Ahead_filt->size[0] * calclen] = r0->data[i0];
    }

    calclen++;
  }

  emxFree_real_T(&r0);
  emxFree_real_T(&b_Rhead);
  emxInit_creal_T(&Y, 2);

  /*  Do weighted average of signals based on (inverse) entropy to yield a */
  /*  signal (heartrate and breathing). */
  /*  power = (a^2+b^2)/N */
  /*  Should be same as 1/N*abs(Y)^2 */
  fft(Rbody, Rbody->size[0], Y);
  i0 = Y->size[0] * Y->size[1];
  emxEnsureCapacity((emxArray__common *)Y, i0, (int)sizeof(creal_T));
  calclen = Y->size[0];
  sz_idx_0 = Y->size[1];
  nx = Rbody->size[0];
  loop_ub = calclen * sz_idx_0;
  for (i0 = 0; i0 < loop_ub; i0++) {
    y = Y->data[i0].re;
    R_im = -Y->data[i0].im;
    R_re = Y->data[i0].re * y - Y->data[i0].im * R_im;
    R_im = Y->data[i0].re * R_im + Y->data[i0].im * y;
    if (R_im == 0.0) {
      Y->data[i0].re = R_re / (double)nx;
      Y->data[i0].im = 0.0;
    } else if (R_re == 0.0) {
      Y->data[i0].re = 0.0;
      Y->data[i0].im = R_im / (double)nx;
    } else {
      Y->data[i0].re = R_re / (double)nx;
      Y->data[i0].im = R_im / (double)nx;
    }
  }

  i0 = (int)floor((double)Rbody->size[0] / 2.0);
  if (1 > i0) {
    loop_ub = 0;
  } else {
    loop_ub = i0;
  }

  emxInit_creal_T(&Fbreathing, 2);
  nx = Y->size[1];
  i0 = Fbreathing->size[0] * Fbreathing->size[1];
  Fbreathing->size[0] = loop_ub;
  Fbreathing->size[1] = nx;
  emxEnsureCapacity((emxArray__common *)Fbreathing, i0, (int)sizeof(creal_T));
  for (i0 = 0; i0 < nx; i0++) {
    for (calclen = 0; calclen < loop_ub; calclen++) {
      Fbreathing->data[calclen + Fbreathing->size[0] * i0] = Y->data[calclen +
        Y->size[0] * i0];
    }
  }

  fft(Rhead, Rbody->size[0], Y);
  i0 = Y->size[0] * Y->size[1];
  emxEnsureCapacity((emxArray__common *)Y, i0, (int)sizeof(creal_T));
  calclen = Y->size[0];
  sz_idx_0 = Y->size[1];
  nx = Rbody->size[0];
  loop_ub = calclen * sz_idx_0;
  emxFree_real_T(&Rhead);
  for (i0 = 0; i0 < loop_ub; i0++) {
    y = Y->data[i0].re;
    R_im = -Y->data[i0].im;
    R_re = Y->data[i0].re * y - Y->data[i0].im * R_im;
    R_im = Y->data[i0].re * R_im + Y->data[i0].im * y;
    if (R_im == 0.0) {
      Y->data[i0].re = R_re / (double)nx;
      Y->data[i0].im = 0.0;
    } else if (R_re == 0.0) {
      Y->data[i0].re = 0.0;
      Y->data[i0].im = R_im / (double)nx;
    } else {
      Y->data[i0].re = R_re / (double)nx;
      Y->data[i0].im = R_im / (double)nx;
    }
  }

  i0 = (int)floor((double)Rbody->size[0] / 2.0);
  if (1 > i0) {
    loop_ub = 0;
  } else {
    loop_ub = i0;
  }

  emxInit_creal_T(&Fheartrate, 2);
  nx = Y->size[1];
  i0 = Fheartrate->size[0] * Fheartrate->size[1];
  Fheartrate->size[0] = loop_ub;
  Fheartrate->size[1] = nx;
  emxEnsureCapacity((emxArray__common *)Fheartrate, i0, (int)sizeof(creal_T));
  for (i0 = 0; i0 < nx; i0++) {
    for (calclen = 0; calclen < loop_ub; calclen++) {
      Fheartrate->data[calclen + Fheartrate->size[0] * i0] = Y->data[calclen +
        Y->size[0] * i0];
    }
  }

  /*  [freq, Fbreathing] = plot_power_spectrum(Abody, 60); */
  /*  [freq, Fheartrate] = plot_power_spectrum(Ahead_filt, 60); */
  loop_ub = Fbreathing->size[1];
  for (i0 = 0; i0 < loop_ub; i0++) {
    Fbreathing->data[Fbreathing->size[0] * i0].re = 0.0;
    Fbreathing->data[Fbreathing->size[0] * i0].im = 0.0;
  }

  loop_ub = Y->size[1];
  for (i0 = 0; i0 < loop_ub; i0++) {
    Fheartrate->data[Fheartrate->size[0] * i0].re = 0.0;
    Fheartrate->data[Fheartrate->size[0] * i0].im = 0.0;
  }

  emxInit_creal_T(&b_Fheartrate, 2);

  /*  Fbreathing(1,:) = zeros(1, size(Fbreathing,2)); */
  /*  Fheartrate(1,:) = zeros(1, size(Fheartrate,2)); */
  i0 = b_Fheartrate->size[0] * b_Fheartrate->size[1];
  b_Fheartrate->size[0] = Fheartrate->size[0];
  b_Fheartrate->size[1] = Fheartrate->size[1];
  emxEnsureCapacity((emxArray__common *)b_Fheartrate, i0, (int)sizeof(creal_T));
  loop_ub = Fheartrate->size[0] * Fheartrate->size[1];
  for (i0 = 0; i0 < loop_ub; i0++) {
    b_Fheartrate->data[i0] = Fheartrate->data[i0];
  }

  emxFree_creal_T(&Fheartrate);
  emxInit_creal_T(&HRentr, 2);
  get_spectral_entropy(b_Fheartrate, HRentr);
  i0 = HRentr->size[0] * HRentr->size[1];
  HRentr->size[0] = 1;
  emxEnsureCapacity((emxArray__common *)HRentr, i0, (int)sizeof(creal_T));
  calclen = HRentr->size[0];
  sz_idx_0 = HRentr->size[1];
  loop_ub = calclen * sz_idx_0;
  emxFree_creal_T(&b_Fheartrate);
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

  emxInit_creal_T(&b_Fbreathing, 2);
  c_Fbreathing = sum(HRentr);
  rdivide(b_HRentr, c_Fbreathing, HRentr);
  i0 = b_Fbreathing->size[0] * b_Fbreathing->size[1];
  b_Fbreathing->size[0] = Fbreathing->size[0];
  b_Fbreathing->size[1] = Fbreathing->size[1];
  emxEnsureCapacity((emxArray__common *)b_Fbreathing, i0, (int)sizeof(creal_T));
  loop_ub = Fbreathing->size[0] * Fbreathing->size[1];
  emxFree_creal_T(&b_HRentr);
  for (i0 = 0; i0 < loop_ub; i0++) {
    b_Fbreathing->data[i0] = Fbreathing->data[i0];
  }

  emxFree_creal_T(&Fbreathing);
  emxInit_creal_T(&RRentr, 2);
  get_spectral_entropy(b_Fbreathing, RRentr);
  i0 = RRentr->size[0] * RRentr->size[1];
  RRentr->size[0] = 1;
  emxEnsureCapacity((emxArray__common *)RRentr, i0, (int)sizeof(creal_T));
  calclen = RRentr->size[0];
  sz_idx_0 = RRentr->size[1];
  loop_ub = calclen * sz_idx_0;
  emxFree_creal_T(&b_Fbreathing);
  for (i0 = 0; i0 < loop_ub; i0++) {
    RRentr->data[i0].re = 1.0 - RRentr->data[i0].re;
    RRentr->data[i0].im = 0.0 - RRentr->data[i0].im;
  }

  emxInit_creal_T(&b_RRentr, 2);
  i0 = b_RRentr->size[0] * b_RRentr->size[1];
  b_RRentr->size[0] = 1;
  b_RRentr->size[1] = RRentr->size[1];
  emxEnsureCapacity((emxArray__common *)b_RRentr, i0, (int)sizeof(creal_T));
  loop_ub = RRentr->size[0] * RRentr->size[1];
  for (i0 = 0; i0 < loop_ub; i0++) {
    b_RRentr->data[i0] = RRentr->data[i0];
  }

  c_Fbreathing = sum(RRentr);
  rdivide(b_RRentr, c_Fbreathing, RRentr);

  /*  Use fft to get rid of phase differences between signals. */
  b_fft(Rbody, Y);
  b_abs(Y, Rbody);
  loop_ub = Rbody->size[1];
  emxFree_creal_T(&b_RRentr);
  emxFree_creal_T(&Y);
  for (i0 = 0; i0 < loop_ub; i0++) {
    Rbody->data[Rbody->size[0] * i0] = 0.0;
  }

  emxInit_creal_T1(&b_Rbody, 1);
  emxInit_creal_T(&c_Rbody, 2);
  i0 = c_Rbody->size[0] * c_Rbody->size[1];
  c_Rbody->size[0] = Rbody->size[0];
  c_Rbody->size[1] = Rbody->size[1];
  emxEnsureCapacity((emxArray__common *)c_Rbody, i0, (int)sizeof(creal_T));
  loop_ub = Rbody->size[1];
  for (i0 = 0; i0 < loop_ub; i0++) {
    nx = Rbody->size[0];
    for (calclen = 0; calclen < nx; calclen++) {
      c_Rbody->data[calclen + c_Rbody->size[0] * i0].re = Rbody->data[calclen +
        Rbody->size[0] * i0];
      c_Rbody->data[calclen + c_Rbody->size[0] * i0].im = 0.0;
    }
  }

  emxFree_real_T(&Rbody);
  i0 = b_Rbody->size[0];
  b_Rbody->size[0] = c_Rbody->size[0];
  emxEnsureCapacity((emxArray__common *)b_Rbody, i0, (int)sizeof(creal_T));
  loop_ub = c_Rbody->size[0];
  for (i0 = 0; i0 < loop_ub; i0++) {
    b_Rbody->data[i0].re = 0.0;
    b_Rbody->data[i0].im = 0.0;
    nx = c_Rbody->size[1];
    for (calclen = 0; calclen < nx; calclen++) {
      y = RRentr->data[RRentr->size[0] * calclen].re;
      R_im = -RRentr->data[RRentr->size[0] * calclen].im;
      R_re = c_Rbody->data[i0 + c_Rbody->size[0] * calclen].re * y -
        c_Rbody->data[i0 + c_Rbody->size[0] * calclen].im * R_im;
      y = c_Rbody->data[i0 + c_Rbody->size[0] * calclen].re * R_im +
        c_Rbody->data[i0 + c_Rbody->size[0] * calclen].im * y;
      b_Rbody->data[i0].re += R_re;
      b_Rbody->data[i0].im += y;
    }
  }

  emxFree_creal_T(&c_Rbody);
  emxFree_creal_T(&RRentr);
  emxInit_creal_T1(&breathing, 1);
  emxInit_creal_T(&b_Ahead_filt, 2);
  ifft(b_Rbody, breathing);
  i0 = b_Ahead_filt->size[0] * b_Ahead_filt->size[1];
  b_Ahead_filt->size[0] = Ahead_filt->size[0];
  b_Ahead_filt->size[1] = Ahead_filt->size[1];
  emxEnsureCapacity((emxArray__common *)b_Ahead_filt, i0, (int)sizeof(creal_T));
  loop_ub = Ahead_filt->size[1];
  emxFree_creal_T(&b_Rbody);
  for (i0 = 0; i0 < loop_ub; i0++) {
    nx = Ahead_filt->size[0];
    for (calclen = 0; calclen < nx; calclen++) {
      b_Ahead_filt->data[calclen + b_Ahead_filt->size[0] * i0].re =
        Ahead_filt->data[calclen + Ahead_filt->size[0] * i0];
      b_Ahead_filt->data[calclen + b_Ahead_filt->size[0] * i0].im = 0.0;
    }
  }

  emxInit_creal_T1(&R, 1);
  i0 = R->size[0];
  R->size[0] = b_Ahead_filt->size[0];
  emxEnsureCapacity((emxArray__common *)R, i0, (int)sizeof(creal_T));
  loop_ub = b_Ahead_filt->size[0];
  for (i0 = 0; i0 < loop_ub; i0++) {
    R_re = 0.0;
    im = 0.0;
    nx = b_Ahead_filt->size[1];
    for (calclen = 0; calclen < nx; calclen++) {
      y = HRentr->data[HRentr->size[0] * calclen].re;
      R_im = -HRentr->data[HRentr->size[0] * calclen].im;
      Ahead_filt_re = b_Ahead_filt->data[i0 + b_Ahead_filt->size[0] * calclen].
        re * y - b_Ahead_filt->data[i0 + b_Ahead_filt->size[0] * calclen].im *
        R_im;
      y = b_Ahead_filt->data[i0 + b_Ahead_filt->size[0] * calclen].re * R_im +
        b_Ahead_filt->data[i0 + b_Ahead_filt->size[0] * calclen].im * y;
      R_re += Ahead_filt_re;
      im += y;
    }

    R->data[i0].re = -R_re;
    R->data[i0].im = -im;
  }

  emxFree_creal_T(&b_Ahead_filt);
  b_exp(R);
  i0 = R->size[0];
  emxEnsureCapacity((emxArray__common *)R, i0, (int)sizeof(creal_T));
  loop_ub = R->size[0];
  for (i0 = 0; i0 < loop_ub; i0++) {
    R->data[i0].re--;
  }

  emxInit_creal_T(&b_R, 2);
  i0 = b_R->size[0] * b_R->size[1];
  b_R->size[0] = 1;
  b_R->size[1] = R->size[0];
  emxEnsureCapacity((emxArray__common *)b_R, i0, (int)sizeof(creal_T));
  loop_ub = R->size[0];
  for (i0 = 0; i0 < loop_ub; i0++) {
    b_R->data[b_R->size[0] * i0].re = R->data[i0].re;
    b_R->data[b_R->size[0] * i0].im = -R->data[i0].im;
  }

  emxInit_real_T1(&xhat, 2);
  kalmanfilt(b_R, xhat);
  i0 = xhat->size[0] * xhat->size[1];
  xhat->size[0] = 1;
  emxEnsureCapacity((emxArray__common *)xhat, i0, (int)sizeof(double));
  calclen = xhat->size[0];
  sz_idx_0 = xhat->size[1];
  loop_ub = calclen * sz_idx_0;
  emxFree_creal_T(&b_R);
  for (i0 = 0; i0 < loop_ub; i0++) {
    xhat->data[i0]++;
  }

  emxInit_real_T1(&heartrate, 2);
  d_log(xhat);
  i0 = heartrate->size[0] * heartrate->size[1];
  heartrate->size[0] = 1;
  heartrate->size[1] = xhat->size[1];
  emxEnsureCapacity((emxArray__common *)heartrate, i0, (int)sizeof(double));
  loop_ub = xhat->size[0] * xhat->size[1];
  for (i0 = 0; i0 < loop_ub; i0++) {
    heartrate->data[i0] = -xhat->data[i0];
  }

  calclen = 1;
  sz_idx_0 = HRentr->size[1];
  mtmp = HRentr->data[0];
  b_loop_ub = 0;
  if (HRentr->size[1] > 1) {
    if (rtIsNaN(HRentr->data[0].re) || rtIsNaN(HRentr->data[0].im)) {
      nx = 1;
      exitg3 = false;
      while ((!exitg3) && (nx + 1 <= sz_idx_0)) {
        calclen = nx + 1;
        if (!(rtIsNaN(HRentr->data[nx].re) || rtIsNaN(HRentr->data[nx].im))) {
          mtmp = HRentr->data[nx];
          b_loop_ub = nx;
          exitg3 = true;
        } else {
          nx++;
        }
      }
    }

    if (calclen < HRentr->size[1]) {
      while (calclen + 1 <= sz_idx_0) {
        c_Fbreathing = HRentr->data[calclen];
        if (relop(c_Fbreathing, mtmp)) {
          mtmp = HRentr->data[calclen];
          b_loop_ub = calclen;
        }

        calclen++;
      }
    }
  }

  emxFree_creal_T(&HRentr);
  emxInit_real_T2(&b_x, 1);
  loop_ub = Ahead_filt->size[0];
  i0 = b_x->size[0];
  b_x->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)b_x, i0, (int)sizeof(double));
  for (i0 = 0; i0 < loop_ub; i0++) {
    b_x->data[i0] = -Ahead_filt->data[i0 + Ahead_filt->size[0] * b_loop_ub];
  }

  emxFree_real_T(&Ahead_filt);
  emxInit_real_T2(&c_x, 1);
  i0 = c_x->size[0];
  c_x->size[0] = b_x->size[0];
  emxEnsureCapacity((emxArray__common *)c_x, i0, (int)sizeof(double));
  loop_ub = b_x->size[0];
  for (i0 = 0; i0 < loop_ub; i0++) {
    c_x->data[i0] = b_x->data[i0];
  }

  for (calclen = 0; calclen + 1 <= b_x->size[0]; calclen++) {
    c_x->data[calclen] = exp(c_x->data[calclen]);
  }

  emxFree_real_T(&b_x);
  emxInit_real_T1(&d_x, 2);
  i0 = d_x->size[0] * d_x->size[1];
  d_x->size[0] = 1;
  d_x->size[1] = c_x->size[0];
  emxEnsureCapacity((emxArray__common *)d_x, i0, (int)sizeof(double));
  loop_ub = c_x->size[0];
  for (i0 = 0; i0 < loop_ub; i0++) {
    d_x->data[d_x->size[0] * i0] = c_x->data[i0] - 1.0;
  }

  emxFree_real_T(&c_x);
  emxInit_real_T1(&b_xhat, 2);
  b_kalmanfilt(d_x, xhat);
  i0 = b_xhat->size[0] * b_xhat->size[1];
  b_xhat->size[0] = 1;
  b_xhat->size[1] = xhat->size[1];
  emxEnsureCapacity((emxArray__common *)b_xhat, i0, (int)sizeof(double));
  loop_ub = xhat->size[0] * xhat->size[1];
  emxFree_real_T(&d_x);
  for (i0 = 0; i0 < loop_ub; i0++) {
    b_xhat->data[i0] = xhat->data[i0] + 1.0;
  }

  emxInit_creal_T1(&d_Fbreathing, 1);
  emxInit_real_T1(&c_xhat, 2);
  b_log(b_xhat, c_xhat);

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
  plot_power_spectrum(breathing, xhat, d_Fbreathing);

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
  b_heartrate[0] = heartrate->size[1];
  calclen = heartrate->size[1];
  c_heartrate = *heartrate;
  c_heartrate.size = (int *)&b_heartrate;
  c_heartrate.numDimensions = 1;
  d_fft(&c_heartrate, calclen, R);

  /*  power = (a^2+b^2)/N */
  /*  Should be same as 1/N*abs(Y)^2 */
  calclen = heartrate->size[1];
  i0 = R->size[0];
  emxEnsureCapacity((emxArray__common *)R, i0, (int)sizeof(creal_T));
  loop_ub = R->size[0];
  emxFree_real_T(&c_xhat);
  emxFree_real_T(&b_xhat);
  for (i0 = 0; i0 < loop_ub; i0++) {
    y = R->data[i0].re;
    R_im = -R->data[i0].im;
    R_re = R->data[i0].re * y - R->data[i0].im * R_im;
    R_im = R->data[i0].re * R_im + R->data[i0].im * y;
    if (R_im == 0.0) {
      R->data[i0].re = R_re / (double)calclen;
      R->data[i0].im = 0.0;
    } else if (R_re == 0.0) {
      R->data[i0].re = 0.0;
      R->data[i0].im = R_im / (double)calclen;
    } else {
      R->data[i0].re = R_re / (double)calclen;
      R->data[i0].im = R_im / (double)calclen;
    }
  }

  calclen = heartrate->size[1];
  y = 60.0 / (double)calclen;
  calclen = heartrate->size[1];
  calclen = (int)floor((double)calclen / 2.0);
  if (calclen - 1 < 0) {
    i0 = xhat->size[0] * xhat->size[1];
    xhat->size[0] = 1;
    xhat->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)xhat, i0, (int)sizeof(double));
  } else {
    i0 = xhat->size[0] * xhat->size[1];
    xhat->size[0] = 1;
    xhat->size[1] = calclen;
    emxEnsureCapacity((emxArray__common *)xhat, i0, (int)sizeof(double));
    loop_ub = calclen - 1;
    for (i0 = 0; i0 <= loop_ub; i0++) {
      xhat->data[xhat->size[0] * i0] = i0;
    }
  }

  i0 = xhat->size[0] * xhat->size[1];
  xhat->size[0] = 1;
  emxEnsureCapacity((emxArray__common *)xhat, i0, (int)sizeof(double));
  calclen = xhat->size[0];
  sz_idx_0 = xhat->size[1];
  loop_ub = calclen * sz_idx_0;
  for (i0 = 0; i0 < loop_ub; i0++) {
    xhat->data[i0] *= y;
  }

  /*  nth bin is n*Fs/N, +1 for index offset */
  calclen = heartrate->size[1];
  i0 = (int)floor((double)calclen / 2.0);
  emxFree_real_T(&heartrate);
  if (1 > i0) {
    loop_ub = 0;
  } else {
    loop_ub = i0;
  }

  i0 = breathing->size[0];
  breathing->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)breathing, i0, (int)sizeof(creal_T));
  for (i0 = 0; i0 < loop_ub; i0++) {
    breathing->data[i0] = R->data[i0];
  }

  emxFree_creal_T(&R);
  d_Fbreathing->data[0].re = 0.0;
  d_Fbreathing->data[0].im = 0.0;
  breathing->data[0].re = 0.0;
  breathing->data[0].im = 0.0;
  calclen = 1;
  sz_idx_0 = breathing->size[0];
  mtmp = breathing->data[0];
  b_loop_ub = 0;
  if (breathing->size[0] > 1) {
    if (rtIsNaN(breathing->data[0].re) || rtIsNaN(breathing->data[0].im)) {
      nx = 2;
      exitg2 = false;
      while ((!exitg2) && (nx <= sz_idx_0)) {
        calclen = nx;
        if (!(rtIsNaN(breathing->data[nx - 1].re) || rtIsNaN(breathing->data[nx
              - 1].im))) {
          mtmp = breathing->data[nx - 1];
          b_loop_ub = nx - 1;
          exitg2 = true;
        } else {
          nx++;
        }
      }
    }

    if (calclen < breathing->size[0]) {
      while (calclen + 1 <= sz_idx_0) {
        c_Fbreathing = breathing->data[calclen];
        if (relop(c_Fbreathing, mtmp)) {
          mtmp = breathing->data[calclen];
          b_loop_ub = calclen;
        }

        calclen++;
      }
    }
  }

  emxFree_creal_T(&breathing);
  *hr = xhat->data[b_loop_ub];
  calclen = 1;
  sz_idx_0 = d_Fbreathing->size[0];
  mtmp = d_Fbreathing->data[0];
  b_loop_ub = 0;
  if (d_Fbreathing->size[0] > 1) {
    if (rtIsNaN(d_Fbreathing->data[0].re) || rtIsNaN(d_Fbreathing->data[0].im))
    {
      nx = 2;
      exitg1 = false;
      while ((!exitg1) && (nx <= sz_idx_0)) {
        calclen = nx;
        if (!(rtIsNaN(d_Fbreathing->data[nx - 1].re) || rtIsNaN
              (d_Fbreathing->data[nx - 1].im))) {
          mtmp = d_Fbreathing->data[nx - 1];
          b_loop_ub = nx - 1;
          exitg1 = true;
        } else {
          nx++;
        }
      }
    }

    if (calclen < d_Fbreathing->size[0]) {
      while (calclen + 1 <= sz_idx_0) {
        c_Fbreathing = d_Fbreathing->data[calclen];
        if (relop(c_Fbreathing, mtmp)) {
          mtmp = d_Fbreathing->data[calclen];
          b_loop_ub = calclen;
        }

        calclen++;
      }
    }
  }

  emxFree_creal_T(&d_Fbreathing);
  *rr = xhat->data[b_loop_ub];

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
  emxFree_real_T(&xhat);
}

/*
 * File trailer for extract_vitals.c
 *
 * [EOF]
 */
