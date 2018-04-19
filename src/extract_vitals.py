# import scipy
#from scipy.signal import butter, lfilter
import cv2
import numpy as np
from time import time
from math import floor
import logging
from emxtypes import emxarray_real_T
from ctypes import *

logging.basicConfig(level=logging.DEBUG,
                    format='(%(threadName)-10s) %(message)s',
                    )

mydll = cdll.LoadLibrary('./lib/extract_vitals_tk1.so')
mydll.rt_InitInfAndNaN(c_size_t(8))


def extract_vitals(frames_head, frames_body, block_size):
  """Transform frames (3D nparray) into emxArray_real_T and extract hr, rr"""

  c_block_size = c_double(block_size)
  c_fps = c_double(30)
  hr_out = c_double(3)
  rr_out = c_double(3)

  logging.debug('Converting frames to emxarray...')
  tstart = time()
  # Need to receive faces_flat and bodys_flat so the data in the emx pointers does not get freed.
  (head_emxarray_ptr, faces_flat) = framestack_to_emxarray(frames_head)
  (body_emxarray_ptr, bodys_flat) = framestack_to_emxarray(frames_body)
  logging.debug('Done (%.2fs)', time()-tstart)
  mydll.extract_vitals_tk1(head_emxarray_ptr, body_emxarray_ptr, c_fps, c_block_size,
                       byref(hr_out), byref(rr_out))
  logging.debug('HR=%.1f, RR=%.1f', hr_out.value, rr_out.value)

  return (hr_out.value, rr_out.value)


def framestack_to_emxarray(frames):
  # frames is a list of nparrays.
  (h, w, nframes) = frames.shape

  ndims_cint = c_int(3)
  dim_size = [h, w, nframes]
  dim_size_carr = (c_int * 3)(*dim_size)

  # Flatten column-wise to be compatible with Matlab-generated C code.
  frames_flat = frames.flatten(order='F')
  # Access the array as a pointer to double, as required by emxArray_real_T
  data_c = frames_flat.ctypes.data_as(POINTER(c_double))
  res = mydll.emxCreateWrapperND_real_T(data_c, ndims_cint, dim_size_carr)
  emxarray_ptr = cast(res, POINTER(emxarray_real_T))

  # # Test it out
  # F = emxarray_ptr.contents.data[0:h*w]
  # F2 = np.reshape(np.array(F, dtype=np.double), (h, w), order='F')
  # cv2.imshow('f', F2)
  # cv2.waitKey()

  # Need to return frames_flat so the memory doesn't get freed.
  return emxarray_ptr, frames_flat



# def extract_vitals(frames, block_size):
#   # face detector
#   # body =
#   # head =
#
#   (h,w,t) = frames.shape
#   print (h,w,t)
#
#   print '1'
#   # print(cv2.getBuildInformation())  # See "Use Cuda" and "Use OpenCL"
#   # im2 = cv2.resize(img, (0,0), fx=1/block_size, fy=1/blocksize, interpolation=INTER_AREA)
#
#   Bbody = frames[0:h/2, 0:w/2, :]
#   Rbody = np.reshape(np.moveaxis(Bbody, 2, 0), (t,-1))
#   Abody = -np.log(1+Rbody)
#
#   Bhead = frames[h/2+1:, w/2+1:, :]
#   Rhead = np.reshape(np.moveaxis(Bhead, 2, 0), (t,-1))
#   Ahead = -np.log(1+Rhead)
#
#   # Get rid of breathing component in signal
#   lowf = 30/60.
#   highf = 350/60.
#   fs = 60.
#   Ahead_filt = np.zeros(Ahead.shape)
#   # print('for...')
#   # for i in xrange(Ahead.shape[1]):
#     # Ahead_filt[:,i] = butter_bandpass(Ahead[:,i], fs, (lowf, highf))
#   # print 'done'
#   Ahead_filt = Ahead
#
#   Fbreathing = power_spectrum(Abody, fs)
#   Fheartrate = power_spectrum(Ahead_filt, fs)
#
#
# def power_spectrum(sigmat, fs):
#   npts = sigmat.shape[1]
#   Y = np.fft.fft(sigmat)
#   Pyy = Y * np.conj(Y) / npts
#
#   last = int(floor(npts/2))
#   freq = [fs/npts * fidx for fidx in range(0, last-1)]
#   P = Pyy[:last,:]
#   return (freq, P)
#
#
# def butter_bandpass(sig, fs, bandpass):
#   # fc - cutoff freq as a fraction of the sampling rate
#   # fc = fc/fs
#   # b = 0.08
#   # N = sig.size  # number of elements
#   # if not N % 2:
#     # N += 1
#   # n = np.arange(N)
#
#   # sinc_func = np.sinc(2 * fc * (n - (N - 1) / 2.))
#   # window = 0.42 - 0.5 * np.cos(2 * np.pi * n / (N - 1)) + b * np.cos(4 * np.pi * n / (N - 1))
#   # sinc_func = sinc_func * window
#   # sinc_func = sinc_func / np.sum(sinc_func)
#
#   # sig_filt = np.convolve(sig, sinc_func, mode='same')
#
#   low = bandpass[0]
#   high = bandpass[1]
#   nyq = 0.5 * fs
#   b, a = butter(3, [low/nyq, high/nyq], btype='band')
#   sig_filt = lfilter(b, a, sig)
#
#   return sig_filt
#
# if __name__ == '__main__':
#   frames = np.ones((400,600,100))
#   print 'Created frames'
#   extract_vitals(frames, 6)