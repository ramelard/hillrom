# import scipy
from scipy.signal import butter, lfilter
import numpy as np
from time import time
from math import floor

def extract_vitals(frames, block_size):
  # face detector
  # body = 
  # head = 
  
  (h,w,t) = frames.shape
  print (h,w,t)

  print '1'
  # print(cv2.getBuildInformation())  # See "Use Cuda" and "Use OpenCL"
  # im2 = cv2.resize(img, (0,0), fx=1/block_size, fy=1/blocksize, interpolation=INTER_AREA)
  
  Bbody = frames[0:h/2, 0:w/2, :]
  Rbody = np.reshape(np.moveaxis(Bbody, 2, 0), (t,-1))
  Abody = -np.log(1+Rbody)
  
  Bhead = frames[h/2+1:, w/2+1:, :]
  Rhead = np.reshape(np.moveaxis(Bhead, 2, 0), (t,-1))
  Ahead = -np.log(1+Rhead)
  
  # Get rid of breathing component in signal
  lowf = 30/60.
  highf = 350/60.
  fs = 60.
  Ahead_filt = np.zeros(Ahead.shape)
  # print('for...')
  # for i in xrange(Ahead.shape[1]):
    # Ahead_filt[:,i] = butter_bandpass(Ahead[:,i], fs, (lowf, highf))
  # print 'done'
  Ahead_filt = Ahead
    
  Fbreathing = power_spectrum(Abody, fs)
  Fheartrate = power_spectrum(Ahead_filt, fs)
  
  
def power_spectrum(sigmat, fs):
  npts = sigmat.shape[1]
  Y = np.fft.fft(sigmat)
  Pyy = Y * np.conj(Y) / npts
  
  last = int(floor(npts/2))
  freq = [fs/npts * fidx for fidx in range(0, last-1)]
  P = Pyy[:last,:]
  return (freq, P)
    

def butter_bandpass(sig, fs, bandpass):
  # fc - cutoff freq as a fraction of the sampling rate
  # fc = fc/fs
  # b = 0.08
  # N = sig.size  # number of elements
  # if not N % 2:
    # N += 1
  # n = np.arange(N)
  
  # sinc_func = np.sinc(2 * fc * (n - (N - 1) / 2.))
  # window = 0.42 - 0.5 * np.cos(2 * np.pi * n / (N - 1)) + b * np.cos(4 * np.pi * n / (N - 1))
  # sinc_func = sinc_func * window
  # sinc_func = sinc_func / np.sum(sinc_func)
  
  # sig_filt = np.convolve(sig, sinc_func, mode='same')
  
  low = bandpass[0]
  high = bandpass[1]
  nyq = 0.5 * fs
  b, a = butter(3, [low/nyq, high/nyq], btype='band')
  sig_filt = lfilter(b, a, sig)
  
  return sig_filt
  
if __name__ == '__main__':
  frames = np.ones((400,600,100))
  print 'Created frames'
  extract_vitals(frames, 6)