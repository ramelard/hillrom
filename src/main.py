import sys
import cv2
import logging
import scipy.io
import threading
import numpy as np

from time import time, strftime, localtime
from extract_vitals import extract_vitals

# if sys.platform == 'linux' or sys.platform == 'linux2':
#   from Queue import Queue
# elif sys.platform == 'darwin':
#   from multiprocessing import Queue

if sys.version_info[0] < 3:
  from Queue import Queue
  range_function = xrange
else:
  from queue import Queue
  range_function = range

logging.basicConfig(level=logging.DEBUG,
                    format='(%(threadName)-10s) %(message)s',
                    )
face_frames = []
body_frames = []
face_roi = [] # shared between threads


def stack_uneven(arrays, fill_value=0.):
    '''
    Fits arrays into a single numpy array, even if they are
    different sizes. `fill_value` is the default value.

    Args:
            arrays: list of np arrays of various sizes
                (must be same rank, but not necessarily same size)
            fill_value (float, optional):

    Returns:
            np.ndarray
    '''
    sizes = [a.shape for a in arrays]
    max_sizes = np.max(list(zip(*sizes)), -1)
    # The resultant array has stacked on the first dimension
    result = np.full((len(arrays),) + tuple(max_sizes), fill_value)
    for i, a in enumerate(arrays):
      # The shape of this array `a`, turned into slices
      slices = tuple(slice(0,s) for s in sizes[i])
      # Overwrite a block slice of `result` with this array `a`
      result[i][slices] = a
    return result


def worker(q, lock):
  # TODO: have to worry about thread safety? atomic blocks?
  global face_roi
  # TODO: Only finds face once (i.e. assumes they're motionless). Change this, and maybe add facial landmarks?
  while True:
    frame, t = q.get(block=True)
    # logging.debug('Extracted frame; q.size()=%u', q.qsize())
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray_64F = np.divide(gray, 255.)
    if not face_roi:
      # We only want 1 thread to find a face. So prohibit multiple threads from getting in here at the beginning.
      lock.acquire()
      if not face_roi:
        face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
        assert(not face_cascade.empty())
        try:
          faces = face_cascade.detectMultiScale(
              gray,
              scaleFactor=1.1,
              minNeighbors=5,
              minSize=(30, 30),
              flags=cv2.CASCADE_SCALE_IMAGE
          )
          if faces.any():
            logging.debug('Found face! %d items left in queue.' % q.qsize())
            (x, y, w, h) = faces[0, :]
            face_roi = (x, y, w, h)
          else:
            logging.warning('Could not detect any faces. %d items left in queue.' % q.qsize())
        except:
          logging.warning('Error detecting faces. %d items left in queue.' % q.qsize())
      lock.release()
    if face_roi:
      (x, y, w, h) = face_roi
      head_hgt = int(np.floor(h * 1.5))
      x = int(np.floor(x))
      w = int(np.floor(w))
      y = int(np.floor(y))
      face_subframe = gray_64F[y : y + head_hgt, x : x + w]
      body_subframe = gray_64F[y + head_hgt : (y + head_hgt) + head_hgt * 2, x : x + w]
      face_frames.append((face_subframe, t)) # multiple threads will append to this array simultaneously
      body_frames.append((body_subframe, t)) # multiple threads will append to this array simultaneously
    q.task_done()


def track_and_display():

  # ----------------------------------------------------------------------------
  #                                                                        Meta
  # ----------------------------------------------------------------------------
  # camera = cv2.VideoCapture(0)
  camera = cv2.VideoCapture(1)
  lock = threading.Lock()
  q = Queue()
  num_workers = 5
  acquisition_time = 20


  # ----------------------------------------------------------------------------
  #                                                               Frame Capture
  # ----------------------------------------------------------------------------
  # Read once before everything to initiate the camera.
  ret, frame = camera.read()
  tstart = time()
  nframes = 0
  # TODO: need better resolution?
  # logging.warning('[WARNING] Frames seem to be 480x640; do not match ecam viewer settings. Does this mean exposure time is not the same too??')
  logging.debug('[INFO] Starting frame acquisition')
  tlast = tstart
  while time() - tstart < acquisition_time:
    # Capture frame-by-frame
    ret, frame = camera.read()
    # Put into our processing queue, to be read by worker (in turn `extract_vitals.so` as the frames are added to queue).
    tnow = time()
    q.put((frame, tnow))
    nframes += 1
    
    if tlast-tstart > 2./30:
      logging.warning('Detected an unexpectedly large delay between frames (%gs)' % tlast-tstart)
  logging.debug('[INFO] Acquisition speed: %.2f fps', nframes / (time() - tstart))


  # ----------------------------------------------------------------------------
  #                                                            Frame Processing
  # ----------------------------------------------------------------------------
  tstart2 = time()
  logging.debug('[INFO] Simultanouesly processing frames added to queue via %d threads)' % num_workers)
  for i in range_function(num_workers):
    t = threading.Thread(target = worker, args = (q, lock))
    t.daemon = True
    t.start()
  q.join() # block until all tasks are done
  logging.debug('[INFO] Processing speed: %.2f fps', nframes / (time() - tstart2))


  # ----------------------------------------------------------------------------
  #                                                               Frame Sorting
  # ----------------------------------------------------------------------------
  # Need to sort since these may have been added out of order by workers.
  face_frames.sort(key=lambda x:x[1])
  body_frames.sort(key=lambda x:x[1])


  # ----------------------------------------------------------------------------
  #                                                               Demonstration
  # ----------------------------------------------------------------------------
  for i in range_function(len(face_frames)):
    face = face_frames[i][0]
    body = body_frames[i][0]
    cv2.imshow('face', face)
    cv2.imshow('body', body)
    cv2.waitKey(32)


  # ----------------------------------------------------------------------------
  #                                                              Extract Vitals
  # ----------------------------------------------------------------------------
  faces = [frame for (frame, t) in face_frames]
  bodys = [frame for (frame, t) in body_frames]
  timestamps = [t for (frame, t) in body_frames] # time is identical for faces and bodys as both are captured from same frame
  block_size = 4
  print(timestamps[-1] - tstart)
  try:
    (hr, rr) = extract_vitals( \
      np.dstack(faces), \
      np.dstack(bodys), \
      np.stack(timestamps), \
      block_size)
  except:
    logging.debug('[WARNING] Camera captured different size frames!')
    (hr, rr) = extract_vitals( \
      stack_uneven(faces), \
      stack_uneven(bodys), \
      np.stack(timestamps), \
      block_size)


  # ----------------------------------------------------------------------------
  #                                                                Save Outputs
  # ----------------------------------------------------------------------------
  fileout = 'frames_%s-hr%u-rr%u' % (strftime('%H%M'), hr, rr)
  logging.debug('Writing frames to ./output/%s', fileout)
  write_frames_to_mat(faces, bodys, timestamps, fileout)

  save_unix_time_string = strftime('%Y-%m-%d %H:%M:%S', localtime(time()))

  # Append results to ./output/hr.txt
  f = open('./output/hr.txt', 'a+')
  f.write('Heart Rate @ %s: \t %s \n' % (save_unix_time_string, hr))
  f.close()

  # Append results to ./output/rr.txt
  f = open('./output/rr.txt', 'a+')
  f.write('Respitory Rate @ %s: \t %s \n' % (save_unix_time_string, rr))
  f.close()

  # Capture 1 frame at the end of test and save
  ret, frame = camera.read()
  cv2.imwrite('./output/frame_%s.png' % save_unix_time_string, frame)

  # When everything is done, release the capture
  camera.release()
  cv2.destroyAllWindows()


def write_frames_to_mat(face_frames, body_frames, timestamps, filename='out'):
  fullfile = './output/%s.mat' % filename
  logging.debug('Saving frames to .mat file...')
  t0 = time()
  scipy.io.savemat(fullfile, {'face_frames': face_frames, 'body_frames': body_frames, 'timestamps': timestamps})
  logging.debug('Done (%.1fs)' % (time()-t0))


def write_video(frames, filename='out'):
  """Save frames to be analyzed in Matlab"""
  logging.warning('ffmpeg not installed. Cannot write video.')
  fullfile = './output/%s.avi' % filename
  out = cv2.VideoWriter(fullfile, -1, 30, frames[0].shape, False)
  for frame in frames:
    out.write(frame)
  out.release()




if __name__ == '__main__':
  track_and_display()
