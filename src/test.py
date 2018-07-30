import threading
from Queue import Queue
import numpy as np
# from extract_vitals import butter_bandpass
from extract_vitals import extract_vitals
import matplotlib.pyplot as plt
import cv2
from time import time, strftime, localtime
import logging
import scipy.io

logging.basicConfig(level=logging.DEBUG,
                    format='(%(threadName)-10s) %(message)s',
                    )
face_frames = []
body_frames = []
face_roi = []  # shared between threads


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
            logging.debug('Found face!')
            (x, y, w, h) = faces[0, :]
            face_roi = (x, y, w, h)
          else:
            logging.warning('Could not detect any faces.')
        except:
          logging.warning('Error detecting faces.')
      lock.release()

    if face_roi:
      (x, y, w, h) = face_roi
      head_hgt = int(np.floor(h*1.5))
      x = int(np.floor(x))
      w = int(np.floor(w))
      y = int(np.floor(y))
      subframe = gray_64F[y:y + head_hgt, x:x + w]
      # if not face_frames:
      #   face_frames = subframe
      # else:
      #   face_frames = np.dstack((face_frames, subframe))
      face_frames.append((subframe, t))
      subframe = gray_64F[y+head_hgt:(y+head_hgt)+head_hgt*2, x:x + w]
      body_frames.append((subframe, t))
      # if not body_frames:
      #   body_frames = subframe
      # else:
      #   body_frames = np.dstack((body_frames, subframe))

    q.task_done()


def track_and_display():
  video_capture = cv2.VideoCapture(1)

  lock = threading.Lock()

  q = Queue()
  num_workers = 5
  for i in xrange(num_workers):
    t = threading.Thread(target=worker, args=(q, lock,))
    # t = threading.Timer(3, extract_vitals)
    t.daemon = True
    t.start()

  tstart = time()
  nframes = 0
  # TODO: need better resolution?
  logging.warning('Frames seem to be 480x640; do not match ecam viewer settings. Does this mean exposure time is not the same too??')
  logging.debug('Starting frame acquisition')
  while time()-tstart < 10:
    # Capture frame-by-frame
    ret, frame = video_capture.read()
    t0 = time()

    # Put into our processing queue, to be read by extract_vitals
    q.put((frame, t0))
    nframes += 1

    t1 = time()
    # logging.debug('%g fps', 1./min(1e-6,(t1-t0)))

    # # Display the resulting frame
    # cv2.imshow('Video', frame)


    # if cv2.waitKey(1) & 0xFF == ord('q'):
      # break

  logging.debug('Acquisition speed: %.2f fps', nframes/(time()-tstart))
  # When everything is done, release the capture
  video_capture.release()
  cv2.destroyAllWindows()

  q.join()  # block until all tasks are done
  logging.debug('Processing speed: %.2f fps', nframes / (time() - tstart))

  tlast = tstart
  # Need to sort since these may have been added out of order by workers.
  face_frames.sort(key=lambda x:x[1])
  body_frames.sort(key=lambda x:x[1])
  #for (frame, t) in face_frames:
  for i in xrange(len(face_frames)):
    face = face_frames[i][0]
    body = body_frames[i][0]
    # frame,t = face_frames.get()
    cv2.imshow('face', face)
    cv2.imshow('body', body)
    # print t-tlast
    # tlast = t
    cv2.waitKey(32)

  faces = [frame for (frame, t) in face_frames]
  bodys = [frame for (frame, t) in body_frames]
  block_size = 6
  (hr, rr) = extract_vitals(np.dstack(faces), np.dstack(bodys), block_size)

  fileout = 'frames_%s-hr%u-rr%u' % (strftime('%H%M'), hr, rr)
  logging.debug('Writing frames to ./output/%s', fileout)
  write_frames_to_mat(faces, bodys, fileout)

  save_unix_time_string = strftime('%Y-%m-%d %H:%M:%S', localtime(time()))

  f = open('./output/hr.txt', 'a+')
  f.write('Heart Rate @ %s: \t %s \n' % (save_unix_time_string, hr))
  f.close()

  f = open('./output/rr.txt', 'a+')
  f.write('Respitory Rate @ %s: \t %s \n' % (save_unix_time_string, rr))
  f.close()


def write_frames_to_mat(face_frames, body_frames, filename='out'):
  fullfile = './output/%s.mat' % filename
  logging.debug('Saving frames to .mat file...')
  t0 = time()
  scipy.io.savemat(fullfile, {'face_frames': face_frames, 'body_frames': body_frames})
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
