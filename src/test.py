import threading
from Queue import Queue
import numpy as np
from extract_vitals import butter_bandpass
import matplotlib.pyplot as plt
import cv2
from time import time
import logging

logging.basicConfig(level=logging.DEBUG,
                    format='(%(threadName)-10s) %(message)s',
                    )
face_frames = []
face_roi = []  # shared between threads


def worker(q, lock):
  # TODO: have to worry about thread safety? atomic blocks?
  global face_roi

  # TODO: Only finds face once (i.e. assumes they're motionless). Change this, and maybe add facial landmarks?
  while True:
    frame, t = q.get(block=True)
    # logging.debug('Extracted frame; q.size()=%u', q.qsize())

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

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
      subframe = gray[y:y + h, x:x + w]
      face_frames.append((subframe, t))
      
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
  while time()-tstart < 5:
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
  # Need to sort since these may have been added out of order by workers
  face_frames.sort(key=lambda x:x[1])
  for (frame, t) in face_frames:
    # frame,t = face_frames.get()
    cv2.imshow('video', frame)
    print t-tlast
    tlast = t
    cv2.waitKey(int(1./5*1000))

  
  
  
if __name__ == '__main__':
  track_and_display()
  # t = np.arange(0,3,1/100.)
  # pi = np.pi
  # sig = np.cos(2*pi*3*t) + np.cos(2*pi*7*t)
  # sig2 = butter_bandpass(sig, 100, 2,4)
  
  # plt.figure(1)
  # plt.subplot(211)
  # plt.plot(sig)
  # plt.subplot(212)
  # plt.plot(sig2)
  # plt.show()