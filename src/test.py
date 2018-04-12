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
face_roi = []


def worker(q):
  # TODO: speed this up by not doing facial tracking each time (xcorr? assume they're motionless?)
  while True:
    frame, t = q.get(block=True)
    logging.debug('Extracted frame; q.size()=%u', q.qsize())

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    if not face_roi:
      face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
      try:
        faces = face_cascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30, 30),
            flags=cv2.CASCADE_SCALE_IMAGE
        )

        if faces.any():
          (x, y, w, h) = faces[0, :]
          face_roi = (range(y,y+h+1), range(x,x+w+1))
        else:
          logging.warning('Could not detect any faces')
      except:
        logging.warning('Error detecting faces')
    else:
      pass

    subframe = gray[face_roi[0], face_roi[1]]
    face_frames.append((subframe, t))
      
    q.task_done()
    

def track_and_display():
  video_capture = cv2.VideoCapture(1)
  
  q = Queue()
  num_workers = 5
  for i in xrange(num_workers):
    t = threading.Thread(target=worker, args=(q,))
    # t = threading.Timer(3, extract_vitals)
    t.daemon = True
    t.start()

  tstart = time()
  nframes = 0
  while time()-tstart < 1:
    # Capture frame-by-frame
    ret, frame = video_capture.read()
    t0 = time()

    # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # try:
      # faces = faceCascade.detectMultiScale(
          # gray,
          # scaleFactor=1.1,
          # minNeighbors=5,
          # minSize=(30, 30),
          # flags=cv2.CASCADE_SCALE_IMAGE
      # )

      # # Draw a rectangle around the faces
      # for (x, y, w, h) in faces:
        # cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
    # except:
      # logging.warning('Error detecting faces')
      
    # Put into our processing queue, to be read by extract_vitals
    q.put((frame,t0))
    nframes += 1
    
    t1 = time()
    # logging.debug('%g fps', 1./min(1e-6,(t1-t0)))

    # # Display the resulting frame
    # cv2.imshow('Video', frame)
    

    # if cv2.waitKey(1) & 0xFF == ord('q'):
      # break

  logging.debug('Acquisition: %.2f fps', nframes/(time()-tstart))
  # When everything is done, release the capture
  video_capture.release()
  cv2.destroyAllWindows()
  
  q.join()  # block until all tasks are done
  logging.debug('Processing: %.2f fps', nframes / (time() - tstart))

  tlast = tstart
  # Need to sort since these may have been added out of order by workers
  face_frames.sort(key=lambda x:x[1])
  for (frame, t) in face_frames:
    # frame,t = face_frames.get()
    cv2.imshow('video',frame)
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