import cv2
import logging



if __name__ == '__main__':
  faceCascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
  video_capture = cv2.VideoCapture(1)
  while True:
    ret, frame = video_capture.read()

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    try:
      faces = faceCascade.detectMultiScale(
          gray,
          scaleFactor=1.1,
          minNeighbors=5,
          minSize=(30, 30),
          flags=cv2.CASCADE_SCALE_IMAGE
      )

      # Draw a rectangle around the faces
      for (x, y, w, h) in faces:
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
      cv2.imshow('Video', frame)
    except:
      logging.warning('Error detecting faces')

    if cv2.waitKey(1) & 0xFF == ord('q'):
      break
