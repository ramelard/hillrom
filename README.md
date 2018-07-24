# hillrom

Heart rate and breathing rate extraction on TX2 board. To setup the TX2 board, run the following:

1. First enable ppa 3rd-party libraries on Ubuntu: link
2. Follow instructions here to install qtcam (​​​​call with sudo qtcam and test to see camera working)
3. Download, build, and install OpenCV for TX2
4. $ git clone https://github.com/ramelard/hillrom.git
5. Test CV2 by running
    1. `$ cd hillrom/src/`
    2. Adjust `video_capture = cv2.VideoCapture(<number>)` in temp.py depending on how many cameras connected. Sometimes TX2 comes with onboard camera; in this case, a USB camera will be index 1.
    3. `$ python ./temp.py`
    4. Assert your face is in the camera feed, and a green box surrounds it
6. All Matlab files have already been codegen'ed to C. All that needs to be done is to compile it into a shared library for the host architecture
    1. `$ cd hillrom/src/lib`
    2. `$ ./gensharedlib`
7. Finally, run the main script
    1. `$ cd hillrom/src/`
    2. Again, adjust `video_capture = cv2.VideoCapture(<number>)` in test.py
    3. `$ python ./test.py`

