#!/usr/bin/env python
import roslib
roslib.load_manifest("perfect_model")

import time
import numpy as np
import cv2

import matplotlib.pyplot as plt

from modeler import Modeler

modeler = Modeler()

def main():
    print "OpenCV version " + cv2.__version__

    cap = cv2.VideoCapture(0)
    start = time.time()
    while(True):
        # Capture frame-by-frame
        ret, frame = cap.read()

        # Our operations on the frame come here
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        modeler.run(gray)
        num_newpoints_list, percent_inliers_list= modeler.stats()
        # Display the resulting frame
        cv2.imshow('frame',gray)
        # Exit if 'q' key pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            end = time.time()
            # Display the stats
            plt.plot(percent_inliers_list)
            plt.show()
            print "Modeling time: " + str(end - start)
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
