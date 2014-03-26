import sys, getopt
import numpy as np
import cv2
import cv2.cv as cv

class HandDetector:
    def __init__ (self, cascade_file):
        self.cascade = cascade_file;
        self.classifier = cv2.CascadeClassifier (self.cascade)
        self.accumulator = np.zeros ((480, 640, 1), np.uint8)
        self.tracking = False;

    def detect(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        #gray = cv2.equalizeHist(gray)
        rects = self.classifier.detectMultiScale(gray, scaleFactor=1.5, minNeighbors=4, minSize=(30, 30), flags = cv.CV_HAAR_SCALE_IMAGE)
        if len(rects) == 0:
            tracking = False
            self.resetAcc()
            return []
        rects[:,2:] += rects[:,:2]
        
        self.drawRectsFull (self.accumulator, rects, (255, 255, 255))
        tracking = True
        
        return rects

    def drawRects(self, img, rects, color):
        for x1, y1, x2, y2 in rects:
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
            
    def drawRectsFull(self, img, rects, color):
        for x1, y1, x2, y2 in rects:
            cv2.rectangle(img, (x1, y1), (x2, y2), color, -1)
    
    def resetAcc (self):
        self.accumulator = np.zeros ((480, 640, 1), np.uint8)

help_message = '''
USAGE: facedetect.py [--cascade <cascade_fn>] [<video_source>]
'''

if __name__ == '__main__':
    print help_message
    
    args, video_src = getopt.getopt(sys.argv[1:], '', ['cascade='])
    try: video_src = video_src[0]
    except: video_src = 0

    args = dict(args)
    cascade_file = args.get('--cascade', "/home/gmanfred/devel/datasets/cascades/hands/poing.xml")
    #cascade_file = args.get('--cascade', "/home/gmanfred/devel/datasets/cascades/hands/palm.xml")
    hd = HandDetector (cascade_file)
    
    cam = cv2.VideoCapture(0)
    while True:
        ret, img = cam.read()
        
        rects = hd.detect(img)
        vis = img.copy()
        hd.drawRects(vis, rects, (0, 255, 0))
        cv2.imshow('Detection', vis)
        cv2.imshow('Accumulator', hd.accumulator)

        if 0xFF & cv2.waitKey(5) == 27:
            break
    cv2.destroyAllWindows()

