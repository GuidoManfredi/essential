#!/usr/bin/env python

#import cv2

from frame import Frame
from object import Object
from pipeline_frame import PipelineFrame

class Detecter:
    def __init__(self):
        self.pf = PipelineFrame()
        self.object = Object()
        self.inliers_threshold = 75.0

    def set_calibration(self, intrinsic, distortion):
        self.pf.set_calibration(intrinsic, distortion)
    
    def process(self, image, xyz=[], show_debug=False):
        frame = self.pf.create_frame(image, xyz)
        return self.process_frame(frame)
        
    def process_frame(self, frame):        
        # Match frame with object
        num_inliers = self.pf.match(frame, self.object)
        print "Frame " + str(frame.id) + " has " + str(num_inliers) + " matches."
        # Compute Motion
        num_inliers = self.pf.motion(frame, self.object)
        print "Frame " + str(frame.id) + " has " + str(num_inliers) + " inliers."
        
        return frame.T
    
    def load(self, filename):
        self.object.load(filename)
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
        
