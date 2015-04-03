#!/usr/bin/env python

#import cv2

from frame import Frame
from object import Object
from pipeline_frame import PipelineFrame

class Modeler:
    def __init__(self):
        self.pf = PipelineFrame()
        self.object = Object()
        self.inliers_threshold = 30.0
        self.counter = 0

    def set_calibration(self, intrinsic, distortion):
        self.pf.set_calibration(intrinsic, distortion)
    
    def process(self, image, xyz=[], show_debug=False):
        #cv2.imwrite('image' + counter + '.png', image)
        #self.counter = self.counter + 1
        frame = self.pf.create_frame(image, xyz)
        return self.process_frame(frame)
        
    def process_frame(self, frame):
        # First frame
        if len(self.object.keyframes) == 0:
            self.object.add_keyframe(frame)
            print "Added new keyframe."
            self.previous_frame = frame
            return frame.T
        
        # Match frame with object
        num_inliers = self.pf.match(frame, self.object)
        print "Frame " + str(frame.id) + " has " + str(num_inliers) + " matches."
        # Compute Motion
        num_inliers = self.pf.motion(frame, self.object)
        print "Frame " + str(frame.id) + " has " + str(num_inliers) + " inliers."
        
        # Need new keyframe ?
        if num_inliers < self.inliers_threshold:
            print "Object has " + str(len(self.object.p3ds)) + " p3ds."
            #num_inliers = self.pf.motion(self.previous_frame, self.object)
            #if len(self.previous_frame.p3ds) == 0: # no depth sensor, triangulation needed
                #self.pf.structure(self.previous_frame, self.object)
            #print len(self.previous_frame.p3ds)
            self.pf.refine(self.previous_frame, self.object)
            self.object.add_keyframe(self.previous_frame)
            print "Added new keyframe."
        
        # Save frame
        self.previous_frame = frame
        
        return frame.T

    def getT(self):
        return self.object.getT()
        
    def get_structure(self):
        return self.object.get_structure()
    
    def save(self, filename):
        self.object.save(filename)
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
        
