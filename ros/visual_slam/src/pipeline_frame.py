#!/usr/bin/env python

import numpy as np

from pipeline_image import PipelineImage
from pipeline_geometric3 import PipelineGeometric
from frame import Frame
import tools

class PipelineFrame:
    def __init__(self):
        self.pi = PipelineImage()
        self.pg = PipelineGeometric()
        self.frame_id = 0
    
    def create_frame(self, image, xyz=[]):
        mask = self.pi.createMask(xyz)
        kpts, descs = self.pi.features(image, mask)
        p3ds = self.pi.get_p3ds(kpts, xyz)
        frame = Frame(self.frame_id, kpts, descs, p3ds)

        self.frame_id = self.frame_id + 1
        print "New frame with " + str(len(kpts)) + " points."
        #print "Got " + str(len(p3ds)) + " p3ds."
        return frame
    
    def match(self, frame, obj):
        matches = self.pi.match(frame.descs, obj.descs)
        frame.set_matches(matches)
        return len(matches)
    
    def motion(self, frame, obj):
        print "Object has " + str(len(obj.p3ds)) + " p3ds."
        T = np.empty((3,4), np.float32)
        
        p2ds, p3ds = tools.aligned_2d3d_points(frame.kpts, obj.p3ds, frame.matches)
        inliers, T = self.pg.image_to_structure(frame.p2ds, obj.p3ds)
        
        frame.set_matches(inliers)
        frame.set_motion(T)
        
    def structure(self, frame, obj):
        p2ds_frame, p2ds_object = self.aligned_2d2d_points(frame.kpts, obj.kpts, frame.matches)
        structure = self.pg.triangulate(p2ds_frame, p2ds_object)
        frame.set_structure(structure)
    
    
