#!/usr/bin/env python

import numpy as np

from pipeline_image import PipelineImage
from pipeline_geometric import PipelineGeometric
from frame import Frame

class PipelineFrame:
    def __init__(self):
        self.pi = PipelineImage()
        self.pg = PipelineGeometric()
        self.frame_id = 0
    
    def create_frame(self, image):
        kpts, descs = self.pi.features(image)
        frame = Frame(frame_id, kpts, descs)
        return frame
    
    def match(self, frame, obj):
        matches = self.pi.match(frame.descs, obj.descs)
        frame.set_matches(matches)
    
    def motion(self, frame, obj):
        T = np.empty((3,4), np.float32)
        
        if len(obj.keyframes) > 1:
            p2ds, p3ds = self.aligned_2d3d_points(frame.kpts, obj.p3ds, frame.matches)
            inliers, T = self.pg.image_to_structure(frame.p2ds, obj.p3ds)
        elif len(obj.keyframes) == 1:
            p2ds_frame, p2ds_object = self.aligned_2d2d_points(frame.kpts, obj.kpts, frame.matches)
            inliers, T = self.pg.image_to_image(frame.p2ds, obj.p2ds)
        else
            print "Error: structure_motion: Number of keyframes is negative or null, not possible!"
        
        frame.set_matches(inliers)
        frame.set_motion(T)
        
    def structure(self, frame, obj):
        p2ds_frame, p2ds_object = self.aligned_2d2d_points(frame.kpts, obj.kpts, frame.matches)
        structure = self.pg.triangulate(p2ds_frame, p2ds_object)
        frame.set_structure(structure)
        
    def aligned_2d2d_points(self, kpts1, kpts2, matches):
        p2ds1 = []
        p2ds2 = []
        for m in matches:
            p2ds1.append(kpts1[m.queryIdx].pt)
            p2ds2.append(kpts2[m.trainIdx].pt)
        p2ds1 = np.float32(p2ds1)
        p2ds2 = np.float32(p2ds2)
        return p2ds1, p2ds2
    
    def aligned_2d3d_points(self, kpts, p3ds, matches):
        p2ds = []
        p3ds = []
        for m in matches:
            p2ds.append(kpts[m.queryIdx].pt)
            p3ds.append(p3d[m.trainIdx].pt)
        p2ds = np.float32(p2ds)
        p3ds = np.float32(p3ds)
        return p2ds, p3ds
    
    