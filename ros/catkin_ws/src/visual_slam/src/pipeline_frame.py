#!/usr/bin/env python

import rospy
from icp.srv import *

import numpy as np
import pointclouds as pc

from pipeline_image import PipelineImage
from pipeline_geometric import PipelineGeometric
from frame import Frame
import tools

class PipelineFrame:
    def __init__(self):
        self.pi = PipelineImage()
        self.pg = PipelineGeometric()
        self.frame_id = 0
    
    def set_calibration(self, intrinsic, distortion):
        self.pg.set_calibration(intrinsic, distortion)
    
    def create_frame(self, image, xyz=[]):
        mask = self.pi.createMask(xyz)
        kpts, descs = self.pi.features(image, mask)
        p3ds = self.pi.get_p3ds(kpts, xyz)
        frame = Frame(self.frame_id, kpts, descs, p3ds)

        self.frame_id = self.frame_id + 1
        #print "New frame with " + str(len(kpts)) + " points."
        #print "Got " + str(len(p3ds)) + " p3ds."
        return frame
    
    def match(self, frame, obj):
        #matches = self.pi.match(frame.descs, obj.descs, ratio_test=False)
        matches = self.pi.match(frame.descs, obj.descs)
        frame.set_matches(matches)
        return len(matches)
    
    def motion(self, frame, obj):
        #print "Object has " + str(len(obj.p3ds)) + " p3ds."
        T = np.empty((3,4), np.float32)
        
        p2ds, p3ds = tools.aligned_2d3d_points(frame.kpts, obj.p3ds, frame.matches)
        inliers, T = self.pg.image_to_structure(p3ds, p2ds)
        
        frame.set_matches(inliers)
        frame.set_motion(T)
        
        return len(inliers)
        
    def structure(self, frame, obj):
        p2ds_frame, p2ds_object = self.aligned_2d2d_points(frame.kpts, obj.kpts, frame.matches)
        structure = self.pg.triangulate(p2ds_frame, p2ds_object)
        frame.set_structure(structure)
    
    # Correct frame's motion by comparing it with obj data
    def refine(self, frame, obj):
        # Express frame p3ds in global frame
        frame.p3ds = tools.transform(frame.p3ds, np.linalg.inv(frame.T))
        # call for ICP service        
        rospy.wait_for_service('icp')
        T = np.empty((3,4), np.float32)
        print "Calling service"
        try:
            ficp = rospy.ServiceProxy('icp', ICP)
            pcframe3d = pc.xyz_array_to_pointcloud2(frame.p3ds)
            pcobj3d = pc.xyz_array_to_pointcloud2(obj.p3ds)
            resp = ficp(pcframe3d, pcobj3d)
            T = tools.pose2array(resp.pose)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        # Set new motion to frame
        #frame.set_motion(T * frame.T) #
        frame.set_motion(frame.T * T) # a l'air bon
        # Refine p3ds positions
        frame.p3ds = tools.transform(frame.p3ds, np.linalg.inv(frame.T))
    
    
    