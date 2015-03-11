#!/usr/bin/env python

import numpy as np

import vtk

from pipeline_image import PipelineImage
from pipeline_geometric import PipelineGeometric
from frame import Frame
import tools
import sba

class PipelineFrame:
    def __init__(self):
        self.pi = PipelineImage()
        self.pg = PipelineGeometric()
        self.frame_id = 0
        self.icp = vtk.vtkIterativeClosestPointTransform()
        self.icpTransformFilter = vtk.vtkTransformPolyDataFilter()
    
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
        pipo = True;
        #T = self.icp(frame, obj)
        #frame.set_motion(T)
    
    # use ICP to refine frame motion w.r.t the 3-D point cloud of the object.
    def icp(self, frame, obj):
        source = createVtkData(frame.p3ds):
        target = createVtkData(obj.p3ds):
        self.icp.SetSource(source)
        self.icp.SetTarget(target)
        self.icp.GetLandmarkTransform().SetModeToRigidBody()
        #icp.DebugOn()
        self.icp.SetMaximumNumberOfIterations(20)
        self.icp.StartByMatchingCentroidsOn()
        self.icp.Modified()
        self.icp.Update()
        
        if vtk.VTK_MAJOR_VERSION <= 5:
            self.icpTransformFilter.SetInput(source)
        else:
            self.icpTransformFilter.SetInputData(source)
 
        self.icpTransformFilter.SetTransform(icp)
        self.icpTransformFilter.Update()
        transformedSource = icpTransformFilter.GetOutput()

        print frame.p3ds.shape
        point_count = frame.p3ds.shape(2)
        for index in range(pointCount):
            point = [0,0,0]
            transformedSource.GetPoint(index, point)
            frame.p3ds[i] = point
    
    def bundle_adjust(self, obj, window_size):
        cameras = self.bundle_cameras(obj, window_size)
        points = self.bundle_points(obj, window_size)
        newcams, newpoints, info = sba.SparseBundleAdjust(cameras, points)
        #print info # error
        obj.set_cameras(newcams)
        obj.set_points(newcams)
    
    def bundle_cameras(self, obj, window_size):
    
    def bundle_points(self, obj, window_size):
    
    
    
    
