#!/usr/bin/env python

import cv2
import numpy as np
from numpy.linalg import svd

import tools

class PipelineGeometric:
    def __init__(self):
        self.K = None
        self.d = None
        
    def set_calibration(self, intrinsic, distortion):
        self.K = np.reshape(intrinsic, (3,3))
        self.invK = np.linalg.inv(self.K)
        self.d = distortion
        
    def image_to_structure(self, p3ds, p2ds):
        #r, t, inliers = cv2.solvePnPRansac(p3ds, p2ds, self.K, self.d)
        r = np.empty((3,1), np.float32)
        t = np.empty((3,1), np.float32)
        inliers = np.empty(p2ds.shape, np.float32)
        r, t, inliers = cv2.solvePnPRansac(p3ds, p2ds, self.K, self.d, r, t, False, 500, 5.0, 100, inliers, cv2.EPNP)
        #p2ds = p2ds[inliers]
        #p3ds = p3ds[inliers]
        #cv2.solvePnP(p3ds, p2ds, self.K, self.d, r, t, True)
        R, jacobian = cv2.Rodrigues(r)
        return inliers, tools.Rt2P(R, t)
        
    def triangulate(self, P1, P2, p2ds1, p2ds2):
        n1 = np.float32(np.transpose(p2ds1))
        n2 = np.float32(np.transpose(p2ds2))
        hp3ds = cv2.triangulatePoints(P1, P2, n1, n2)
        hp3ds = np.transpose(hp3ds)
        return self.from_homogeneous(hp3ds)
        
    def reprojection_error(self, P, p3ds, p2ds):
        R = P[np.ix_([0,1,2],[0,1,2])]
        rvec, jac = cv2.Rodrigues(R)
        tvec = P[np.ix_([0,1,2],[3])]

        estimated_p2ds, jac = cv2.projectPoints(p3ds, rvec, tvec, self.K, None)
        estimated_p2ds = np.squeeze(estimated_p2ds)
        errors = np.linalg.norm(p2ds - estimated_p2ds, axis=1)

        return np.mean(errors)
        
        
        
        
        
