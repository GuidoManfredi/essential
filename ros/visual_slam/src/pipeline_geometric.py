#!/usr/bin/env python

import cv2
import numpy as np
from numpy.linalg import svd

class PipelineGeometric:
    def __init__(self):
    
    def image_to_structure(self, p2ds, p3ds):
    
    def image_to_image(self, p2ds1, p2ds2):
        if p2ds1.shape[1] < 8:
            return numpy.empty((1,2), np.float32), numpy.empty((3,4), np.float32)
    
        motion, mask = motion_from_fundamental(self, p2ds1, p2ds2)
        # Select only inlier points
        pts1 = pts1[mask.ravel()==1]
        pts2 = pts2[mask.ravel()==1]
        
    def motion_from_fundamental(self, p2ds1, p2ds2):
        n1, n2 = normalize_points(p2ds1, p2ds2)
    
        F, mask = cv2.findFundamentalMat(p2ds1,p2ds2,cv2.FM_LMEDS)
        E1 = self.K.T * F * self.K
        u1, s1, v1 = np.linalg.svd(E1)
        v1t = np.transpose(v1)
        # cf. http://www.robots.ox.ac.uk/~vgg/hzbook/hzbook2/HZepipolar.pdf
        E2 = u * np.diag([1,1,0]) * v1t
        u2, s2, v2 = np.linalg.svd(E2)
        
        W = np.float32([0,-1,0, 1,0,0, 0,0,1]);
        Wt = np.float32[0,1,0, -1,0,0, 0,0,1]);
        
        v2t = np.transpose(v2)
        R1 = u2 * W * v2t
        R2 = u2 * Wt * v2t
        t1 = u2.col(2)
        t2 = -u2.col(2)
        
        return , mask

    def normalize_points(p2ds1, p2ds2):
        return self.invK * p2ds1, self.invK * p2ds2
