#!/usr/bin/env python

import numpy as np
import scipy as sp

import cv2

class PipelineImage:
    def __init__(self):
        self.extractor = cv2.SIFT()
        self.matcher = cv2.BFMatcher()
        #self.matcher = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True)

    def features(self, image, mask):
        kpts, descs = self.extractor.detectAndCompute(image, mask, None)
        return kpts, descs

    def getCorrespondingP2DP3D(self, online_p3d, online_p2d, train_p2d, matches):
        p3d = []
        p2d = []
        for m in matches:
            y, x = online_p2d[m.queryIdx]
            finite_array = np.isfinite(online_p3d[x][y])
            finite = np.logical_and.reduce(finite_array, 0)
            if finite: # Remove NaN points from matches
                p3d.append(online_p3d[x][y])
                p2d.append(train_p2d[m.trainIdx])
        return p2d, p3d

    def match(self, descs1, descs2, ratio_test=True):
        #self.matcher = cv2.BFMatcher() # Create a new matcher each time otherwise knnMatch bugs and returns always
                                        # two identical neighbours which screw up distance ratio (no matches).
        matches = self.matcher.knnMatch(np.asarray(descs1,np.float32),np.asarray(descs2,np.float32),k = 2)
        if ratio_test:
            good = []
            for [m,n] in matches:
                if m.distance < 0.75*n.distance:
                    #good.append([m]) # list of lists
                    good.append(m)
            return good

        matches = [m for m,n in matches]
        return matches
        
    def createMask(self, xyz):
        if len(xyz) == 0:
            return None

        #cv2.imshow('xyz', xyz)
        #cv2.waitKey(0)
        
        finite = np.zeros(xyz.shape, dtype = np.uint8)
        np.isfinite(xyz, finite)
        
        mask = np.zeros((xyz.shape[0],xyz.shape[1]), dtype = np.uint8)
        np.logical_and.reduce(finite, 2, np.uint8, mask)

        #mask = np.asarray(mask,dtype=np.uint8)
        #mask = mask * 255;        
        #cv2.imshow('mask', mask)
        #cv2.waitKey(0)
        
        return mask

    def get_p3ds(self, kpts, xyz):
        #for kpt in kpts:
         #   u = round(kpt.pt[1]);
          #  v = round(kpt.pt[0]);
           # finite_array = np.isfinite(xyz[u,v])
            #finite = np.logical_and.reduce(finite_array, 0)
            #if not finite:
             #   print kpt.pt
              #  print 'kikou'
        
        p3ds = np.float32([xyz[round(kpt.pt[1]),round(kpt.pt[0])] for kpt in kpts])
        #p3ds = np.float32([xyz[kpt.pt[1],kpt.pt[0]] for kpt in kpts])
        return p3ds
    
    def draw(self, img1, k1, img2, k2, matches):
        h1, w1 = img1.shape[:2]
        h2, w2 = img2.shape[:2]
        view = sp.zeros((max(h1, h2), w1 + w2, 3), sp.uint8)
        view[:h1, :w1, 0] = img1
        view[:h2, w1:, 0] = img2
        view[:, :, 1] = view[:, :, 0]
        view[:, :, 2] = view[:, :, 0]
        for m in matches:
            color = tuple([sp.random.randint(0, 255) for _ in xrange(3)])
            cv2.line(view, (int(k1[m.queryIdx].pt[0]), int(k1[m.queryIdx].pt[1])) , (int(k2[m.trainIdx].pt[0] + w1), int(k2[m.trainIdx].pt[1])), color)
        return view



