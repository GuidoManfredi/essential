#!/usr/bin/env python

import numpy as np
import scipy as sp

import cv2

class PipelineImage:
    def __init__(self):
        self.extractor = cv2.SIFT(500)
        self.matcher = cv2.BFMatcher()
        #self.matcher = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True) # not possible with distance ratio

    def features(self, image, mask):
        kpts, descs = self.extractor.detectAndCompute(image, mask, None)
        return kpts, descs

    def getCorrespondingP2DP3D(self, online_p3d, online_p2d, train_p2d, matches):
        p2d = []
        p3d = []
        for m in matches:
            y, x = online_p2d[m.queryIdx]
            #finite_array = np.isfinite(online_p3d[x][y])
            #finite = np.logical_and.reduce(finite_array, 0)
            #if finite: # Remove NaN points from matches
            #    print kikou
                #p3d.append(online_p3d[x][y])
                #p2d.append(train_p2d[m.trainIdx])
            p2d.append(train_p2d[m.trainIdx])
            p3d.append(online_p3d[x][y])
        return p2d, p3d

    def match(self, descs1, descs2, ratio_test=True):
        matches = self.matcher.knnMatch(np.asarray(descs1,np.float32),np.asarray(descs2,np.float32),k = 2)
        if ratio_test:
            good = []
            for [m,n] in matches:
                if (m.distance < 0.8 * n.distance) or (m.distance - n.distance < 1e-5):
                    #good.append([m]) # list of lists
                    good.append(m)
            return good

        matches = [m for m,n in matches]
        return matches
        
    def createMask(self, xyz):
        if len(xyz) == 0:
            return None

        # Filtering out NaN and Inf points
        finite = np.zeros(xyz.shape, dtype = np.uint8)
        np.isfinite(xyz, finite)

        finite_mask = np.zeros((xyz.shape[0],xyz.shape[1]), dtype = np.uint8)
        np.logical_and.reduce(finite, 2, np.uint8, finite_mask)

        # Filtering out parts we don't want to model
        x = -0.15
        X = 0.15
        y = -0.20
        Y = 0.15
        z = 0.40
        Z = 0.70
        # x,y and z filtered
        xyz_mask = ( (x < xyz[:,:,0]) & (xyz[:,:,0] < X)
                   & (y < xyz[:,:,1]) & (xyz[:,:,1] < Y)
                   & (z < xyz[:,:,2]) & (xyz[:,:,2] < Z) )

        final_mask = np.logical_and(finite_mask, xyz_mask)
        final_mask = np.asarray(final_mask,dtype=np.uint8)
        
        #display = final_mask * 255;
        #cv2.imshow('mask', display)
        #cv2.waitKey(1)
        
        return final_mask

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



