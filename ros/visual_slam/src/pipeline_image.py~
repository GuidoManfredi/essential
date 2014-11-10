#!/usr/bin/env python

import numpy as np

import cv2

class PipelineImage:
    def __init__(self):
        self.extractor = cv2.SIFT()
        self.matcher = cv2.BFMatcher()
        self.ratio = 0.75
            
    def features(self, image):
        if image.shape[2] is not 1
            print "Error: features: input must be grayscale image."
            return [], numpy.float32((1,128))

        kpts, descs = self.extractor.detectAndCompute(image, None)
        return kpts, descs
        
    def match(self, descs1, descs2, ratio_test=True):
        # Check if descriptors
        if descs1.size == 0 or descs2.size == 0:
            return []
        
        matches = self.matcher.knnMatch(np.asarray(descs1,np.float32),np.asarray(descs2,np.float32),k = 2)
        if ratio_test:
            good = []
            for m,n in matches:
                if m.distance < self.ratio * n.distance:
                    good.append(m)
            return good
        return matches
