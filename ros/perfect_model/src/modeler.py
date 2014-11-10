#!/usr/bin/env python
import numpy as np
import scipy as sp

import cv2

class Modeler:
    def __init__(self):
        self.extractor = cv2.SIFT()
        self.matcher = cv2.BFMatcher()

        self.threshold = 10.0 # in percents of number of keypoints in input view
        self.initialised = False
        #self.previous_kpts = []
        #self.previous_descs = np.empty((1,256))
        #self.previous_num_newpoints = 0
        self.num_newpoints_list = []
        self.percent_inliers_list = []

    def run (self, frame):
        if not self.initialised:
            print "Initialising model"
            self.init(frame)
        else:
            self.process(frame)

    def stats (self):
        return self.num_newpoints_list, self.percent_inliers_list

    def init (self, frame):
        self.model_kpts, self.model_descs = self.features(frame)
        self.num_newpoints_list.append(len(self.model_kpts))
        self.save_view(self.model_kpts, self.model_descs, len(self.model_kpts))
        self.initialised = True
        
    def process (self, image):
        kpts, descs = self.features(image)
        
        matches = self.match(descs, self.model_descs, True)
        num_inliers = self.filter_matches(kpts, self.model_kpts, matches)
        print "Found " + str(num_inliers) + "/" + str(len(matches)) + "/" + str(len(kpts)) + " inliers."

        percent_inliers = float(num_inliers) / float(len(kpts)) * 100
        self.percent_inliers_list.append(percent_inliers)
        if percent_inliers < self.threshold:
            self.add_view(self.previous_kpts, self.previous_descs, self.previous_num_newpoints) 
           
        self.save_view(kpts, descs, len(kpts) - num_inliers)

    def add_view (self, kpts, descs, num_newpoints):
        print "Adding new view, with " + str(self.previous_num_newpoints) + " new points."
        self.model_kpts = self.model_kpts + kpts
        self.model_descs = np.concatenate((self.model_descs, descs), 0)
        self.num_newpoints_list.append(num_newpoints)

    def save_view (self, kpts, descs, num_newpoints):
        self.previous_kpts = kpts
        self.previous_descs = descs
        self.previous_num_newpoints = num_newpoints

    def features (self, image):
        kpts, descs = self.extractor.detectAndCompute(image, None)
        return kpts, descs
    
    def match (self, descs1, descs2, ratio_test=True):
        if descs1.size == 0 or descs2.size == 0:
            return []
            
        matches = self.matcher.knnMatch(np.asarray(descs1,np.float32),np.asarray(descs2,np.float32),k = 2)
        if ratio_test:
            #print "Using ratio test"
            good = []
            for m,n in matches:
                if m.distance < 0.75*n.distance:
                    good.append(m)
            return good
        return matches

    def filter_matches (self, kpts1, kpts2, matches):
        if len(matches) < 8:
            return 0
    
        pts1 = []
        pts2 = []
        for m in matches:
            pts1.append(kpts1[m.queryIdx].pt)
            pts2.append(kpts2[m.trainIdx].pt)
        pts1 = np.float32(pts1)
        pts2 = np.float32(pts2)
        F, mask = cv2.findFundamentalMat(pts1,pts2,cv2.FM_LMEDS)
        # We select only inlier points
        pts1 = pts1[mask.ravel()==1]
        pts2 = pts2[mask.ravel()==1]
        return len(pts1)
        
        
