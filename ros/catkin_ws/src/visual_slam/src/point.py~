#!/usr/bin/env python

import numpy as np

class Point:
    def __init__(self, kf_idx, kpts, desc):
        self.kf_idx = [kf_idx]
        self.p2ds = kpts.pt
        self.descs = desc
        self.p3d = np.zeros((1,3),dtype=np.float32)
        self.3d_set = False
        
    def add_feature(self, kf_idx, kpts, desc):
        self.kf_idx.append(kf_idx)
        self.p2ds = np.concatenate((self.p2ds, kpts.pt), 0)
        self.descs = np.concatenate((self.descs, descs), 0)

    def set_p3d(self, p3d):
        self.p3d = p3d
        self.3d_set = True
        
    def is3d(self):
        return self.3d_set:
