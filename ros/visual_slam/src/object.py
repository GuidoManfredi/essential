#!/usr/bin/env python

import numpy as np

import tools

class Object:
    def __init__(self):
        self.keyframes = []
        self.points = []
        self.descs = np.zeros((1,128),dtype=np.float32)
        self.p3ds = np.zeros((1,3),dtype=np.float32)
    
    def add_keyframe(self, frame):
        if len(self.keyframes) == 0:
            self.add_first_keyframe(frame)
        else:
            self.add_other_keyframe(frame)
    
    def add_first_keyframe(self, frame):
        #for p3d, idx in enumerate(frame.p3ds):
            #pt = Point(frame.kpts[idx], frame.descs[idx])
            #pt.set_p3d(p3d)
            #self.points.append(pt)
        self.descs = frame.descs
        self.p3ds = frame.p3ds
        self.keyframes.append(frame)
        
    def add_other_keyframe(self, frame):
        # multiply structure by transform to express it in first frame
        #frame.p3ds = tools.transform(frame.p3ds, np.linalg.inv(frame.T))
        self.descs = np.concatenate((self.descs, frame.descs), 0)
        self.p3ds = np.concatenate((self.p3ds, frame.p3ds), 0)
        self.keyframes.append(frame)
        
    def getT(self):
        Ts = [kf.T for kf in self.keyframes]
        return Ts
        
    def get_structure(self):
        return self.p3ds
        
    def covis(self):
        n = len(self.points)
        ncam = len(self.keyframes)
        covis = np.zeros((n,ncam,2),dtype=np.float32)
        covis_mask = np.zeros((n,ncam),dtype=np.int8)
        
        for pt,npt in enumerate(self.points):
            for p2d,i in enumerate(pt.p2ds):
                nkf = pt.kf_idx[i]
                self.covis[npt,nkf,0] = p2d[0]
                self.covis[npt,nkf,1] = p2d[1]
                self.covis_mask[npt,nkf] = 1
                
        return covis, covis_mask
        
        
        
        
