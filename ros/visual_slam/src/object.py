#!/usr/bin/env python

import numpy as np

import tools

class Object:
    def __init__(self):
        self.keyframes = []
        self.p3ds = np.zeros((1,3),dtype=np.float32)
    
    def add_keyframe(self, frame):
        if len(self.keyframes) == 0:
            self.add_first_keyframe(frame)
        else:
            self.add_other_keyframe(frame)
    
    def add_first_keyframe(self, frame):
        self.kpts = frame.kpts
        self.descs = frame.descs
        self.p2ds = frame.p2ds
        self.p3ds = frame.p3ds
        self.keyframes.append(frame)
        
    def add_other_keyframe(self, frame):
        self.kpts = self.kpts + frame.kpts
        self.descs = np.concatenate((self.descs, frame.descs), 0)
        self.p2ds = np.concatenate((self.p2ds, frame.p2ds), 0)
        # multiply structure by transform to express it in first frame
        frame.p3ds = tools.transform(frame.p3ds, np.linalg.inv(frame.T))
        self.p3ds = np.concatenate((self.p3ds, frame.p3ds), 0)
        
        self.keyframes.append(frame)
        
    def getT(self):
        Ts = [kf.T for kf in self.keyframes]
        return Ts
        
    def get_structure(self):
        return self.p3ds
