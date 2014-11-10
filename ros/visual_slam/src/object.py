#!/usr/bin/env python

class Object:
    def __init__(self):
        self.keyframes = []
    
    def add_keyframe(self, frame):
        self.kpts = self.kpts + frame.kpts
        self.descs = np.concatenate((self.descs, frame.descs), 0)
        #self.p2ds = np.concatenate((self.p2ds, frame.p2ds), 0)
        self.p3ds = np.concatenate((self.p3ds, frame.p3ds), 0)
        self.keyframes.append(frame)