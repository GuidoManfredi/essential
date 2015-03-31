#!/usr/bin/env python

import numpy as np

class Frame:
    def __init__(self, frame_id, kpts, descs, p3ds):
        self.id = frame_id
        self.kpts = kpts
        self.descs = descs
        self.p2ds = np.float32([kpt.pt for kpt in kpts])
        self.p3ds = p3ds
        self.T = np.eye(4, dtype=np.float32)

    def set_matches(self, matches):
        self.matches = matches

    def set_motion(self, T):
        self.T = T
        
    def set_structure(self, p3ds):
        self.p3ds = p3ds
        
