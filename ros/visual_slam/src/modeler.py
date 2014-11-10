#!/usr/bin/env python

class Modeler:
    def __init__(self):
        self.pf = PipelineFrame()
        self.object = Object()
        self.inliers_threshold = 0.0
    
    def process(self, image):
        frame = self.pf.create_frame(image)
        # First frame
        if len(self.object.keyframes) == 0:
            self.object.add_keyframe(frame)
            return
        # Match frame with object
        num_inliers = self.pf.match(frame, self.object)
        # Need new frame ?
        if num_inliers < self.inliers_threshold:
            self.pf.motion(self.previous_frame, self.object)
            self.pf.structure(self.previous_frame, self.object)
            self.object.add_keyframe(self.previous_frame)
        # Save frame
        self.previous_frame = frame
        

    
