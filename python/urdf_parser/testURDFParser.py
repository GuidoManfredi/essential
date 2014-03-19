#!/usr/bin/env python

from dynamic_graph.sot.pr2.dynamic_pr2 import DynamicPr2
import numpy as np

pr2_urdf_file = "/opt/ros/electric/stacks/pr2_mechanism/pr2_mechanism_model/pr2.urdf"
dyn = DynamicPr2('dyn')
dyn.load(pr2_urdf_file)


#referenceJoint = 'base_footprint_joint'
#joint = 'r_shoulder_pan_joint'
#dyn.signal(referenceJoint).recompute(dyn.signal(referenceJoint).time+1)
#dyn.signal(referenceJoint).value
#dyn.signal(joint).value
#dyn.signal(joint).recompute(dyn.signal(joint).time+1)

#jointT = np.matrix(dyn.signal(joint).value)
#referenceT = np.matrix(dyn.signal(referenceJoint).value)

#T = jointT * referenceT

#print T
