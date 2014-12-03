#!/usr/bin/env python

# TODO finir classe pipeline_geometric
# TODO filtrer matches en fonction de fondamentale (mask and correctMatches)
# TODO filtrer matches en fonction de resultats triangulation ( enlever les Z negatifs)
# TODO calculer erreur de reprojection
# TODO trouver données de test et faire classes de test

import roslib
roslib.load_manifest("visual_slam")

import numpy as np
from numpy.linalg import svd

import cv2

broadcaster = tf.TransformBroadcaster()

def cb_pointcloud(cloud_msg):
    global obj
    global broadcaster
    
    pointcloud = pc.pointcloud2_to_array(cloud_msg, split_rgb=True)
    
    xyz = pc.get_xyz_points(pointcloud, remove_nans=False)
    bgr = pc.get_bgr_points(pointcloud)
    
    T = detector.process(bgr, xyz, obj, show_debug=False)
    broadcaster.sendTransform(T[0:3,3], tf.transformations.quaternion_from_matrix(T),
                                rospy.Time.now(),
                                "/lait", "/world")
    print T

def main():
    rospy.init_node('visual slam')
    rospy.Subscriber("camera/depth_registered/points", PointCloud2, cb_pointcloud)
    rospy.spin()

if __name__ == "__main__":
    main()
    
    
    
