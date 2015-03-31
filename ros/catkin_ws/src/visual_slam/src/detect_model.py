#!/usr/bin/env python

import rospy
import tf

import pointclouds as pc
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import CameraInfo

from modeler import Modeler

import cv2

detecter = Detecter()
camera_info = CameraInfo()
calibrated = False

broadcaster = tf.TransformBroadcaster()

def cb_calib(camera_info_msg):
    global calibrated
    if not calibrated:
        detecter.set_calibration(camera_info_msg.K, camera_info_msg.D)
        calibrated = True

def cb_pointcloud(cloud_msg):
    pointcloud = pc.pointcloud2_to_array(cloud_msg, split_rgb=True)
    
    xyz = pc.get_xyz_points(pointcloud)
    bgr = pc.get_bgr_points(pointcloud)

    global detecter
    currentT = detecter.process(bgr, xyz, True)

    global broadcaster
    broadcaster.sendTransform(currentT[0:3,3], tf.transformations.quaternion_from_matrix(currentT), rospy.Time.now(), "world", "/current_camera")

def main():
    rospy.init_node('create_model')
    rospy.Subscriber("camera/depth_registered/points", PointCloud2, cb_pointcloud)
    rospy.Subscriber("camera/rgb/camera_info", CameraInfo, cb_calib)

    rospy.spin()
    #rate = rospy.Rate(10)
    #while not rospy.is_shutdown():
    #    rate.sleep()

if __name__ == "__main__":
    main()
    

