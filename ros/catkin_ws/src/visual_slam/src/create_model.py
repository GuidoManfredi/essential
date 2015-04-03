#!/usr/bin/env python

import rospy
import tf

import pointclouds as pc
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import CameraInfo

from modeler import Modeler

import cv2

modeler = Modeler()
camera_info = CameraInfo()
calibrated = False

broadcaster = tf.TransformBroadcaster()

# TODO 
#   Verifier que le produit entre la frame raffinee T et la frame.T est dans le bon sens.
#   Mettre des pourcents de matchs a atteindre, pas un seuil fixe.
#   Faire saveOBJ() avec 3D texturee (RGBD) pour visu.
#   Faire model et tester avec detecteur

def cb_calib(camera_info_msg):
    global calibrated
    if not calibrated:
        modeler.set_calibration(camera_info_msg.K, camera_info_msg.D)
        calibrated = True

def cb_pointcloud(cloud_msg):
    pointcloud = pc.pointcloud2_to_array(cloud_msg, split_rgb=True)
    
    xyz = pc.get_xyz_points(pointcloud)
    bgr = pc.get_bgr_points(pointcloud)
    
    #cv2.imshow("Debug", bgr)
    #cv2.waitKey(1)

    global modeler
    currentT = modeler.process(bgr, xyz, True)
    Ts = modeler.getT()
    global broadcaster
    # Publish current frame pose to TF
    broadcaster.sendTransform(currentT[0:3,3], tf.transformations.quaternion_from_matrix(currentT), rospy.Time.now(), "world", "/current_camera")
    # Publish keyframes poses to TF
    #for idx, T in enumerate(Ts):
        #broadcaster.sendTransform(T[0:3,3], tf.transformations.quaternion_from_matrix(T), rospy.Time.now(), "/world", "/camera" + str(idx))

def main():
    input_topic = "camera/depth_registered/points"
    rospy.init_node('create_model')
    rospy.Subscriber(input_topic, PointCloud2, cb_pointcloud)
    rospy.Subscriber("camera/rgb/camera_info", CameraInfo, cb_calib)
    #pub_pointcloud = rospy.Publisher("/structure", PointCloud2, queue_size=1)
    pub_pointcloud = rospy.Publisher("/structure", PointCloud2)

    #rospy.spin()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        p3ds = modeler.get_structure()
        if p3ds.shape[0] > 1:
            pointcloud_msg = pc.xyz_array_to_pointcloud2(p3ds, rospy.Time.now(), "/world")
            pub_pointcloud.publish(pointcloud_msg)
            rate.sleep()
        
        if cv2.waitKey(1) == 27: # 27 == esc
            break

    modeler.save('model')

if __name__ == "__main__":
    main()
    

