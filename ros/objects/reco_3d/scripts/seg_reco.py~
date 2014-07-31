#!/usr/bin/env python
import roslib; roslib.load_manifest('reco_3d')

import sys
import rospy
from segment_plans_objects.srv import *
from reco_3d.srv import *

segmentation_service_name = '/segmentation'
recognition_service_name = '/recognition_obb'

def segmentation_recognition ():
	clusters = segmentation_client()
	names = list()
	for cluster in clusters:
		names.append(recognition_client (cluster));
	return names

def segmentation_client():
  print "Waiting for segmentation service"
  rospy.wait_for_service(segmentation_service_name)
  print "Calling segmentation..."
  try:
    segment = rospy.ServiceProxy(segmentation_service_name, PlantopSegmentation)
    res = segment()
    return res.clusters
  except rospy.ServiceException, e:
    print "Segmentation service call failed: %s" % e

def recognition_client(cluster):
  print "Waiting for obb recognition service"
  rospy.wait_for_service(recognition_service_name)
  print "Calling recognition..."
  try:
    recognize_obb = rospy.ServiceProxy(recognition_service_name, OrientedBoundingBoxRecognition)
    names = list()
    res = recognize_obb(cluster, names)
    return res.names[0]
  except rospy.ServiceException, e:
    print "Segmentation service call failed: %s" % e

if __name__ == "__main__":
  names = segmentation_recognition()
  print names
