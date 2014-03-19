#!/usr/bin/env python
import roslib; roslib.load_manifest('segment_plans_objects')

import sys
import rospy
from segment_plans_objects.srv import *

service_name = '/segmentation'

def segmentation_client():
    print "Waiting for segmentation service"
    rospy.wait_for_service(service_name)
    print "Calling segmentation..."
    try:
        segment = rospy.ServiceProxy(service_name, PlantopSegmentation)
        res = segment()
        return len(res.clusters)
    except rospy.ServiceException, e:
        print "Segmentation service call failed: %s" % e

if __name__ == "__main__":
    num_clusters = segmentation_client()
    print "Found %d clusters" % num_clusters
    
