import sys
import numpy as np

import cv2
from cv2 import cv

PACKAGE='opencv_display'
import roslib
roslib.load_manifest(PACKAGE)

import rospy
from opencv_display.msg import LocaPose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Visualizer:
    # adjustable parameters for axis length and thickness
    size = 100
    thickness = 5

    K = np.zeros((3,3), np.float32)
    img = np.zeros ((480, 640, 3), np.uint8)
    
    points = np.array([[0, 0, 0], [size, 0, 0], [0, size, 0], [0, 0, size]], np.float32).reshape (4, 3)
    projected_points = np.zeros ((4, 2), np.float32)

    
    def loadIntrinsic (self, filepath):
        self.K = np.array( cv2.cv.Load(filepath, cv.CreateMemStorage(), 'camera_matrix') )
    
    def setIntrinsic (self, K):
        self.K = K
        
    def updatePose (self, r, t):
        self.projected_points = self.project (r, t)
    
    def updateImage (self, img):
        self.img = np.copy(img)
    
    def draw(self):
        self.img = self.drawFrame (self.img, self.projected_points)
        cv2.imshow ('Localised!', self.img)
        cv2.waitKey(1)

    def project(self, r, t):
        d = np.array([])
        projected_points, jac = cv2.projectPoints(self.points, r, t, self.K, d)
        print projected_points
        return projected_points
    
    def drawFrame(self, img, corners):
        origin = tuple(corners[0].ravel())
        cv2.line(img, origin, tuple(corners[1].ravel()), (0,0,255), self.thickness)
        cv2.line(img, origin, tuple(corners[2].ravel()), (0,255,0), self.thickness)
        cv2.line(img, origin, tuple(corners[3].ravel()), (255,0,0), self.thickness)
        return img
################################################################################
vis = Visualizer ()
filepath = '/home/gmanfred/.ros/camera_info/webcam_gilgamesh_opencv.yml'
vis.loadIntrinsic (filepath)
bridge = CvBridge()

def msgToArray2 (msg):
    R = np.array([[msg.t00, msg.t01, msg.t02],
                 [msg.t10, msg.t11, msg.t12],
                 [msg.t20, msg.t21, msg.t22]])
    r, jcb = cv2.Rodrigues(R)
    t = np.array([msg.t03, msg.t13, msg.t23])
    return (r, t)

def image_callback (msg):
    try:
        cv_image = bridge.imgmsg_to_cv(msg, "bgr8")
    except CvBridgeError, e:
        print e
    image = np.asarray(cv_image)
    vis.updateImage (image)
    vis.draw()

def pose_callback(msg):
    r, t = msgToArray2 (msg)
    vis.updatePose(r, t)

def main():
    rospy.init_node('opencv_display_pose', anonymous=True)
    rospy.Subscriber('pose', LocaPose, pose_callback)
    rospy.Subscriber('image', Image, image_callback)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv2.destroyAllWindows()
    
if __name__ == '__main__':
    main()
