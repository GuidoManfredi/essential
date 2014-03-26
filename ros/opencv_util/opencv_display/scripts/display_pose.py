import sys
import numpy as np

PACKAGE='opencv_display'
import roslib
roslib.load_manifest(PACKAGE)

import rospy
import cv2
from opencv_display.msg import LocaPose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Visualizer:
    K = np.zeros((3,3), np.float32)
    size = 1
    points = np.array([[0, 0, 0], [size, 0, 0], [0, size, 0], [0, 0, size]], np.float32).reshape (-1, 3)
    projected_points = np.zeros ((4, 2), np.float32)
    img = np.zeros ((480, 640, 3), np.uint8)
    
    def setIntrinsic (self, K):
        self.K = K

    def updatePose (self, P):
        self.projected_points = self.project (P)
        
    def cv_updatePose (self, r, t):
        self.projected_points = self.cv_project (r, t)
    
    def updateImage (self, img):
        self.img = np.copy(img)
    
    def draw(self):
        self.img = self.drawFrame (self.img, self.projected_points)
        cv2.imshow ('Localised!', self.img)
        cv2.waitKey(1)

    # project 3D points to image plane
    def cv_project(self, r, t):
        d = np.array([])
        projected_points, jac = cv2.projectPoints(self.points, r, t, self.K, d)
        return projected_points

    def project (self, P):
        projected_points = []
        for point in self.points:
            point.append(1) # homogeneous coordinates
            A = self.K.dot(P)
            projected_point = A.dot(point)
            if (projected_point[-1] != 0):
                projected_point /= projected_point[-1]
            projected_point = np.delete(projected_point, -1)
            projected_point = projected_point.astype(int)
            projected_points.append(projected_point)
        return projected_points
    
    def drawFrame(self, img, corners):
        cv2.line(img, tuple(corners[0].ravel()), tuple(corners[1].ravel()), (0,0,255), 3)
        cv2.line(img, tuple(corners[0].ravel()), tuple(corners[2].ravel()), (0,255,0), 3)
        cv2.line(img, tuple(corners[0].ravel()), tuple(corners[3].ravel()), (255,0,0), 3)
        #points = tuple(map(tuple, corners))
        #cv2.line(img, points[0], points[1], (0,0,255), 3)
        #cv2.line(img, points[0], points[2], (0,255,0), 3)
        #cv2.line(img, points[0], points[3], (255,0,0), 3)
        return img
################################################################################
K = np.float32([[741,   0,   331],
                [  0, 740.5, 244],
                [  0,   0,   1]]).reshape (-1, 3)
vis = Visualizer ()
vis.setIntrinsic (K)
bridge = CvBridge()

def msgToArray (msg):
    P = [[msg.t00, msg.t01, msg.t02, msg.t03],
         [msg.t10, msg.t11, msg.t12, msg.t03],
         [msg.t20, msg.t21, msg.t22, msg.t23],
         [      0,       0,       0,       1]]
    return P

def msgToArray2 (msg):
    R = np.array([[msg.t00, msg.t01, msg.t02],
                 [msg.t10, msg.t11, msg.t12],
                 [msg.t20, msg.t21, msg.t22]])
    r, jcb = cv2.Rodrigues(R)
    t = np.array([msg.t03, msg.t13, msg.t23])
    return (r, t)

def pose_callback(msg):
    #P = msgToArray (msg)
    #vis.updatePose (P)
    r, t = msgToArray2 (msg)
    vis.cv_updatePose(r, t)

def image_callback (msg):
    try:
        cv_image = bridge.imgmsg_to_cv(msg, "bgr8")
    except CvBridgeError, e:
        print e
    image = np.asarray(cv_image)
    vis.updateImage (image)
    vis.draw()

def main():
    rospy.init_node('opencv_display', anonymous=True)
    rospy.Subscriber("pose", LocaPose, pose_callback)
    rospy.Subscriber("image", Image, image_callback)

    # spin() simply keeps python from exiting until this node is stopped
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv2.destroyAllWindows()
    
if __name__ == '__main__':
    main()