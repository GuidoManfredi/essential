import sys
import numpy as np

import rospy
import cv2

class Visualizer:
    K = np.zeros((3,4), np.float32)
    size = 50
    points = tuple([[0, 0, 0], [size, 0, 0], [0, size, 0], [0, 0, size]])
    
    def setIntrinsic (self, K):
        self.K = K

    def draw(self, img, P):
        projected_points = self.project (P)
        img = self.drawFrame (img, projected_points)
        return img

    def project (self, P):
        projected_points = []
        print self.points
        for point in self.points:
            point.append(1) # homogeneous coordinates
            A = self.K.dot(P)
            print A
            print point
            projected_point = A.dot(point)
            if (projected_point[-1] != 0):
                projected_point /= projected_point[-1]
            projected_point = np.delete(projected_point, -1)
            projected_point = projected_point.astype(int)
            projected_points.append(projected_point)
        return projected_points
    
    def drawFrame(self, img, corners):
        points = tuple(map(tuple, corners))
        cv2.line(img, points[0], points[1], (0,0,255), 3)
        cv2.line(img, points[0], points[2], (0,255,0), 3)
        cv2.line(img, points[0], points[3], (255,0,0), 3)
        return img

def main(argv=None):
    K = np.float32([[800,   0, 320, 0],
                    [  0, 800, 240, 0],
                    [  0,   0,   1, 0]]).reshape (-1, 4)
    P = np.float32([[  1,   0,   0,    0],
                    [  0,   1,   0,    0],
                    [  0,   0,   1,  300],
                    [  0,   0,   0,    1]]).reshape (-1, 4)    
    img = cv2.imread ('/home/gmanfred/Pictures/nesquik_facile.jpg')
    vis = Visualizer ()
    vis.setIntrinsic (K)
    
    img = vis.draw (img, P)    
    cv2.imshow ('Localised!', img)
    cv2.waitKey(0)
    
    cv2.destroyAllWindows()
        
if __name__ == '__main__':
    sys.exit (main())
