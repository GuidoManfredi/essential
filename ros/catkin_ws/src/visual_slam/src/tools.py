#!/usr/bin/env python

import tf

import numpy as np
import cv2

def aligned_2d2d_points(kpts1, kpts2, matches):
    p2ds1 = []
    p2ds2 = []
    for m in matches:
        p2ds1.append(kpts1[m.queryIdx].pt)
        p2ds2.append(kpts2[m.trainIdx].pt)
    p2ds1 = np.float32(p2ds1)
    p2ds2 = np.float32(p2ds2)
    return p2ds1, p2ds2

def aligned_2d3d_points(kpts, in_p3ds, matches):
    out_p2ds = []
    out_p3ds = []
    for m in matches:
        out_p2ds.append(kpts[m.queryIdx].pt)
        out_p3ds.append(in_p3ds[m.trainIdx])
    out_p2ds = np.float32(out_p2ds)
    out_p3ds = np.float32(out_p3ds)
    return out_p2ds, out_p3ds
    
def p2d_to_kpts(p2ds):
    kpts = [cv2.KeyPoint(p2d[0],p2d[1],1) for p2d in p2ds]
    return kpts
    
def Rt2P(R,t):
    P = np.eye(4)
    P[0:3,0:3] = R
    P[0:3,3] = np.transpose(t)
    return P
    
def to_homogeneous(pts):
    return np.squeeze(cv2.convertPointsToHomogeneous(np.float32(pts)))
    
def from_homogeneous(pts):
    return np.squeeze(cv2.convertPointsFromHomogeneous(np.float32(pts)))
    
def transform(p3ds, T):
    hp3ds = to_homogeneous(p3ds)
    #print T.shape
    #print np.transpose(hp3ds).shape
    #print hp3ds.shape
    hp3ds = T.dot(np.transpose(hp3ds))
    hp3ds = np.transpose(hp3ds)
    return from_homogeneous(hp3ds)
    
def createVtkData(p3ds):
    points = vtk.vtkPoints()
    vertices = vtk.vtkCellArray()
    for p3d in p3ds:
        i = points.InsertNextPoint(p3d)
        vertices.InsertNextCell(1)
        vertices.InsertCellPoint(i)
        
    data = vtk.vtkPolyData()
    data.SetPoints(points)
    data.SetVerts(vertices)
    if vtk.VTK_MAJOR_VERSION <= 5:
        data.Update()
    return data
    return points, vertices
    
def pose2array(msg):
    T = np.empty((3,4), np.float32)
    q = msg.pose.orientation
    T = tf.transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
    T[0,3] = msg.pose.position.x
    T[1,3] = msg.pose.position.y
    T[2,3] = msg.pose.position.z
    return T



