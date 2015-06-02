#include "POD.h"
#include <fstream>
#include <boost/algorithm/string.hpp>

using namespace std;
using namespace cv;

POD::POD () {
    bool is_intrinsic_set_ = false;
}

void POD::setObject (Object object) {
    if (!object.views_.size())
        cout << "Could not load object" << endl;
    objects_.push_back(object);
}

void POD::loadObject (std::string object_path) {    
    vector<string> strs;
    boost::split(strs,object_path,boost::is_any_of("/"));
    string object_file = strs[strs.size() - 1];
    boost::split(strs,object_file,boost::is_any_of("."));
    string object_name = strs[0];
    cout << "Loading " << object_name << " from " << object_path << "." << endl;
    
    FileStorage fs(object_path, FileStorage::READ);
    Object object;
    fs[object_name] >> object;
    setObject(object);
}

int POD::loadObjectsFromList (string list_path) {
    int num_objects = 0;
    string line;
    ifstream file(list_path.c_str());
    if (file.is_open()) {
        while( getline(file, line)) {
            loadObject(line);
            ++num_objects;
        }
        file.close();
    } else {
        cout << "Could not open " << list_path << "." << endl;
    }
    return num_objects;
}

void POD::setIntrinsic (cv::Mat K) {
    K.copyTo(K_);
    is_intrinsic_set_ = true;
}

void POD::loadIntrinsic (string calibration_file) {
    Mat K(3, 3, CV_32F);
    FileStorage r_fs;
    r_fs.open (calibration_file, cv::FileStorage::READ);
    r_fs["camera_matrix"]>>K;
    r_fs.release ();
    setIntrinsic (K);
}

bool POD::isIntrinsicSet() {
    return is_intrinsic_set_;
}

vector<int> POD::process (const Mat image, vector<Mat> &poses, vector<string> &names) {
    poses.clear(); names.clear();
    View view = createView (image);
    return process (view, poses, names);
}

vector<int> POD::process (View current, vector<Mat> &poses, vector<string> &names) {
    poses.clear(); names.clear();
    vector<int> object_numbers;
    vector<DMatch> matches;
    for (size_t i = 0; i < objects_.size(); ++i) {
        matches.clear();
        if (current.descriptors_.rows) {
            match (current, objects_[i], matches);
            //cout << "Matches: " << matches.size() << endl;
            if (matches.size() > 10) {
                Mat pose = computePose (current, objects_[i], matches);
                poses.push_back(pose);
                names.push_back(objects_[i].name());
                object_numbers.push_back(i);
            }
        }
    }
    return object_numbers;
}
////////////////////////////////////////////////////////////////////////////////
//  PRIVATE METHODS
////////////////////////////////////////////////////////////////////////////////
View POD::createView (Mat image) {
    View view;
    pipeline2d_.getGray (image, view.view_);
    //pipeline2d_.extractFeatures (view.view_, false, view.keypoints_, view.descriptors_);
    pipeline2d_.extractFeatures (view.view_, USE_SIFTGPU, view.keypoints_, view.descriptors_);
    //cout << "Extracted " << view.keypoints_.size() << " keypoints." << endl;
    return view;
}

void POD::match (View current, Object object, vector<DMatch> &matches) {
    Mat descriptors;
    object.getDescriptors(descriptors);
    //cout << "Descs: " << descriptors.rows << " vs " << current.descriptors_.rows << endl;
    //pipeline2d_.match (current.descriptors_, descriptors, false, matches);
    pipeline2d_.match (current.descriptors_, descriptors, USE_SIFTGPU, matches);
}

Mat POD::computePose (View current, Object object, vector<DMatch> matches) {
    if ( matches.size() <= 4) {
        cout << "POD.cpp: computePose : not enought matches (" << matches.size() << ")." << endl;
        Mat R = Mat::eye(3,3,CV_32F);
        Mat t = Mat::zeros(1,3,CV_32F);
        return Rt2P(R,t);
    }

    vector<Point3f> points;
    object.getPoints(points);
    vector<Point2f> points2d;
    vector<Point3f> points3d;
    match2points (current.keypoints_, points, matches,
                  points2d, points3d);

    Mat p3d_inliers, p2d_inliers;
    //cout << points3d.size() << endl;
    Mat R, t;
    pipelineGeom_.mySolvePnP (Mat(points3d), Mat(points2d), K_, R, t,
                              p3d_inliers, p2d_inliers);

    return Rt2P(R, t);
}

void POD::match2points (vector<KeyPoint> keypoints, vector<Point3f> points, vector<DMatch> matches,
                         vector<Point2f> &points2d, vector<Point3f> &points3d) {
    points2d.clear();
    points3d.clear();
    for (size_t i = 0; i < matches.size(); ++i) {
        points2d.push_back (keypoints[matches[i].queryIdx].pt);
        points3d.push_back (points[matches[i].trainIdx]);
    }
}
