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

void POD::loadObjectsFromList (string list_path) {
    string line;
    ifstream file(list_path.c_str());
    if (file.is_open()) {
        while( getline(file, line)) {
            loadObject(line);
        }
        file.close();
    } else {
        cout << "Could not open " << list_path << "." << endl;
    }
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

void POD::process (const Mat image, vector<Mat> &poses, vector<string> &names) {
    poses.clear(); names.clear();
    View view = createView (image);
    process (view, poses, names);
}

void POD::process (View current, vector<Mat> &poses, vector<string> &names) {
    poses.clear(); names.clear();
    for (size_t i = 0; i < objects_.size(); ++i) {
        vector<DMatch> matches;
        //cout << "Matching" << endl;
        match (current, objects_[i], matches);
        cout << "Matches: " << matches.size() << endl;
        //cout << "Computing pose" << endl;
        Mat pose = computePose (current, objects_[i], matches);
        poses.push_back(pose);
        names.push_back(objects_[i].name());
    }
}
////////////////////////////////////////////////////////////////////////////////
//  PRIVATE METHODS
////////////////////////////////////////////////////////////////////////////////
View POD::createView (Mat image) {
    View view;
    pipeline2d_.getGray (image, view.view_);
    pipeline2d_.extractFeatures (view.view_, view.keypoints_, view.descriptors_);
    //cout << "Extracted " << view.keypoints_.size() << " keypoints." << endl;
    return view;
}

void POD::match (View current, Object object, vector<DMatch> &matches) {
    Mat descriptors;
    object.getDescriptors(descriptors);
    pipeline2d_.match (current.descriptors_, descriptors, matches);
    //cout << "Descs: " << descriptors.rows << " vs " << current.descriptors_.rows << endl;
}

Mat POD::computePose (View current, Object object, vector<DMatch> matches) {
    if ( matches.size() <= 4) {
        cout << "ComputeTrainPose : not enought matches (" << matches.size() << ")." << endl;
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
