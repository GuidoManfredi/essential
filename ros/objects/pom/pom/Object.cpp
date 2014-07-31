#include <iostream>

#include "Object.h"

using namespace std;
using namespace cv;

Object::Object () {
    number_views_ = 0;
}

void Object::addView (std::vector<Point3f> points3d, cv::Mat descriptors) {
    View view;
    view.points3d_.swap (points3d);
    descriptors.copyTo(view.descriptors_);
    ++number_views_;
    views_.push_back (view);
}

void Object::getDescriptors (Mat &descriptors) {
    descriptors = Mat::zeros (0, 0, CV_32F);
    for (size_t i = 0; i < views_.size(); ++i)
        descriptors.push_back (views_[i].descriptors_);
}

void Object::getPoints (vector<Point3f> &points3d) {
    points3d.clear();
    for (size_t i = 0; i < views_.size(); ++i) {
        for (size_t j = 0; j < views_[i].points3d_.size(); ++j) {
            points3d.push_back (views_[i].points3d_[j]);
        }
    }
}

string Object::name() {
    return name_;
}

void Object::write(cv::FileStorage& fs) const {
    fs << "{" << "number_views" << number_views_;
    for ( int i = 0; i < number_views_; ++i ) {
        ostringstream view_stream;
        view_stream << "view_" << i;
        fs << view_stream.str() << views_[i];
    }
    fs << "}";
}

void Object::read(const FileNode& node) {
    if (node.isNamed())
        name_ = node.name();
    else
        cout << "Warning : no name for this object." << endl;
    node["number_views"] >> number_views_;
    views_.resize(number_views_);
    for ( int i = 0; i < number_views_; ++i ) {
        ostringstream view_stream;
        view_stream << "view_" << i;
        node[view_stream.str()] >> views_[i];
    }
}
