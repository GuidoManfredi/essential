#include "Object.h"

using namespace std;
using namespace cv;

Object::Object () {}

void Object::addView (std::vector<Point3f> points3d, cv::Mat descriptors) {
    View view;
    view.points3d_.swap (points3d);
    descriptors.copyTo(view.descriptors_);
    views_.push_back (view);
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
    node["number_views"] >> number_views_;
    views_.resize(number_views_);
    for ( int i = 0; i < number_views_; ++i ) {
        ostringstream view_stream;
        view_stream << "view_" << i;
        node[view_stream.str()] >> views_[i];
    }
}
