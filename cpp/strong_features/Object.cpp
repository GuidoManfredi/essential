#include <iostream>

#include "Object.h"

using namespace std;
using namespace cv;

Object::Object () {
    number_views_ = 0;
}

void Object::write(cv::FileStorage& fs) const {
    fs << "{" << "number_views" << number_views_
              << "features_type" << features_type_;
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
    node["features_type"] >> features_type_;
    views_.resize(number_views_);
    for ( int i = 0; i < number_views_; ++i ) {
        ostringstream view_stream;
        view_stream << "view_" << i;
        node[view_stream.str()] >> views_[i];
    }
}
