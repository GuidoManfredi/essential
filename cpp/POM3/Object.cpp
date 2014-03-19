#include "Object.h"

using namespace std;
using namespace cv;

Object::Object () {
    ;
}

void Object::addViews (std::vector<Mat> views) {
    for ( size_t i = 0; i < views.size(); ++i ) {
        views_.push_back (views[i]);
    }
}

void Object::addFace (Face face) {
    for ( size_t i = 0; i < face.views_.size(); ++i ) {
        views_.push_back (face.views_[i]);
        keypoints_.push_back (face.keypoints_[i]);
        descriptors_.push_back (face.descriptors_[i]);
        bows_.push_back (face.bows_[i]);
    }
    vocabulary_.push_back (face.vocabulary_);
}
