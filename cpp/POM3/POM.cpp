#include "POM.h"

using namespace std;
using namespace cv;

POM::POM () {

}

Object POM::createObjectWithSynthetic (vector<Mat> images) {
    Object object;
    // Add generated views from each face
    for ( size_t i = 0; i < images.size(); ++i ) {
        std::vector<cv::Point2f> corners = view_generator_.getCorners (images[i]);
        vector<Mat> views, affines;
        view_generator_.generateViews (images[i], corners, views, affines);
        object.addViews(views);
    }
    // Create a big face (= object) from all these views
    Face face = createFace (object.views_);
    object.addFace (face);

    return object;
}

Face POM::createFaceWithSynthetic (Mat image) {
    std::vector<cv::Point2f> corners = view_generator_.getCorners (image);
    vector<Mat> views, affines;
    view_generator_.generateViews (image, corners, views, affines);
    Face face = createFace (views);
    return face;
}

Face POM::createFace(std::vector<cv::Mat> views) {
    Face face;
    for ( size_t i = 0; i < views.size(); ++i ) {
        vector<KeyPoint> tmp_keypoints;
        Mat tmp_descriptors;
        features_pipeline_.extractFeatures(views[i], tmp_keypoints, tmp_descriptors);
        face.keypoints_.push_back (tmp_keypoints);
        face.descriptors_.push_back (tmp_descriptors);
        face.views_.push_back (views[i]);
    }
    // Create a vocabulary from these views and affect it to the face
    features_pipeline_.createVocabulary (face.descriptors_, face.vocabulary_);
    features_pipeline_.setVocabulary (face.vocabulary_);

    for ( size_t i = 0; i < views.size(); ++i ) {
        cv::Mat bow;
        //features_pipeline_.computeBoW (face.views_[i], face.keypoints_[i], bow);
        features_pipeline_.computeBoW (face.descriptors_[i], bow);
        face.bows_.push_back (bow);
    }
    return face;
}
/*
bool saveFace (Face face, string filename) {
    FileStorage file (filename, FileStorage::WRITE);
}

Face loadFace (string filepath) {

}



    fs << "frameCount" << 5;
    time_t rawtime; time(&rawtime);
    fs << "calibrationDate" << asctime(localtime(&rawtime));
    Mat cameraMatrix = (Mat_<double>(3,3) << 1000, 0, 320, 0, 1000, 240, 0, 0, 1);
    Mat distCoeffs = (Mat_<double>(5,1) << 0.1, 0.01, -0.001, 0, 0);
    fs << "cameraMatrix" << cameraMatrix << "distCoeffs" << distCoeffs;
    fs << "features" << "[";
    for( int i = 0; i < 3; i++ )
    {
        int x = rand() % 640;
        int y = rand() % 480;
        uchar lbp = rand() % 256;

        fs << "{:" << "x" << x << "y" << y << "lbp" << "[:";
        for( int j = 0; j < 8; j++ )
            fs << ((lbp >> j) & 1);
        fs << "]" << "}";
    }
    fs << "]";
    fs.release();
    return 0;

*/
