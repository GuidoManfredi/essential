#include "POD.h"

using namespace std;
using namespace cv;

POD::POD () {

}

bool POD::process (cv::Mat image) {
    Mat gray;
    pipeline2d_.getGray (image, current_image_);

    //pipeline2d_.detectFeatures (gray, current_keypoints_);
    //pipeline2d_.computeBoW (gray, current_keypoints_, current_descriptors_, current_bow_);
    pipeline2d_.extractFeatures (current_image_, current_keypoints_, current_descriptors_);
    //pipeline2d_.computeBoW (current_image_, current_keypoints_, current_bow_);
    pipeline2d_.computeBoW (current_descriptors_, current_bow_);
    if ( faces_.size() > 0 )
        findFace (faces_[0]);
}

bool POD::findFace (Face face) {
    closest_view_ = pipeline2d_.matchBoW (current_bow_, face.bows_);
    findView (face.keypoints_[closest_view_], face.descriptors_[closest_view_]);
}

bool POD::findView (vector<KeyPoint> train_keypoints, Mat train_descriptors) {
    pipeline2d_.match (current_descriptors_, train_descriptors, current_matches_);
    Mat H;
    pipelineGeom_.filterMatchesHomography (current_keypoints_, train_keypoints, current_matches_, H);
    showMatches(current_image_, current_keypoints_,
                faces_[0].views_[closest_view_], faces_[0].keypoints_[closest_view_],
                current_matches_);
}

void POD::addFace (Face face) {
    faces_.push_back (face);
    pipeline2d_.setVocabulary(face.vocabulary_);
}
////////////////////////////////////////////////////////////////////////////////
// PRIVATE METHODS
////////////////////////////////////////////////////////////////////////////////
void POD::showMatches (Mat image1, vector<KeyPoint> keypoints1,
                        Mat image2, vector<KeyPoint> keypoints2,
                        vector<DMatch> matches) {
    namedWindow("matches", 1);
    Mat img_matches;
    drawMatches(image1, keypoints1, image2, keypoints2, matches, img_matches);
    imshow("matches", img_matches);
    waitKey(1);
}
