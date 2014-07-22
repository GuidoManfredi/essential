#include "Pipeline.h"

using namespace cv;
using namespace std;

Posture Pipeline::createPosture(std::string path) {
    Mat img = imread(path, CV_LOAD_IMAGE_GRAYSCALE);
    if ( !img.data ) {
        cout << "Couldn't load " << path << endl;
    } else {
        img = Mat::zeros(640, 480, CV_32F); // blank image
    }

    return createPosture(img);
}

Posture Pipeline::createPosture(cv::Mat img) {
    Posture post;
    pipe2d.getFeatures(img, post.kpts_, post.descs_);
    return post;
}

int matchPostures(Posture p1, vector<Posture> ps) {
    int tmp_matches = 0;
    int min_matches = 100;
    int min_idx = 0;
    for (size_t i = 0; i < ps.size(); ++i) {
        tmp_matches = matchPostures(p1,ps[i]);
        if (tmp_matches < min_matches) {
            min_matches = tmp_matches;
            min_idx = i;
        }
    }
    return min_idx;
}

int Pipeline::matchPostures(Posture p1, Posture p2) {
    std::vector<cv::DMatch> matches;
    pipe2d.match(p1.descs_, p2.descs_, matches);
    return matches.size();
}
