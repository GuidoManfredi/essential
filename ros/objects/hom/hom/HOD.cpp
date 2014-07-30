#include "HOD.h"

using namespace std;
using namespace cv;

HOD::HOD() {

}

int HOD::find(cv::Mat image, std::vector<cv::Mat> &P) {
    // We use the Face class because it has all we need
    Face frame = hom_.createFace(image, 0);
    findFaces(frame, faces_, P);
}

int HOD::loadFacesFromList(std::string list_path) {
    faces_.clear();
    vector<string> paths;
    files_.getPathsFromList(list_path, paths);
    for (size_t i = 0; i < paths.size(); ++i) {
        loadFace(paths[i]);
    }
}

int HOD::loadFace(std::string path) {
    FilesManager fm;
    string face_name = fm.getName(path);
    cout << "Loading face " << face_name << " from " << path << endl;
    FileStorage fs(path, FileStorage::READ);
    Face face;
    fs[face_name] >> face;
    faces_.push_back(face);
    return 0;
}

void HOD::setIntrinsics(cv::Mat K) {
    K_ = K;
}

void HOD::loadIntrinsics(std::string intrinsics_path) {
    Mat K(3, 3, CV_32F);
    FileStorage r_fs;
    r_fs.open (intrinsics_path, cv::FileStorage::READ);
    r_fs["camera_matrix"]>>K;
    r_fs.release ();
    setIntrinsics (K);
}
////////////////////////////////////////////////////////////////////////////////
// PRIVATE METHODS
////////////////////////////////////////////////////////////////////////////////
int HOD::findFaces(Face frame, std::vector<Face> faces, std::vector<cv::Mat> &Ps) {
    Ps.clear();
    bool found = 1;
    for (size_t i = 0; i < faces.size(); ++i) {
        //while(found == 1) {
            Mat P;
            vector<DMatch> inliers;
            found = findFace(frame, faces[i], inliers, P);
            if (found) {
                Ps.push_back(P);
                //frame.removeMatches(inliers);
            }
        //}
    }

}

int HOD::findFace(Face frame, Face face,
                    std::vector<cv::DMatch> &inliers, cv::Mat &P) {
    pip2d_.match(frame.descriptors_, face.descriptors_, inliers);
    cout << "Found " << inliers.size() << " matches." << endl;
    // matche against temporary keypoints = keypoints with inliers from other faces
    //  removed.
    bool found = geopip_.computePose(frame, face,
                                      P, inliers);
    cout << "Face found ? " << found << endl;
    return found;
}
