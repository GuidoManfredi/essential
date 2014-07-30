#include "HOM.h"

using namespace std;
using namespace cv;

Face HOM::createFace(std::string image_path, int id) {
    cout << "Create face from image " << image_path << endl;
    Mat image = imread(image_path, CV_LOAD_IMAGE_COLOR);
    if (image.empty())
        cout << "Could not find image " << image_path << endl;
    //imshow("Debug", image); waitKey(0);
    return createFace(image, id);
}

Face HOM::createFace(cv::Mat image, int id) {
    std::vector<cv::KeyPoint> kpts;
    cv::Mat descs;
    pip_.extractFeatures(image, kpts, descs);
    cout << "Extracted " << kpts.size() << " keypoints." << endl;
    return Face(image, id, kpts, descs);
}

void HOM::saveFace (Face face, string path) {
    cout << "Saving Face" << endl;
    FilesManager fm;
    string object_name = fm.getName (path);
    string object_filename = object_name + ".yaml";
    cout << "Saving to " << object_filename << endl;
    FileStorage fs(object_filename, FileStorage::WRITE);
    fs << object_name << face;
    fs.release();
}
