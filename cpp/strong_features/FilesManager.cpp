#include "FilesManager.h"

#include <opencv2/highgui/highgui.hpp>

#include "alphanumComp.hpp"

using namespace std;
using namespace boost::filesystem;
using namespace cv;

FilesManager::FilesManager(Mat K) {
    K.copyTo(K_);
}

Object FilesManager::loadObject (string folder_path) {
    Object object;
    boost::filesystem::path bf_folder_path(folder_path);
    if (is_directory(bf_folder_path)) {
        typedef vector<path> vec;
        vec v;
        copy(directory_iterator(bf_folder_path), directory_iterator(), back_inserter(v));
        sort(v.begin(), v.end(), doj::alphanum_less<path>());

        for (size_t i = 0; i < v.size(); ++i) {
            string base_name;
            if (isPoseFile(v[i].filename().string(), base_name)) {
                //cout << base_name << endl;
                string pose_path = v[i].string();
                string image_path = v[i].parent_path().string() + "/" + base_name + "_crop.png";
                string depth_path = v[i].parent_path().string() + "/" + base_name + "_depthcrop.png";
                string mask_path = v[i].parent_path().string() + "/" + base_name + "_maskcrop.png";

                View view;
                view.angle_ = readAngle (pose_path);
                //cout << view.angle_ << endl;
                Mat image = imread(image_path, CV_LOAD_IMAGE_GRAYSCALE);
                Mat depth = imread(mask_path, CV_LOAD_IMAGE_GRAYSCALE);
                //imshow("Debug", image); waitKey(0);
                Mat mask = imread(mask_path, CV_LOAD_IMAGE_GRAYSCALE);

                pipe2d_.extractDescriptors (image, mask, view.keypoints_, view.descriptors_);
                //pipe2d_.extractDescriptors (image, mask, view.keys_);
                key2kpts(view.keys_, view.keypoints_);
                key2desc(view.keys_, view.descriptors_);

                view.points_ = depth2points(depth);

                //view.width_ = image.cols;
                //view.height_ = image.rows;
                object.views_.push_back(view);
            }
        }
    } else {
        cout << "Error: loadObject: " << folder_path << " not a folder." << endl;
    }
/*
    for (size_t i = 0; i < ; ++i) {

    }
*/
    return object;
}

float FilesManager::readAngle (std::string pose_path) {
    float angle = 0.0;
    string line;
    ifstream stream (pose_path.c_str());
    if (stream.is_open()) {
        while ( getline (stream, line) ) {
            angle = atof(line.c_str());
        }
        stream.close();
        return angle;
    } else {
        cout << "Unable to open file";
    }

    return -1.0;
}

int FilesManager::isPoseFile (string filename, string &base_name) {
    vector <string> fields;
    split(fields, filename, boost::is_any_of("_"));
    if (fields.size() == 6) {
        base_name = fields[0] + "_" + fields[1] + "_" + fields[2] + "_" + fields[3] + "_" + fields[4];
        return !strcmp(fields[5].c_str(),"pose.txt");
    }

    return 0;
}

void FilesManager::key2desc (vector<vector<keypointslist > > key, Mat &desc) {
    for (size_t i = 0; i < key.size(); ++i) {
        for (size_t j = 0; j < key[i].size(); ++j) {
            for (size_t k = 0; k < key[i][j].size(); ++k) {
                Mat sift = Mat(1, 128, CV_32F, &key[i][j][k].vec);
                desc.push_back (sift);
            }
        }
    }
}

void FilesManager::key2kpts (vector<vector<keypointslist > > key, vector<KeyPoint> &kpts) {
    for (size_t i = 0; i < key.size(); ++i) {
        for (size_t j = 0; j < key[i].size(); ++j) {
            for (size_t k = 0; k < key[i][j].size(); ++k) {
                KeyPoint kpt;
                kpt.pt.x = key[i][j][k].x;
                kpt.pt.y = key[i][j][k].y;
                kpt.angle = key[i][j][k].angle;
                kpt.size = key[i][j][k].scale;
                kpts.push_back (kpt);
            }
        }
    }
}

vector<Point3f> FilesManager::depth2points (Mat depth) {
    float f = K_.at<float>(0, 0);
    float u0 = K_.at<float>(0, 2);
    float v0 = K_.at<float>(1, 2);

    vector<Point3f> points;
    for (int x = 0; x < depth.cols; ++x) {
        for (int y = 0; y < depth.rows; ++y) {
            Point3f pt;
            pt.x = x * depth.at<float>(y, x) * f;
            pt.y = y * depth.at<float>(y, x) * f;
            pt.z = depth.at<float>(y, x);
            points.push_back(pt);
        }
    }
    return points;
}

/*
int getFileType(string filename) {


    if (fields.size() == 5)
        return 0;
    else
    if (strcmp(ext, ".png"))
}
*/
