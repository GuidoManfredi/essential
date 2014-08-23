#include "FilesManager.h"

#include <opencv2/highgui/highgui.hpp>

#include "alphanumComp.hpp"

using namespace std;
using namespace boost::filesystem;
using namespace cv;

FilesManager::FilesManager() {}

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
                string mask_path = v[i].parent_path().string() + "/" + base_name + "_maskcrop.png";

                View view;
                view.angle_ = readAngle (pose_path);
                //cout << view.angle_ << endl;
                Mat image = imread(image_path, CV_LOAD_IMAGE_GRAYSCALE);
                //imshow("Debug", image); waitKey(0);
                Mat mask = imread(mask_path, CV_LOAD_IMAGE_GRAYSCALE);
                pipe2d_.extractDescriptors (image, mask, view.descriptors_);

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

/*
int getFileType(string filename) {


    if (fields.size() == 5)
        return 0;
    else
    if (strcmp(ext, ".png"))
}
*/
