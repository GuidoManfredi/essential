#include "FilesManager.h"

#include <opencv2/highgui/highgui.hpp>

#include "alphanumComp.hpp"

using namespace std;
using namespace boost::filesystem;
using namespace cv;

FilesManager::FilesManager(Mat K, Pipeline2D* pipe): pipe2d_(pipe), K_(K) {
}

void FilesManager::setFeatures(Feature ft) {
    pipe2d_->setFeatures(ft);
    features_ = ft;
}

int FilesManager::getFeatures() {
    return features_;
}

Object FilesManager::createObject (string folder_path) {
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
                string loc_path = v[i].parent_path().string() + "/" + base_name + "_loc.txt";
                string mask_path = v[i].parent_path().string() + "/" + base_name + "_maskcrop.png";
                vector<string> base_split;
                boost::split(base_split, base_name, boost::is_any_of("_"));

                View view;
                view.tilt_ = atoi(base_split[3].c_str()); // 1, 2 or 4
                view.angle_ = readAngle (pose_path);
                Point2f origin = readOrigin (loc_path);
                //cout << view.angle_ << endl;
                //cout << origin << endl;
                Mat image = imread(image_path, CV_LOAD_IMAGE_GRAYSCALE);
                view.image_ = image;
                Mat depth = imread(depth_path, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
                depth.convertTo(depth, CV_32F); // convert to float values
                Mat mask = imread(mask_path, CV_LOAD_IMAGE_GRAYSCALE);

                pipe2d_->extractDescriptors (image, mask, view.keypoints_, view.descriptors_);

                view.points_ = depth2points(depth, origin);
                //cout << image_path << endl;
                //imshow("Debug", image); waitKey(0);

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
    object.number_views_ = object.views_.size();
    object.features_type_ = pipe2d_->getFeatures();
    return object;
}

int FilesManager::getNumFolders(string p) {
    boost::filesystem::path bf_folder_path(p);
    if (is_directory(bf_folder_path)) {
        typedef vector<path> vec;
        vec v;
        copy(directory_iterator(bf_folder_path), directory_iterator(), back_inserter(v));

        int num_folders = 0;
        for (size_t i = 0; i < v.size(); ++i)
            if (is_directory(v[i])) ++num_folders;
        return num_folders;
    }
    return 0;
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

Point2f FilesManager::readOrigin (std::string path) {
    Point2f origin;
    string line;
    vector <string> fields;
    ifstream stream (path.c_str());
    if (stream.is_open()) {
        while ( getline (stream, line) ) {
            split(fields, line, boost::is_any_of(","));
            origin.x = atof(fields[0].c_str());
            origin.y = atof(fields[1].c_str());
        }
        stream.close();
        return origin;
    } else {
        cout << "Unable to open file";
    }

    return Point2f(0.0, 0.0);
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

vector<Point3f> FilesManager::depth2points (Mat depth, Point2f origin) {
    float mm_per_m = 1000; // milimeter per meter
    float f = K_.at<double>(0, 0);
    float u0 = K_.at<double>(0, 2);
    float v0 = K_.at<double>(1, 2);

    vector<Point3f> points;
    for (int x = 0; x < depth.cols; ++x) {
        for (int y = 0; y < depth.rows; ++y) {
            Point3f pt;
            //cout << depth.at<float>(y, x) << " ";
            pt.x = (x + (origin.x - u0)) * depth.at<float>(y, x) / f / mm_per_m;
            pt.y = (y + (origin.y - v0)) * depth.at<float>(y, x) / f / mm_per_m;
            pt.z = depth.at<float>(y, x) / mm_per_m;
            points.push_back(pt);
            //cout << pt << endl;
        }
        //cout << endl;
    }
    //cout << points[0] << endl;
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
