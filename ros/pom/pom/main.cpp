#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

//#include <opencv2/highgui/highgui.hpp>

#include "POM.h"
//#include "POD.h"

using namespace std;
using namespace boost::filesystem;
using namespace cv;

Pipeline2D pipeline2d;
POM pom;

Mat readImage (string image_path) {
    Mat image;
    path bf_images_path(image_path);
    if (extension(bf_images_path) == ".png"
        || extension(bf_images_path) == ".jpg") {
        image = imread (bf_images_path.string(), CV_LOAD_IMAGE_COLOR);
        if(! image.data )
            cout << "Could not open or find the image at " << bf_images_path.string() << endl;
    } else {
        cout << "Did not read " << bf_images_path.string() << endl;
    }
    return image;
}

vector<Mat> readImages (string images_path) {
    vector<Mat> images;
    path bf_images_path(images_path);
    if ( is_directory(bf_images_path) ) {
        typedef vector<path> vec;
        vec v;
        copy(directory_iterator(bf_images_path), directory_iterator(), back_inserter(v));
        sort (v.begin(), v.end());
        for (vec::const_iterator it (v.begin()); it != v.end(); ++it) {
            Mat image = readImage (it->string());
            if ( image.data )
                images.push_back (image);
        }
    } else {
        cout << "Not a directory. Failing, boum!" << endl;
    }
    return images;
}

vector<vector<Point2f> > readCorners (string corners_file) {
    ifstream file (corners_file.c_str());
    string line;
    vector<vector<Point2f> > all_corners;
    if (file.is_open()) {
        while ( getline (file,line) ) {
            vector<string> parts;
            boost::split(parts, line, boost::is_any_of(" "));
            vector<Point2f> corners;
            //cout << "Partsize " << parts.size() << endl;
            // TODO change ca
            //for ( size_t i = 0; i < parts.size(); i += 2 ) {
            for ( size_t i = 0; i < 8; i += 2 ) {
                int x = atoi (parts[i].c_str());
                int y = atoi (parts[i+1].c_str());
                corners.push_back (Point2f(x, y));
            }
            all_corners.push_back (corners);
        }
        file.close();
    }
    return all_corners;
}

Point3f readDimensions (string corners_file) {
    Point3f dimensions;

    ifstream file (corners_file.c_str());
    string line;
    vector<vector<Point2f> > all_corners;
    if (file.is_open()) {
        while ( getline (file,line) ) {
            vector<string> parts;
            boost::split(parts, line, boost::is_any_of(" "));
            for ( size_t i = 0; i < 3; ++i ) {
                int x = atoi (parts[0].c_str());
                int y = atoi (parts[1].c_str());
                int z = atoi (parts[2].c_str());
                dimensions = Point3f (x, y, z);
            }
        }
        file.close();
    }
    return dimensions;
}

Mat readIntrinsic (string calibration_file) {
    Mat K(3,3,CV_32F);
    FileStorage r_fs;
    r_fs.open (calibration_file, cv::FileStorage::READ);
    r_fs["camera_matrix"]>>K;
    r_fs.release ();
	return K;
}

void testModeling () {
    string intrinsic = "/home/gmanfred/.ros/camera_info/webcam_gilgamesh_opencv.yml";
    Mat K = readIntrinsic (intrinsic);
    pom.setIntrinsic (K);

    string base = "/home/gmanfred/devel/datasets/my_objects/purfruit/";
    vector<Mat> images = readImages(base);
    vector< vector<Point2f> > corners = readCorners (base + "corners.txt");
    Point3f dims = readDimensions (base + "dimensions.txt");
    Object obj = pom.model (images, corners, dims);
}

int main()
{
    testModeling ();
    return 0;
}
