#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "../pompod/POM2D3D.h"

using namespace std;
using namespace cv;
using namespace boost::filesystem;

POM2D3D modeler;

Mat readImage (string image_path);
vector<Mat> readImages (string images_path);
vector<vector<Point2f> > readCorners (string corners_file);
Point3f readDimensions (string corners_file);
cv::Mat readIntrinsic (std::string intrinsic);


// ./bin/modelingPlanar /home/gmanfred/devel/datasets/my_objects/purfruit
int main (int argc, char** argv) {
    Mat CI = readIntrinsic ("/home/gmanfred/.ros/camera_info/my_xtion.yml");
    modeler.setIntrinsic (CI);

    string image_path (argv[1]);
    string corners_path = image_path + "corners.txt";
    string dims_path = image_path + "dimensions.txt";
    vector<Mat> images = readImages (image_path);
    vector<vector<Point2f> > corners = readCorners (corners_path);
    Point3f dimensions = readDimensions (dims_path);
    //cout << dimensions << endl;
    /*
    for ( size_t i = 0; i < 3; ++i ) {
        for ( size_t j = 0; j < 4; ++j ) {
            cout << corners[i][j] << " ";
        }
        cout << endl;
    }
    */
    Object2D3D object = modeler.model (images, corners, dimensions);
    
    return 0;
}

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
        directory_iterator end_itr;
        for ( directory_iterator itr(bf_images_path); itr != end_itr; ++itr ) {
            Mat image = readImage (itr->path().string());
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
