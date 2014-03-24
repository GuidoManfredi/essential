#include "SaveCorners.h"

using namespace cv;
using namespace std;
using namespace boost::filesystem;

SaveCorners::SaveCorners (string object_dir) {
    boost::filesystem::path bf_images_path(object_dir);
    copy(directory_iterator(bf_images_path), directory_iterator(), back_inserter(v_));
    sort(v_.begin(), v_.end(), myCompare);

    file_.open ((object_dir + "/corners.txt").c_str()); // we want it to be at the bottom of the dir
}

SaveCorners::~SaveCorners () {
    file_.close();
}

void SaveCorners::saveCornersObject () {
    for ( size_t i = 0; i < v_.size(); ++i) {
        if ( extension(v_[i]) == ".png" || extension(v_[i]) == ".jpg") {
            cout << "Processing " << v_[i].string() << endl;
            Mat image = imread (v_[i].string(), CV_LOAD_IMAGE_GRAYSCALE);
            getCornersOnePicture (image);
            saveCorners (corners);
        } else {
            cout << "Not processing " << v_[i].string() << endl;
        }
    }
}
////////////////////////////////////////////////////////////////////////////////
// PRIVATE METHODS
////////////////////////////////////////////////////////////////////////////////
void SaveCorners::getCornersOnePicture (Mat image) {
    namedWindow ("Select object corners");
    setMouseCallback ("Select object corners", mouse_cb);
    imshow ("Select object corners", image);
    cout << "Select object corners in that order: top left, top right, bottom right, bottom left. Then press any key to continue." << endl;
    waitKey(0);
}

void SaveCorners::saveCorners (vector<Point2f> corners) {
    for ( size_t i = 0; i < corners.size(); ++i ) {
        if ( i != (corners.size() - 1) )
            file_ << corners[i].x << " " << corners[i].y << " ";
        else
            file_ << corners[i].x << " " << corners[i].y;
    }
    file_ << endl;
}

void SaveCorners::removeCornersFile (vector<path> v) {
    for ( vector<path>::iterator it = v.begin(); it < v.end(); it++ ) {
        string current_file = (*it).stem().string();
        if ( current_file == string("corners") ) {
            cout << "Erasing " << (*it).string() << endl;
            v.erase (it);
        }
    }
}
