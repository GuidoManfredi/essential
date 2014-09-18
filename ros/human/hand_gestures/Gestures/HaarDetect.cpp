#include "HaarDetect.h"

#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

HaarDetect::HaarDetect() {}

Rect HaarDetect::detect(Mat image, Point2f clue, int padding, bool display) {
    Rect roi (MAX(0, clue.x - padding), MAX(0, clue.y - padding),
              MIN(image.cols - clue.x - 1, 2 * padding), MIN(image.rows - clue.y - 1, 2 * padding));
    //cout << roi << endl;
    Mat frame = image(roi);
    // Detect objects
    std::vector<Rect> objects;
    double scale_factor = 1.05;
    //int min_neighbors = 15;
    int min_neighbors = 10;
    face_cascade_.detectMultiScale( frame, objects, scale_factor, min_neighbors, 0, Size(20, 20) );

    if (display) {
        for( size_t i = 0; i < objects.size(); i++ ) {
            Point center( objects[i].x + objects[i].width*0.5, objects[i].y + objects[i].height*0.5 );
            ellipse( frame, center, Size( objects[i].width*0.5, objects[i].height*0.5), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );

            Mat faceROI = frame( objects[i] );
        }
        // Show what you got
        imshow("Debug", frame ); waitKey(0);
    }

    if (objects.size() > 0)
        return objects[0];

    Rect zero(0,0,0,0);
    return zero;
}

int HaarDetect::loadCascade(string cascade_path) {
    face_cascade_name_ = cascade_path;
    int loaded = face_cascade_.load(face_cascade_name_);
    if (!loaded)
        cout << "Could not load " << cascade_path << endl;
    return loaded;
}
////////////////////////////////////////////////////////////////////////////////
// PRIVATE METHODS
////////////////////////////////////////////////////////////////////////////////
