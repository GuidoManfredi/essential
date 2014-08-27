#include "HaarDetect.h"

#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

HaarDetect::HaarDetect() {}

Rect HaarDetect::detect(Mat frame, bool display)
{
    Mat frame_gray;
    cvtColor( frame, frame_gray, CV_BGR2GRAY );
    equalizeHist( frame_gray, frame_gray );
    // Detect faces
    std::vector<Rect> faces;
    double scale_factor = 1.1;
    int min_neighbors = 8;
    face_cascade_.detectMultiScale( frame_gray, faces, scale_factor, min_neighbors, 0, Size(30, 30) );

    if (display) {
        for( size_t i = 0; i < faces.size(); i++ ) {
            Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
            ellipse( frame, center, Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );

            Mat faceROI = frame_gray( faces[i] );
        }
        // Show what you got
        imshow("Debug", frame ); waitKey(0);
    }

    if (faces.size() > 0)
        return faces[0];

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
