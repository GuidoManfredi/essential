#include <iostream>
#include "HOM.h"
#include "HOD.h"

using namespace std;
using namespace cv;

// Warning : If you change the object filename, you must also change the first name
//          in the object file (object name). Otherwise loading won't know which object
//          to load.
// TODO Test a lot
//      Add the Object class and loop closure for multiple faces detection.
//      Create an object for which multiple faces can be seen at the same time.
void testSaveLoad () {
    HOM hom;
    Face F = hom.createFace("/home/gmanfred/Pictures/box.png", 1);
    hom.saveFace(F, "box.yaml");
    F = hom.createFace("/home/gmanfred/Pictures/faugeras.jpg", 1);
    hom.saveFace(F, "faugeras.yaml");
    F = hom.createFace("/home/gmanfred/Pictures/rice.jpg", 1);
    hom.saveFace(F, "rice.yaml");

    HOD hod;
    hod.loadFace("box.yaml");
    hod.loadFace("faugeras.yaml");
    hod.loadFace("rice.yaml");
    F = hod.faces_[0];
    assert(F.size_.width == 324);
    assert(F.size_.height == 223);
    assert(F.keypoints_.size() == 672);
}

void testStatic() {
    HOM hom;
    Face F = hom.createFace("/home/gmanfred/Pictures/box.png", 1);
    hom.saveFace(F, "box.yaml");
    F = hom.createFace("/home/gmanfred/Pictures/faugeras.jpg", 1);
    hom.saveFace(F, "faugeras.yaml");
    F = hom.createFace("/home/gmanfred/Pictures/rice.jpg", 1);
    hom.saveFace(F, "rice.yaml");

    HOD hod;
    hod.loadFace("box.yaml");
    hod.loadFace("faugeras.yaml");
    hod.loadFace("rice.yaml");
    Mat image = imread("/home/gmanfred/Pictures/box_in_scene.png", CV_LOAD_IMAGE_COLOR);
    std::vector<cv::Mat> Ps;
    hod.find(image, Ps);
    if (Ps.size() > 0)
        cout << "Found " << Ps.size() << " faces in the scene." << endl;
    else
        cout << "Found no known face" << endl;
}

void testDynamic() {
    HOM hom;
    Face F = hom.createFace("/home/gmanfred/devel/datasets/my_objects/purfruit/big1.png", 1);
    hom.saveFace(F, "purfruit0.yaml");
    F = hom.createFace("/home/gmanfred/devel/datasets/my_objects/purfruit/big2.png", 2);
    hom.saveFace(F, "purfruit1.yaml");
    F = hom.createFace("/home/gmanfred/devel/datasets/my_objects/purfruit/big3.png", 3);
    hom.saveFace(F, "purfruit3.yaml");
    F = hom.createFace("/home/gmanfred/devel/datasets/my_objects/reveil_fruite/small1.png", 1);
    hom.saveFace(F, "reveil_fruite0.yaml");
    F = hom.createFace("/home/gmanfred/devel/datasets/my_objects/reveil_fruite/small2.png", 1);
    hom.saveFace(F, "reveil_fruite1.yaml");
    F = hom.createFace("/home/gmanfred/devel/datasets/my_objects/reveil_fruite/small3.png", 1);
    hom.saveFace(F, "reveil_fruite2.yaml");

    HOD hod;
    hod.loadFace("purfruit0.yaml");
    hod.loadFace("purfruit1.yaml");
    hod.loadFace("purfruit3.yaml");
    hod.loadFace("reveil_fruite1.yaml");
    hod.loadFace("reveil_fruite2.yaml");
    hod.loadFace("reveil_fruite2.yaml");
    VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return;

    std::vector<cv::Mat> Ps;
    for(;;) {
        Ps.empty();
        Mat frame;
        cap >> frame;
        hod.find(frame, Ps);
    }
}

int main() {
    //testSaveLoad();
    //testStatic();
    testDynamic();
    return 0;
}
