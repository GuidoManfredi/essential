#include <iostream>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include "SwiftHand.h"
#include "Database.h"
#include "HandSeg.h"
#include "generateList.h"

// TODO
// Faire fonctions pour avoir les trues positives et false positive à partir de la
//  confusion matrix (ROC curve).
// Regarder pourquoi jamais de resultats pour les class 2, 4 et 6.
// Sauvegarder des données pour des tailles de vocab differentes.

using namespace std;
using namespace cv;

Pipeline2D pipeline;
CvSVM mySVM;

SwiftHand sh;
Database db;

void generateMarcelUniformTrainList(string list_path) {
    bool append = false;
    generateListFromFolder("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Test/A/uniform/", list_path, ".ppm", append, 1);
    append = true;
    generateListFromFolder("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Test/B/uniform/", list_path, ".ppm", append, 2);
    generateListFromFolder("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Test/C/uniform/", list_path, ".ppm", append, 3);
    generateListFromFolder("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Test/Five/uniform/", list_path, ".ppm", append, 4);
    generateListFromFolder("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Test/Point/uniform/", list_path, ".ppm", append, 5);
    generateListFromFolder("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Test/V/uniform/", list_path, ".ppm", append, 6);
}

void generateMarcelComplexTestList6(string list_path) {
    bool append = false;
    generateListFromFolder("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Test/A/complex/", list_path, ".ppm", append, 1);
    append = true;
    generateListFromFolder("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Test/B/complex/", list_path, ".ppm", append, 2);
    generateListFromFolder("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Test/C/complex/", list_path, ".ppm", append, 3);
    generateListFromFolder("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Test/Five/complex/", list_path, ".ppm", append, 4);
    generateListFromFolder("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Test/Point/complex/", list_path, ".ppm", append, 5);
    generateListFromFolder("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Test/V/complex/", list_path, ".ppm", append, 6);
}

void generateMoeslundUniformTrainList(string list_path) {
    bool append = false;
    generateListFromFolder("/home/gmanfred/devel/essential/cpp/swift_hand/data/moeslund/A/", list_path, ".tif", append, 1);
    append = true;
    generateListFromFolder("/home/gmanfred/devel/essential/cpp/swift_hand/data/moeslund/B/", list_path, ".tif", append, 2);
}

void generateMarcelComplexTestList2(string list_path) {
    bool append = false;
    generateListFromFolder("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Test/A/complex/", list_path, ".ppm", append, 1);
    append = true;
    generateListFromFolder("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Test/B/complex/", list_path, ".ppm", append, 2);
}

void createVocabAndSaveData (string train_list_path, string test_list_path) {
    cv::Mat train_matrix, test_matrix;
    std::vector<float> train_labels, test_labels;
    bool skin_seg = false;
    cout << "Loading train data" << endl;
    db.createTrainData (train_list_path, train_matrix, train_labels, skin_seg);

    cout << "Loading test data" << endl;
    skin_seg = true;
    db.createTestData (test_list_path, test_matrix, test_labels, skin_seg);

    cout << "Saving train data" << endl;
    db.saveMat("train_matrix.yaml", train_matrix);
    Mat train_labels_mat(train_labels.size(), 1, CV_32F, &train_labels[0]); // convert vector to matrix
    db.saveMat("train_labels.yaml", train_labels_mat);

    cout << "Saving test data" << endl;
    db.saveMat("test_matrix.yaml", test_matrix);
    Mat test_labels_mat(test_labels.size(), 1, CV_32F, &test_labels[0]); // convert vector to matrix
    db.saveMat("test_labels.yaml", test_labels_mat);
}

void mat2libsvm(Mat data, Mat labels, string path) {
    ofstream s;
    s.open (path.c_str());
    for (size_t i = 0; i < data.rows; ++i) {
        s << labels.at<float>(i) << " ";
        for (size_t j = 0; j < data.row(i).cols; ++j) {
            s << j + 1 << ":" << data.row(i).at<float>(j) << " ";
        }
        s << endl;
    }
    s.close();
}

void mat2libsvm() {
    Mat train_matrix, train_labels, test_matrix, test_labels;
    train_matrix = db.loadMat("train_matrix.yaml");
    train_labels = db.loadMat("train_labels.yaml");
    test_matrix = db.loadMat("test_matrix.yaml");
    test_labels = db.loadMat("test_labels.yaml");
    mat2libsvm(train_matrix, train_labels, "train");
    mat2libsvm(test_matrix, test_labels, "test");
}

void testSkinSeg() {
    Mat image = imread("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Test/A/complex/A-complex02.ppm", CV_LOAD_IMAGE_COLOR);
    HandSeg hs;
    Mat hand = hs.getHand(image);
    imshow("kikou", hand); waitKey(0);
}

int main() {
    string train_list_path("data/train_list.txt");
    string test_list_path("data/test_list.txt");
    generateMarcelUniformTrainList(train_list_path);
    generateMarcelComplexTestList6(test_list_path);
    //generateMoeslundUniformTrainList(train_list_path);
    //generateMarcelComplexTestList2(test_list_path);
    createVocabAndSaveData (train_list_path, test_list_path);
    mat2libsvm();
    return 0;
}
/*
VideoCapture cap(0);
if(!cap.isOpened()) {
    cout << "Couldn't open webcam 0" << endl;
    return -1;
}

namedWindow("edges",1);
for(;;) {
    Mat frame;
    cap >> frame; // get a new frame from camera
    cvtColor(frame, edges, CV_BGR2GRAY);
    imshow("edges", edges);
    if(waitKey(30) >= 0) break;
}
*/

