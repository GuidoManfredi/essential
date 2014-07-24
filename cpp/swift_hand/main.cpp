#include <iostream>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include "SwiftHand.h"
#include "Database.h"
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

void createVocabAndSaveData () {
    // Create data
    cv::Mat train_matrix, test_matrix;
    std::vector<float> train_labels, test_labels;
    cout << "Loading train data" << endl;
//    db.createTrainDataVocab ("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Train/marcel_train_list.txt", "vocab.yaml",
//                            train_matrix, train_labels);
    db.createTrainData ("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Train/marcel_train_list.txt",
                        train_matrix, train_labels);
    cout << "Loading test data" << endl;
    db.createTestData ("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Test/marcel_test_list.txt",
                       test_matrix, test_labels);
    db.saveMat("train_matrix.yaml", train_matrix);
    Mat train_labels_mat(train_labels.size(), 1, CV_32F, &train_labels[0]); // convert vector to matrix
    db.saveMat("train_labels.yaml", train_labels_mat);
    db.saveMat("test_matrix.yaml", test_matrix);
    Mat test_labels_mat(test_labels.size(), 1, CV_32F, &test_labels[0]); // convert vector to matrix
    db.saveMat("test_labels.yaml", test_labels_mat);
}

void LoadDataAndTuneParameters () {
    Mat train_matrix, train_labels, test_matrix, test_labels;
    train_matrix = db.loadMat("train_matrix250.yaml");
    train_labels = db.loadMat("train_labels.yaml");
    test_matrix = db.loadMat("test_matrix250.yaml");
    test_labels = db.loadMat("test_labels.yaml");

    // Tune parameters
    float min_c = 1e-5, step_c = 10, max_c = 1e5;
    float min_gamma = 1e-5, step_gamma = 10, max_gamma = 1e5;
    sh.tuneParameters(train_matrix, train_labels, test_matrix, test_labels,
                      min_c, step_c, max_c,
                      min_gamma, step_gamma, max_gamma);
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
    train_matrix = db.loadMat("train_matrix100.yaml");
    train_labels = db.loadMat("train_labels.yaml");
    test_matrix = db.loadMat("test_matrix100.yaml");
    test_labels = db.loadMat("test_labels.yaml");
    mat2libsvm(train_matrix, train_labels, "train");
    mat2libsvm(test_matrix, test_labels, "test");
}

int main() {
    //createVocabAndSaveData ();
    //LoadDataAndTuneParameters ();
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

