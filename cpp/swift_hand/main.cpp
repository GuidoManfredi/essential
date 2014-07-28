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

string base_path ("/home/gmanfred/devel/essential/cpp/swift_hand/data/");

void generateMoeslundTrainList(string list_path) {
    bool append = false;
    generateListFromFolder(base_path + "moeslund/A/", list_path, ".tif", append, 1);
    append = true;
    generateListFromFolder(base_path + "moeslund/B/", list_path, ".tif", append, 2);
}

void generateMarcelUniformTrainList(string list_path) {
    bool append = false;
    generateListFromFolder(base_path + "Marcel-Test/MiniTrieschGallery/A/uniform/", list_path, ".pgm", append, 1);
    append = true;
    generateListFromFolder(base_path + "Marcel-Test/MiniTrieschGallery/B/uniform/", list_path, ".pgm", append, 2);
    //generateListFromFolder(base_path + "Marcel-Test/MiniTrieschGallery/C/uniform/", list_path, ".pgm", append, 3);
    //generateListFromFolder(base_path + "Marcel-Test/MiniTrieschGallery/Five/uniform/", list_path, ".pgm", append, 3);
    //generateListFromFolder(base_path + "Marcel-Test/MiniTrieschGallery/Point/uniform/", list_path, ".pgm", append, 3);
    //generateListFromFolder(base_path + "Marcel-Test/MiniTrieschGallery/V/uniform/", list_path, ".pgm", append, 6);
    generateListFromFolder(base_path + "Marcel-Train/mini/A/", list_path, ".ppm", append, 1);
    generateListFromFolder(base_path + "Marcel-Train/mini/B/", list_path, ".ppm", append, 2);
    generateListFromFolder(base_path + "Marcel-Train/mini/Five/", list_path, ".ppm", append, 3);
}

void generateMarcelComplexTestList(string list_path) {
    bool append = false;
    generateListFromFolder(base_path + "Marcel-Test/A/complex/", list_path, ".ppm", append, 1);
    append = true;
    generateListFromFolder(base_path + "Marcel-Test/B/complex/", list_path, ".ppm", append, 2);
    //generateListFromFolder(base_path + "Marcel-Test/MiniTrieschGallery/C/complex/", list_path, ".ppm", append, 3);
    //generateListFromFolder(base_path + "Marcel-Test/Five/complex/", list_path, ".ppm", append, 3);
    //generateListFromFolder(base_path + "Marcel-Test/Point/complex/", list_path, ".ppm", append, 3);
    //generateListFromFolder(base_path + "Marcel-Test/MiniTrieschGallery/V/complex/", list_path, ".ppm", append, 6);
}

void generateMarcelComplexTestListMini(string list_path) {
    bool append = false;
    generateListFromFolder(base_path + "Marcel-Test/MiniTrieschGallery/A/complex/", list_path, ".pgm", append, 1);
    append = true;
    generateListFromFolder(base_path + "Marcel-Test/MiniTrieschGallery/B/complex/", list_path, ".pgm", append, 2);
    //generateListFromFolder(base_path + "Marcel-Test/MiniTrieschGallery/C/complex/", list_path, ".pgm", append, 3);
    //generateListFromFolder(base_path + "Marcel-Test/MiniTrieschGallery/Five/complex/", list_path, ".pgm", append, 4);
    generateListFromFolder(base_path + "Marcel-Test/MiniTrieschGallery/Point/complex/", list_path, ".pgm", append, 3);
    //generateListFromFolder(base_path + "Marcel-Test/MiniTrieschGallery/V/complex/", list_path, ".pgm", append, 6);
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
    Mat image = imread(base_path + "Marcel-Test/A/complex/A-complex02.ppm", CV_LOAD_IMAGE_COLOR);
    HandSeg hs;
    Mat hand = hs.getHand(image);
    imshow("kikou", hand); waitKey(0);
}

void tuneSVM() {
    string train_list_path("data/train_list.txt");
    string test_list_path("data/test_list.txt");
    //generateMarcelUniformTrainList(train_list_path);
    generateMoeslundTrainList(train_list_path);
    generateMarcelComplexTestList(test_list_path);
    createVocabAndSaveData (train_list_path, test_list_path);
    mat2libsvm();
}

int main() {
    //tuneSVM();
    testSkinSeg();
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

