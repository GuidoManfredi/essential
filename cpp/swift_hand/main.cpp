#include <iostream>
#include <fstream>
#include <opencv2/ml/ml.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "Pipeline2D.h"
#include "generateList.h"

// TODO
// Faire un createTestMat qui va mettre un bof du test set dans chaque
// ligne d'une matrice.
// Faire une fonction qui a partir d'une testMat calcule la confusion matrix
//  + des fonctions pour avoir les trues positives et false positive (ROC curve).

using namespace std;
using namespace cv;

void mat2file(Mat matrix, string path);
void testFileList (string path);
float testFile (string path, float label);
void readFileList (string path, vector<string> &paths, vector<float> &labels);
Mat createTrainMat (vector<string> paths);
Mat createTestMat (vector<string> paths);
Mat confusionMatrix(Mat testMat, vector<float> labels, int max_label);
void mat2file(Mat matrix, string path);
void path2images(vector<string> paths, vector<Mat> &images);
void images2bof(vector<Mat> images, int vocab_size, Mat &bofs);

Pipeline2D pipeline;
CvSVM mySVM;

void generateMarcelTrainLists() {
    generateList("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Train/A/","A-train",1,1329,".ppm", "1");
    generateList("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Train/B/","B-train",1,487,".ppm", "2");
    generateList("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Train/C/","C-train",1,572,".ppm", "3");
    generateList("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Train/Five/","Five-train",1,654,".ppm", "4");
    generateList("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Train/Point/","Point-train",1,1395,".ppm", "5");
    generateList("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Train/V/","V-train",1,435,".ppm", "6");
}

void generateMarcelTestLists() {
    // 39 41 47 58 54 38
    generateList("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Test/A/complex/","A-complex",1,39,".ppm", "1");
    generateList("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Test/B/complex/","B-complex",1,41,".ppm", "2");
    generateList("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Test/C/complex/","C-complex",1,47,".ppm", "3");
    generateList("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Test/Five/complex/","Five-complex",1,58,".ppm", "4");
    generateList("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Test/Point/complex/","Point-complex",1,54,".ppm", "5");
    generateList("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Test/V/complex/","V-complex",1,38,".ppm", "6");
}

int main() {
    int gen = 0;
    if (gen == 1) {
        //generateMarcelTrainLists();
        //generateMarcelTestLists();
    } else {
        vector<string> paths;
        vector<float> labels;
        // Create the train matrix
        cout << "Computing train matrix" << endl;
        readFileList ("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Train/marcel_train_list.txt", paths, labels);
        Mat labelsMat(paths.size(), 1, CV_32F, &labels[0]); // convert vector to array
        Mat trainMat = createTrainMat (paths);
        // Create the test matrix
        cout << "Computing test matrix" << endl;
        readFileList ("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Test/marcel_test_list.txt", paths, labels);
        Mat testMat = createTestMat(paths);

        // Set up SVM's parameter
        CvSVMParams params;
        params.svm_type = CvSVM::C_SVC;
        //params.C = 1e6;
        //params.svm_type    = CvSVM::ONE_CLASS;
        params.kernel_type = CvSVM::RBF; // according to http://www.csie.ntu.edu.tw/~cjlin/libsvm/faq.html#f506
        params.gamma = 1;
        params.term_crit = cvTermCriteria(CV_TERMCRIT_ITER, 100, 1e-6);

        // Train the SVM
        cout << "Training SVM" << endl;
        mySVM.train(trainMat, labelsMat, Mat(), Mat(), params);
        // Test the SVM
        cout << "Testing SVM" << endl;
        int max_label = 6;
        Mat confusion_matrix = confusionMatrix(testMat, labels, max_label);
        cout << confusion_matrix << endl;

        mat2file(trainMat, "trainMat.txt");
        mat2file(testMat, "testMat.txt");
        mat2file(confusion_matrix, "confusionMat.txt");
    }

    return 0;
}

void readFileList (string path,
                   vector<string> &paths,
                   vector<float> &labels) {
    paths.clear();  labels.clear();
    vector<string> list;
    string line;
    ifstream myfile (path.c_str());
    if (myfile.is_open()) {
        string path;
        string label;
        while ( getline (myfile,line) ) {
            stringstream ss(line);
            ss >> path;
            ss >> label;
            paths.push_back(path);
            labels.push_back(atof(label.c_str()));
        }
        myfile.close();
    }
    else cout << "Unable to open " << path << endl;

    return;
}

Mat createTrainMat (vector<string> paths) {
    // save images in vector
    vector<Mat> train_images;
    path2images(paths, train_images);
    // Create and set vocabulary
    Mat vocabulary;
    pipeline.createVocabulary (train_images, vocabulary);
    pipeline.setVocabulary(vocabulary);
    // Compute corresponding BoFs
    Mat train_bofs;
    images2bof(train_images, vocabulary.rows, train_bofs);

    return train_bofs;
}

Mat createTestMat (vector<string> paths) {
    // save images in vector
    vector<Mat> test_images;
    path2images(paths, test_images);
    // Compute corresponding BoF
    Mat test_bofs;
    images2bof(test_images, pipeline.getVocabulary().rows, test_bofs);

    return test_bofs;
}

Mat confusionMatrix(Mat testMat, vector<float> labels, int max_label) {
    Mat confusion_matrix = Mat::zeros(max_label, max_label, CV_16S);
    for (int i = 0; i < testMat.rows; ++i) {
        Mat bof = testMat.row(i);
        float response = mySVM.predict(bof);
        //cout << labels[i] << " " << static_cast<int>(response) << endl;
        int true_label = static_cast<int>(labels[i]) - 1;
        int predicted_label = static_cast<int>(response) - 1;
        confusion_matrix.at<int>(true_label, predicted_label) += 1;
    }
    return confusion_matrix;
}

void mat2file(Mat matrix, string path) {
    ofstream file;
    file.open (path.c_str());
    file << matrix << endl;
    file.close();
}

void path2images(vector<string> paths, vector<Mat> &images) {
    for (size_t i = 0; i < paths.size(); ++i) {
        Mat tmp_img;
        Mat tmp_small_img;
        //cout << paths[i] << endl;
        tmp_img = imread(paths[i], CV_LOAD_IMAGE_GRAYSCALE);
        resize(tmp_img, tmp_small_img, Size(256,256));
        images.push_back(tmp_small_img);
    }
}

void images2bof(vector<Mat> images, int vocab_size, Mat &bofs) {
    bofs = Mat::zeros(images.size(), vocab_size, CV_32F);
    Mat tmp_bof;
    vector<KeyPoint> tmp_kpts;
    for (size_t i = 0; i < images.size(); ++i) {
        //imshow("kikou",images[i]); waitKey(0);
        pipeline.detectFeatures(images[i], tmp_kpts);
        pipeline.computeBoW (images[i], tmp_kpts, tmp_bof);
        //cout << tmp_bof << endl;
        tmp_bof.copyTo(bofs.row(i));
    }
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
