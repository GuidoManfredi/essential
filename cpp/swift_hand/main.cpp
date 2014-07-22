#include <iostream>
#include <fstream>
#include <opencv2/ml/ml.hpp>
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

void testFileList (string path);
float testFile (string path, float label);
void readFileList (string path, vector<string> &paths, vector<float> &labels);
Mat createTrainingMat (vector<string> paths);
Mat createTestMat (vector<string> paths);
Mat confusionMatrix(Mat testMat, vector<float> labels, int max_label);

Pipeline2D pipeline;
CvSVM mySVM;

void generateMarcelLists() {
    generateList("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Train/A/","A-train",1,1329,".ppm", "1");
    generateList("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Train/B/","B-train",1,487,".ppm", "2");
    generateList("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Train/C/","C-train",1,572,".ppm", "3");
    generateList("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Train/Five/","Five-train",1,654,".ppm", "4");
    generateList("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Train/Point/","Point-train",1,1395,".ppm", "5");
    generateList("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Train/V/","V-train",1,435,".ppm", "6");
}

void generateMarcelTestLists() {
    generateList("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Test/A/complex/","A-complex",1,39,".ppm", "1");
    generateList("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Test/B/complex/","B-complex",1,41,".ppm", "0");
    /*
    generateList("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Train/C/","C-train",1,572,".ppm", "3");
    generateList("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Train/Five/","Five-train",1,654,".ppm", "4");
    generateList("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Train/Point/","Point-train",1,1395,".ppm", "5");
    generateList("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Train/V/","V-train",1,435,".ppm", "6");
    */
}

int main() {
    int gen = 0;
    if (gen == 1) {
        generateMarcelTestLists();
        //generateMarcelTrainLists();
    } else {
        vector<string> paths;
        vector<float> labels;
        //readFileList ("/home/gmanfred/devel/essential/cpp/swift_hand/data/train_images_list.txt", paths, labels);
        readFileList ("/home/gmanfred/devel/essential/cpp/swift_hand/data/marcel_train_list.txt", paths, labels);
        // Set up training data
        Mat labelsMat(paths.size(), 1, CV_32F, &labels[0]); // convert vector to array
        Mat trainingDataMat = createTrainingMat (paths);

        // Set up SVM's parameters
        CvSVMParams params;
        params.svm_type = CvSVM::C_SVC;
        params.C = 0.5;
        //params.svm_type    = CvSVM::ONE_CLASS;
        //params.nu = 0.9;
        params.kernel_type = CvSVM::LINEAR;
        //params.kernel_type = CvSVM::SIGMOID;
        //params.kernel_type = CvSVM::RBF;
        //params.kernel_type = CvSVM::POLY;
        //params.gamma = 0.6;
        params.term_crit = cvTermCriteria(CV_TERMCRIT_ITER, 100, 1e-6);

        // Train the SVM
        mySVM.train(trainingDataMat, labelsMat, Mat(), Mat(), params);
        // Create the test matrix
        readFileList ("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Test/A/complex/list.txt", paths, labels);
        Mat testMat = createTestMat(paths);
        int max_label = 5;
        // Test the SVM
        Mat confusion_matrix = confusionMatrix(testMat, labels, max_label);
        cout << confusion_matrix << endl;

        //testFileList("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Test/A/complex/list.txt");
        //testFileList("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Test/B/complex/list.txt");
    }

    return 0;
}

void testFileList (string path) {
    vector<string> paths;
    vector<float> labels;
    readFileList (path, paths, labels);
    float pos = 0;
    for (size_t i = 0; i < paths.size(); ++i) {
        pos += testFile(paths[i], labels[i]);
    }
    cout << "True positive = " << pos/paths.size()*100 << "%" << endl;
}

float testFile (string path, float label) {
    Mat test_image = imread (path, CV_LOAD_IMAGE_GRAYSCALE);
    if (test_image.empty())
        cout << "Could not load image in " << path << endl;
    vector<KeyPoint> kpts;
    pipeline.detectFeatures(test_image, kpts);
    Mat bof;
    pipeline.computeBoW (test_image, kpts, bof);
    float response = mySVM.predict(bof);
    cout << "Class " << response << "(Should be "<< label << ")."<< endl;
    return (response == label);
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

Mat createTrainingMat (vector<string> paths) {
    // save images in vector
    vector<Mat> training_images;
    Mat tmp_img;
    for (size_t i = 0; i < paths.size(); ++i) {
        tmp_img = imread(paths[i], CV_LOAD_IMAGE_GRAYSCALE);
        //imshow("kikou", tmp_img); waitKey(0);
        training_images.push_back(tmp_img);
    }
    // Create and set vocabulary
    Mat vocabulary;
    pipeline.createVocabulary (training_images, vocabulary);
    pipeline.setVocabulary(vocabulary);
    // Compute corresponding BoF
    Mat training_bofs (paths.size(), vocabulary.rows, CV_32F);
    Mat tmp_bof;
    vector<KeyPoint> tmp_kpts;
    for (size_t i = 0; i < paths.size(); ++i) {
        //imshow("kikou",training_images[i]); waitKey(0);
        pipeline.detectFeatures(training_images[i], tmp_kpts);
        pipeline.computeBoW (training_images[i], tmp_kpts, tmp_bof);
        tmp_bof.copyTo(training_bofs.row(i));
    }
    return training_bofs;
}

Mat createTestMat (vector<string> paths) {
    // save images in vector
    vector<Mat> test_images;
    Mat tmp_img;
    for (size_t i = 0; i < paths.size(); ++i) {
        tmp_img = imread(paths[i], CV_LOAD_IMAGE_GRAYSCALE);
        //imshow("kikou", tmp_img); waitKey(0);
        test_images.push_back(tmp_img);
    }
    // Compute corresponding BoF
    Mat test_bofs (paths.size(), pipeline.getVocabulary().rows, CV_32F);
    Mat tmp_bof;
    vector<KeyPoint> tmp_kpts;
    for (size_t i = 0; i < paths.size(); ++i) {
        //imshow("kikou",training_images[i]); waitKey(0);
        pipeline.detectFeatures(test_images[i], tmp_kpts);
        pipeline.computeBoW (test_images[i], tmp_kpts, tmp_bof);
        tmp_bof.copyTo(test_bofs.row(i));
    }
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
