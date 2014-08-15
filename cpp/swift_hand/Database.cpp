#include "Database.h"

using namespace std;
using namespace cv;

void Database::saveMat (string path, Mat mat) {
    cv::FileStorage fs(path.c_str(), cv::FileStorage::WRITE);
    fs << "Matrix" << mat;
    fs.release();
}

Mat Database::loadMat (string path) {
    cv::FileStorage fs(path.c_str(), cv::FileStorage::READ);
    Mat mat;
    fs["Matrix"] >> mat;
    fs.release();
    return mat;
}

void Database::createTrainData (string list_path,
                                 Mat &train_matrix, vector<float> &labels, bool skin_segmentation) {
    labels.clear();
    vector<string> paths;
    readFileList (list_path, paths, labels);
    train_matrix = createDataMatrix(paths, true);
}

void Database::createTestData (string list_path,
                                Mat &test_matrix, vector<float> &labels, bool skin_segmentation) {
    labels.clear();
    vector<string> paths;
    readFileList (list_path, paths, labels);
    test_matrix = createDataMatrix(paths, false);
}
////////////////////////////////////////////////////////////////////////////////
// PRIVATE METHODS
////////////////////////////////////////////////////////////////////////////////
Mat Database::createDataMatrix (vector<string> paths, bool create_vocab) {
    // save images in vector
    vector<Mat> images;
    cout << "Loading images...";
    path2images(paths, images);
    cout << "loaded " << images.size() << " images" << endl;
    if (create_vocab) {
        // Create and set vocabulary
        cout << "Creating vocabulary...";
        Mat vocabulary;
        pipeline_.createVocabulary (images, vocabulary);
        pipeline_.saveVocabulary("vocab.yaml");
        pipeline_.setVocabulary(vocabulary);
        cout << "vocabulary has " << vocabulary.rows << " words." << endl;
    } else {
        ;//pipeline_.loadVocabulary("vocab.yaml"); // nothing for now. Later, load vocabulary
    }
    // Compute corresponding BoFs
    cout << "Computing bofs" << endl;
    Mat bofs;
    images2bof(images, pipeline_.getVocabulary().rows, bofs);

    return bofs;
}

void Database::readFileList (string path,
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

void Database::path2images(vector<string> paths, vector<Mat> &images) {
    for (size_t i = 0; i < paths.size(); ++i) {
        Mat tmp_img;
        Mat tmp_small_img;
        //cout << paths[i] << endl;
        tmp_img = imread(paths[i], CV_LOAD_IMAGE_COLOR);
        resize(tmp_img, tmp_small_img, Size(128,128));
        images.push_back(tmp_small_img);
        //images.push_back(tmp_img);
    }
}

void Database::images2bof(vector<Mat> images, int vocab_size, Mat &bofs) {
    bofs = Mat::zeros(images.size(), vocab_size, CV_32F);
    Mat tmp_bof;
    vector<KeyPoint> tmp_kpts;
    for (size_t i = 0; i < images.size(); ++i) {
        Mat gray;
        pipeline_.getGray(images[i], gray);
        //imshow("kikou",images[i]); waitKey(0);
        pipeline_.detectFeatures(gray, tmp_kpts);
        //cout << tmp_kpts.size() << endl;
        pipeline_.computeBoW (gray, tmp_kpts, tmp_bof);
        //cout << tmp_bof.cols << endl;
        tmp_bof.copyTo(bofs.row(i));
    }
}

