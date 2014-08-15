#include "SwiftHand.h"

using namespace std;
using namespace cv;

SwiftHand::SwiftHand() {}

void SwiftHand::tuneParameters (Mat train_mat, Mat train_labels,
                                 Mat test_mat, Mat test_labels,
                                 float min_c, float step_c, float max_c,
                                 float min_gamma, float step_gamma, float max_gamma) {
    int max_label = 3;
    for (float i = min_c; i < max_c; i *= step_c) {
        for (float j = min_gamma; j < max_gamma; j *= step_gamma) {
            cout << "Training..." << endl;
            cout << "C = " << i << "/ gamma = " << j << endl;
            train(train_mat, train_labels, i, j);
            cout << "Computing confusion matrix..." << endl;
            Mat conf_mat = confusionMatrix(test_mat, test_labels, max_label);
            cout << conf_mat << endl;
            cout << trace(conf_mat) << endl;
        }
    }
}

void SwiftHand::train (cv::Mat train_mat, Mat train_labels, float C, float gamma) {
    CvSVMParams params;
    params.svm_type = CvSVM::C_SVC;
    params.C = C;
    params.kernel_type = CvSVM::RBF; // best according to http://www.csie.ntu.edu.tw/~cjlin/libsvm/faq.html#f506
    params.gamma = gamma;
    params.term_crit = cvTermCriteria(CV_TERMCRIT_ITER, 100, 1e-6);
    mySVM_.train(train_mat, train_labels, Mat(), Mat(), params);
}

Mat SwiftHand::confusionMatrix(Mat test_mat, Mat labels, int max_label) {
    Mat confusion_matrix = Mat::zeros(max_label, max_label, CV_32F);
    for (int i = 0; i < test_mat.rows; ++i) {
        Mat bof = test_mat.row(i);
        float response = mySVM_.predict(bof);
        //cout << labels[i] << " " << static_cast<int>(response) << endl;
        int true_label = static_cast<int>(labels.at<float>(i)) - 1;
        int predicted_label = static_cast<int>(response) - 1;
        confusion_matrix.at<float>(true_label, predicted_label) += 1;
    }
    return confusion_matrix;
}
////////////////////////////////////////////////////////////////////////////////
// PRIVATE METHODS
////////////////////////////////////////////////////////////////////////////////

