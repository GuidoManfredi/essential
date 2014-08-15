#pragma once
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "HandSeg.h"
#include "Pipeline2D.h"

class Database {
  public:
    void saveMat (std::string path, cv::Mat mat);
    cv::Mat loadMat (std::string path);

    void createTrainData (std::string list_path,
                           cv::Mat &train_matrix, std::vector<float> &labels, bool skin_segmentation);
    void createTestData (std::string list_path,
                           cv::Mat &test_matrix, std::vector<float> &labels, bool skin_segmentation);
  private:
    cv::Mat createDataMatrix (std::vector<std::string> paths, bool create_vocabulary);
    void readFileList (std::string path, std::vector<std::string> &paths, std::vector<float> &labels);
    void path2images(std::vector<std::string> paths, std::vector<cv::Mat> &images);
    void images2bof(std::vector<cv::Mat> images, int vocab_size, cv::Mat &bofs);

    Pipeline2D pipeline_;
};
