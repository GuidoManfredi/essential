#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

class Face {
  public:
    Face();
    Face(cv::Mat img, int id_, std::vector<cv::KeyPoint> kpts, cv::Mat descs);
    void removeMatches(std::vector<cv::DMatch> matches);

    void write (cv::FileStorage& fs) const;
    void read (const cv::FileNode& node);
  //private:
    int id_;
    cv::Mat image_;
    cv::Size size_;
    std::vector<cv::Point3f> points3d_;
    std::vector<cv::KeyPoint> keypoints_;
    // Keypoints with some of them removed.
    std::vector<cv::KeyPoint> tmp_keypoints_;
    cv::Mat descriptors_;
    cv::Mat tmp_descriptors_;
};

static void write(cv::FileStorage& fs, const std::string&, const Face& x){
  x.write(fs);
}
static void read(const cv::FileNode& node, Face& x, const Face& default_value = Face()){
  if(node.empty())
    x = default_value;
  else
    x.read(node);
}
