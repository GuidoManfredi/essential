#pragma once

#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

class FilesManager
{
  public:
    FilesManager ();
    void getImagesAndFaces (std::string images_path,
                            std::vector<cv::Mat> &images, std::vector<int> &faces);
    std::vector<cv::Mat> getImages (std::string images_path);
    std::vector<std::vector<cv::Point2f> > getCorners (std::string corners_file);
    cv::Point3f getDimensions (std::string corners_file);
    std::string getDirName (std::string train_dir);
  private:
    cv::Mat getImage (std::string image_path);
};
