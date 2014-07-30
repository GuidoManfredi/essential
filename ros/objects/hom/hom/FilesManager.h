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
    void getPathsFromList(std::string list_path,
                           std::vector<std::string> &paths);
    std::vector<cv::Mat> getImagesFromList(std::string list_path);
    // If images_path is an image return a vector with a single image.
    //  If images_path is a folder, returns a ordered vector of images.
    std::vector<cv::Mat> getImages (std::string images_path);
    std::string getName (std::string file_or_dir_path);

  private:
    cv::Mat getImage (std::string image_path);

};
