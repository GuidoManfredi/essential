#pragma once

#include <iostream>
#include <fstream>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

class SaveCorners {
  public:
    SaveCorners (std::string object_dir);
    ~SaveCorners ();
    void saveCornersObject (std::string object_dir, int start_number, int step);
  private:
    void getCornersOnePicture (cv::Mat image);
    void saveCorners (std::vector<cv::Point2f> corners);
    void removeCornersFile (std::vector<boost::filesystem::path> v);

    std::ofstream file_;
    std::vector<boost::filesystem::path> v_;
};

static std::vector<cv::Point2f> corners;

static void mouse_cb( int event, int x, int y, int flags, void* param )
{
  // If button released
  if (event == CV_EVENT_LBUTTONUP)
  {
    if (corners.size () >= 4)
      corners.clear ();

    corners.push_back ( cv::Point2f(x, y));
    std::cout << "Corner " << corners.size () << " saved." << std::endl;
  }
}

static bool myCompare (boost::filesystem::path s1, boost::filesystem::path s2) {
    std::vector<std::string> parts1, parts2;
    boost::split(parts1, s1.string(), boost::is_any_of("_"));
    boost::split(parts2, s2.string(), boost::is_any_of("_"));
    std::vector<std::string> p1, p2;
    boost::split(p1, parts1.back(), boost::is_any_of("."));
    boost::split(p2, parts2.back(), boost::is_any_of("."));
    int i = atoi ( p1[0].c_str() );
    int j = atoi ( p2[0].c_str() );
    return (i<j);
}

