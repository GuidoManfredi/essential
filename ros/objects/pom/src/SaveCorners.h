#pragma once

#include <iostream>
#include <fstream>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

/**
*   This class allows reading a directory with images and displing them one by one.
*   For each image, one selects the four corners of an object of interest.
*   The corners are selected in the given order: up-left, up-right, down-left, down-right.
*   The coordinates of these corners are saved in a text file and can later be 
*   used to model the object with the POM software.
*/
class SaveCorners {
  public:
    /** 
    *   \brief Constructor
    *   \param object_dir A directory with images corresponding to one object.  
    */
    SaveCorners (std::string object_dir);
    /** 
    *   \brief Destructor
    */
    ~SaveCorners ();
    /**
    *   \brief For any .jpg or .png image in the initial folder, display it for
    *           processing and save the resulting corners.
    */
    void saveCornersObject ();
  private:
    /**
    *   \brief Display a picture and wait for the user to mark four corners.
    *   \param image The image to display.
    */
    void getCornersOnePicture (cv::Mat image);
    /**
    *   \brief Save a list of corners in a text file called "corners.txt" and located
    *           in the images directory.
    *   \param image The image to display.
    */    
    void saveCorners (std::vector<cv::Point2f> corners);
    /**
    *   \brief Remove the file created by saveCorners.
    */
    void removeCornersFile (std::vector<boost::filesystem::path> v);
    /// A file, called "corners.txt" into which corners coordinates are saved.
    std::ofstream file_;
    /// A vector of path to each image in the initial folder.
    std::vector<boost::filesystem::path> v_;
};
/// The corners to acquire and save.
static std::vector<cv::Point2f> corners;
/**
*   \brief A mouse callback to acquire the corners pointed by the user.
*/
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
/**
*   \brief Allows comparing filenames which name end with a number, based only
*           on the number value.
*/
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

