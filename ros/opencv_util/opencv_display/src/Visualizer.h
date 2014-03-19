#ifndef OPENCV_DISPLAY_VISUALIZER_H_
#define OPENCV_DISPLAY_VISUALIZER_H_

#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

void draw_callback(void* param);

class Visualizer
{
public:
  Visualizer();
  Visualizer(std::string windowName, cv::Size frame_size, cv::Mat K);
  ~Visualizer();

	cv::Mat object_pose;
	std::string name_pose;	

  // Set the new frame for the background
  void updateBackground(const cv::Mat& frame);
  void updatePose(const cv::Mat& pose);
  void updateWindow();
  
  void setK (cv::Mat K);
  void setSize (int width, int height);

private:
	friend void draw_callback(void* param);
  // Render entire scene in the OpenGl window
  void draw();

  // Draws the background with video
  void drawCameraFrame();

  // Draws the AR
  void drawAugmentedScene();

  // Builds the right projection matrix from the camera calibration
  void buildProjectionMatrix(cv::Mat &result);
  
  // Draws the coordinate axis 
  void drawCoordinateAxis();

  bool               isTextureInitialized_;
  unsigned int       backgroundTextureId_;
  cv::Mat						 K_;
  cv::Mat            backgroundImage_;
  std::string        windowName_;
  int 							 width_, height_;
};

#endif
