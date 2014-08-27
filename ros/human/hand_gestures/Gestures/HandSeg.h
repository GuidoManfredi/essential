#pragma once

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
// PointCloud -> Human, Human -> Hands, Hands -> Mask, Mask -> Gesture
class HandSeg {
  public:
    HandSeg ();
    int getGesture (cv::Mat depth, cv::Mat mask);


  //private:
    // Return the histogram from a depth image.
    cv::Mat getZhistogram(cv::Mat depth, cv::Mat mask);
    // Return a mask of the part of the image belonging to the hand.
    cv::Mat cropHand (cv::Mat z_histogram);
    // Return the fartest Z belonging to the hand
    float getHandMaxZ (cv::Mat z_histogram);
    // Project the cropped hand to the image plan to get a 2D image.
    //Mat project2CameraPlan(cv::Mat hand);
    int gesture (cv::Mat mask, cv::Mat &display);
};
