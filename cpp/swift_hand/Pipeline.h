#pragma once

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include "Posture.h"
#include "Pipeline2d.h"

class Pipeline {
    Pipeline();

    /**
     *  \brief Create a posture class and fill it with the keypoints and descriptors
     *          extracted in the image loaded from the given file.
     */
    Posture createPosture(std::string path);
    /**
     *  \brief Create a posture class and fill it with the keypoints and descriptors
     *          extracted from the given image.
     */
    Posture createPosture(cv::Mat img);
    /**
     *  \brief Match a posture with a vector of postures and return the indice of the posture
     *          with more common descriptors.
     */
    int matchPostures(Posture p1, std::vector<Posture> ps);
    /**
     *  \brief Match the descriptors from two postures and returns the number of inliers
     */
    int matchPostures(Posture p1, Posture p2);

    Pipeline2d pipe2d;
};
