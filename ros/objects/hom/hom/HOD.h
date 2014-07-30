#pragma once

#include <opencv2/core/core.hpp>
#include "HOM.h"
#include "FilesManager.h"
//#include "Pipeline2D.h"
#include "PipelineGpu2D.h"
#include "PipelineGeom.h"

class HOD {
  public:
    HOD();

    int find(cv::Mat image, std::vector<cv::Mat> &P);

    int loadFacesFromList(std::string list_path);
    int loadFace(std::string path);

    void setIntrinsics(cv::Mat K);
    void loadIntrinsics(std::string intrinsics_path);

  //private:
    int findFaces(Face frame, std::vector<Face> faces, std::vector<cv::Mat> &Ps);
    int findFace(Face frame, Face face,
                 std::vector<cv::DMatch> &inlier, cv::Mat &P);

    FilesManager files_;
    cv::Mat K_;
    std::vector<Face> faces_;
    PipelineGeom geopip_;
    //Pipeline2D pip2d_;
    PipelineGpu2D pip2d_;
    HOM hom_;
};

/*
vector<Mat> poses;
for(size_t i = 0; i < faces.size(); ++i) {
    while(found == 1) {
        Mat P;
        int found = find(faces[i], frame, matches, P);
        if (found == 1) {
            poses.push_back(P);
            removeMatches(matches, frame);
        }
    }
}

std::vector<Face> getFaces(std::vector<Object> objs);

int find(faces[i], frame, matches, P) {
    int matched = match(faces[i].descriptors_, frame.descriptors, matches);
    int localised = computeHomography(faces[i].keypoints_, frame.keypoints_, matches, P);
    if (matched && localised)
        return 1;
    else
        return 0;
}
*/
