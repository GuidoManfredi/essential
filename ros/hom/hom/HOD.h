#pragma once

#include <opencv2/core/core.hpp>
#include "Face.h"

class HOD {
  public:
    HOD();
    int find(cv::Mat frame, std::vector<cv::Mat> P);

    int loadFacesFromList(std::string list_path);
    int loadFace(std::string path);

    void setIntrinsics(cv::Mat K);
    void loadIntrinsics(std::string intrinsics_path);

    //std::vector<Face> getFaces(std::vector<Object> objs);
    //int find(Object obj, cv::Mat frame, cv::Mat P);
    //int loadObjectsList(std::string list_path);
    //int loadObject(std::string path);
  private:
    int findFaces(std::vector<Face> faces, cv::Mat frame, vector<cv::Mat> P);
    int findFace(Face face, cv::Mat frame, 
                 std::vector<cv::DMatch> &matches, cv::Mat P);
                 
    int ReadList(std::string list_path,
                 std::vector<std::string> paths);
                
    cv::Mat K_;
    std::vector<Faces> faces_;
    //std::vector<Object> objs_;
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
