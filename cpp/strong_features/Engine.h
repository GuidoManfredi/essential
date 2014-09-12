#pragma once

#include <fstream>

#include <opencv2/highgui/highgui.hpp>

#include "Pipeline2D.h"
#include "Containers.h"

class Engine {
  public:
    Engine(Pipeline2D* pipe);
    void setFeatures(Feature ft);

    std::vector<int> match (Model model, Object object);
    std::vector<int> match (cv::Mat descriptors, std::vector<View> image);

    int match (Object model, Object object,
                 std::vector<int> &number_matches,
                 std::vector<float> &percent_matches,
                 std::vector<float> &rotation_error);
    int match (std::vector<View> model_views, std::vector<View> object_views,
                 std::vector<int> &number_matches,
                 std::vector<float> &percent_matches,
                 std::vector<float> &rotation_error);

    Model modelFromObject (Object object, std::vector<int> model_images);
    Object objectFromObject (Object object, std::vector<int> images);
    void sortViewByAngle(Object &object);
    int getIdxFromAngle (Object object, float angle, int tilt);

    void save(std::string file, std::vector<int> num_matches, std::vector<float> matches_percent);
    void saveVector(std::string out, std::vector<int> vec);
    void saveVector(std::string out, std::vector<float> vec);

  private:
    struct IndexDistanceComparatorClass {
        bool operator() (const View& l, const View& r) {
            return l.angle_ < r.angle_;
        }
    } Comparator;

    Pipeline2D* pipe2d_;
};
