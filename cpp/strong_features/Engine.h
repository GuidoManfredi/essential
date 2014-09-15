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
                 std::vector<Error> &errors);
    int match (std::vector<View> model_views, std::vector<View> object_views,
                 std::vector<Error> &errors);

    Model modelFromObject (Object object, std::vector<int> model_images);
    Object objectFromObject (Object object, std::vector<int> images);
    void sortViewByAngle(Object &object);
    int getIdxFromAngle (Object object, float angle, int tilt);

    vector<Error> getMean (std::vector<std::vector<Error> > errors);

    void save(std::string file, std::vector<Error> error);

  private:
    struct IndexDistanceComparatorClass {
        bool operator() (const View& l, const View& r) {
            return l.angle_ < r.angle_;
        }
    } Comparator;

    Pipeline2D* pipe2d_;
};
