#pragma once

#include "Pipeline2D.h"
#include "Containers.h"

class Engine {
  public:
    Engine();
    std::vector<int> match (Model model, Object object);
    std::vector<int> match (cv::Mat descriptors, std::vector<View> image);

    std::vector<int> match (Object model, Object object);
    std::vector<int> match (std::vector<View> model_views, std::vector<View> object_views);
    std::vector<int> match2 (std::vector<View> model_views, std::vector<View> object_views);

    Model modelFromObject (Object object, std::vector<int> model_images);
    Object objectFromObject (Object object, std::vector<int> images);
    void sortViewByAngle(Object &object);
    int getIdxFromAngle (Object object, float angle);

  private:
    struct IndexDistanceComparatorClass {
        bool operator() (const View& l, const View& r) {
            return l.angle_ < r.angle_;
        }
    } Comparator;

    Pipeline2D pipe2d_;
};
