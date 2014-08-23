#pragma once

#include "Pipeline2D.h"
#include "Containers.h"

class Engine {
  public:
    Engine();
    std::vector<int> match (Model model, Object object);
    std::vector<int> match (cv::Mat descriptors, std::vector<View> image);

    Model modelFromObject (Object object, std::vector<int> model_images);
    void sortViewByAngle(Object &object);

  private:
    struct IndexDistanceComparatorClass {
        bool operator() (const View& l, const View& r) {
            return l.angle_ < r.angle_;
        }
    } Comparator;

    Pipeline2D pipe2d_;
};
