#pragma once

#include <pcl/recognition/linemod.h>
#include <pcl/recognition/color_gradient_modality.h>
#include <pcl/recognition/surface_normal_modality.h>

class VirtualTrain {
  public:
    VirtualTrain();
    void loadObject(std::string model);
    void model(int hSampling, int vSampling);
    
  private:
    modeler_;
};
