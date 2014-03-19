#include "POD.h"

using namespace std;
using namespace cv;

POD::POD(Mat K) : K_(K) {
  ;
}

void POD::train(const PlanarObject& object)
{
  ;
  //objects_.push_back(object);

  //cv::Mat descriptors = object.descriptors.clone();
  //pipeline_.add_descriptors(descriptors);
}
