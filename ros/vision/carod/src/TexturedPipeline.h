#ifndef CAROD_TEXTUREX_PIPELINE_H_
#define CAROD_TEXTUREX_PIPELINE_H_

#include <boost/filesystem.hpp>

#include "Matcher.h"
#include "PoseEstimator.h"
#include "Object.h"

class TexturedPipeline {
 public:
 	TexturedPipeline (cv::Mat intrinsics);
 	~TexturedPipeline ();
 	
 	void load_objects (std::string objects_models_dir);
 	void recognise_localise (cv::Mat image);
 																		       //std::vector<std::string> candidates=NULL);
 	void recognise (cv::Mat image);
 									//std::vector<std::string> candidates=NULL);
 	void localise (cv::Mat image);
 								 //std::vector<std::string> objects_name);
	std::vector<std::string> get_recognised_locations ();
	std::vector<cv::Mat> get_recognised_names ();				 
 private:
  // Data storage variables
  std::vector<Object> known_objects_;
  cv::Mat K_;
  // Computation motor classes
  Matcher* matcher_;
  PoseEstimator* poser_;
  // Temporary processing values
  std::vector<cv::KeyPoint> current_kpts_;
  std::vector< std::vector<int> > p3d_idx_;
  std::vector< std::vector<int> > p2d_idx_;
  std::vector<int> candidate_objects_idx;
  // Output
  std::vector<cv::Mat> transforms_;
  std::vector<int> recognised_objects_idx_;
};

#endif
