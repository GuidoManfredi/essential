#include "TexturedPipeline.h"

namespace bf = boost::filesystem;
using namespace cv;
using namespace std;

TexturedPipeline::TexturedPipeline (Mat intrinsics): K_(intrinsics) {
	poser_ = new PoseEstimator (intrinsics);
	matcher_ = new Matcher ();
}

TexturedPipeline::~TexturedPipeline () {

}

void TexturedPipeline::load_objects (string object_models_dir) {
	try	{
		if (bf::exists (object_models_dir)) {
		  if (bf::is_directory (object_models_dir)) {
		    bf::directory_iterator end_itr;
		    for (bf::directory_iterator i (object_models_dir); i != end_itr; ++i) {
		    	Object current_object;
	        //current_object.load_data (i->path().string());
	        known_objects_.push_back (current_object);
	      }
		  }
		}
	}
	catch (const bf::filesystem_error& ex) {
		cout << ex.what() << endl;
	}
}

void TexturedPipeline::recognise_localise (Mat image) {
	recognise (image);
	localise (image);
}

void TexturedPipeline::recognise (Mat image) {
	recognised_objects_idx_.clear();
}

void TexturedPipeline::localise (Mat image) {
	transforms_.clear();
	for (size_t i=0; i<recognised_objects_idx_.size(); ++i) {
	  if (p3d_idx_[i].size () > 4) {
	  	int idx = recognised_objects_idx_[i];
  	  poser_->compute_pnp_ransac(known_objects_[idx]._p3d,
  	  													 current_kpts_,
  	  													 p3d_idx_[i], p2d_idx_[i], 2.0, 200);
  	  transforms_.push_back(poser_->get_P ());
  	}
  	else
    	cout << "Carod:Localise: not enough matches for pose estimation." << endl;
  }
}

std::vector<cv::Mat> TexturedPipeline::get_recognised_names () {
	return transforms_;
}

std::vector<std::string> TexturedPipeline::get_recognised_locations () {
	vector<string> names;
	for (size_t i=0; i<recognised_objects_idx_.size(); ++i)
		names.push_back (known_objects_[ recognised_objects_idx_[i] ]._name);
	return names;
}

