#include "Engine.h"

using namespace std;
using namespace cv;

Engine::Engine (Pipeline2D* pipe): pipe2d_(pipe) {
}

void Engine::setFeatures(Feature ft) {
    pipe2d_->setFeatures(ft);
}

vector<int> Engine::match (Model model, Object object) {
    return match (model.descriptors_, object.views_);
}

int Engine::match (Object model, Object object,
                   vector<Error> &errors) {
    errors.clear();
    return match (model.views_, object.views_, errors);
}

vector<int> Engine::match (Mat model_descriptors, vector<View> object_views) {
    vector<int> results (object_views.size());

    int number_matches = 0;
    for (size_t i = 0; i < object_views.size(); ++i) {
        vector<DMatch> matches;
        number_matches = pipe2d_->match (model_descriptors, object_views[i].descriptors_,
                                        matches);
        results[i] = number_matches;
    }
    return results;
}

int Engine::match (vector<View> model_views, vector<View> object_views,
                    vector<Error> &errors) {
    errors.resize(object_views.size());

    for (size_t i = 0; i < object_views.size(); ++i) {
        vector<int> number_matches (model_views.size());
        vector<float> rotation_error (model_views.size());
        for (size_t j = 0; j < model_views.size(); ++j) {
            vector<DMatch> matches;
            pipe2d_->match (model_views[j].descriptors_, object_views[i].descriptors_, matches);

            float pose_rad = 0;
            if (matches.size() > 6)
                pose_rad = pipe2d_->estimate_pose2 (model_views[j].keypoints_, object_views[i].keypoints_, matches);
                //float pose_rad = pipe2d_->estimate_pose (model_views[j].keypoints_, object_views[i].points_, matches);
/*
            Mat img;
            drawMatches (model_views[j].image_, model_views[j].keypoints_, object_views[i].image_, object_views[i].keypoints_, matches, img);
            imshow("Debug", img); waitKey(0);
*/
            rotation_error[j] = pose_rad;
            number_matches[j] = matches.size();
        }
        errors[i].N_ = *max_element(number_matches.begin(), number_matches.end());
        errors[i].P_ = static_cast<float>(errors[i].N_) / static_cast<float>(object_views[i].keypoints_.size()) * 100;
        errors[i].Rerr_ = *min_element(rotation_error.begin(), rotation_error.end());
    }

    return 0;
}

Model Engine::modelFromObject (Object object, vector<int> model_images) {
    Model model;

    for (int i = 0; i < model_images.size(); ++i) {
        int idx = model_images[i];
        if (i == 0)
            model.descriptors_ = object.views_[idx].descriptors_;
        else
            vconcat(model.descriptors_, object.views_[idx].descriptors_, model.descriptors_);
    }

    //cout << model.descriptors_.size() << endl;
    return model;
}

Object Engine::objectFromObject (Object object, vector<int> images) {
    Object res;

    for (int i = 0; i < images.size(); ++i) {
        int idx = images[i];
        res.views_.push_back(object.views_[idx]);
    }

    return res;
}

void Engine::sortViewByAngle(Object &object) {
    sort (object.views_.begin(), object.views_.end(), Comparator);
//    for (size_t i = 0; i < object.views_.size(); ++i)
//        cout << object.views_[i].angle_ << endl;
}

int Engine::getIdxFromAngle (Object object, float angle, int tilt) {
    assert( (tilt != 1 || tilt != 2 || tilt != 4) && "Error: getIdxFromAngle: invalid tilt");
    float min_diff = 360.0;
    int min_idx = 0;
    for (size_t i = 0; i < object.views_.size(); ++i) {
        if ( object.views_[i].tilt_ == tilt ) {
            //imshow("Debug", object.views_[i].image_); waitKey(0);
            //cout << object.views_[i].angle_ << endl;
            float diff = fabs(object.views_[i].angle_ - angle);
            if (diff < min_diff) {
                min_diff = diff;
                min_idx = i;
            }
        }
    }
    return min_idx;
}

vector<Error> Engine::getMean (vector<vector<Error> > errors) {
    //int num_views = errors[0].size();
    int num_views = 120;
    int num_objects = errors.size();
    vector<Error> mean;
    mean.resize(num_views);

    for (size_t n = 0; n < num_views; ++n) {
        int count = 0;
        for (size_t i = 0; i < num_objects; ++i) {
            if ( n < errors[i].size()) {
                //if (errors[i][n].P_ > 100) cout << n << " " << i << " " << errors[i].size() << " " << errors[i][n].P_ <<endl;
                mean[n].N_ += errors[i][n].N_;
                mean[n].P_ += errors[i][n].P_;
                mean[n].Rerr_ += errors[i][n].Rerr_;
                ++count;
            }
        }

        if (count != 0) {
            mean[n].N_ /= count;
            mean[n].P_ /= count;
            mean[n].Rerr_ /= count;
        } else {
            mean[n].N_ = 0;
            mean[n].P_ = 0;
            mean[n].Rerr_ = 0;
        }
        //cout << count << endl;
    }
    return mean;
}

void Engine::save(std::string out_basename, std::vector<Error> error) {
    std::string name = out_basename + "_error.csv";

    fstream stream;
    stream.open(name.c_str(), std::fstream::out);
    //    cout << "Could not open file " << out << endl;

    for (size_t i = 0; i < error.size(); ++i)
        stream << error[i].N_ << "," << error[i].P_ << "," << error[i].Rerr_ << endl;
    stream.close();
}
