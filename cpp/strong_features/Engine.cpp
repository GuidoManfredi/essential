#include "Engine.h"

using namespace std;
using namespace cv;

Engine::Engine () {}

void Engine::setFeatures(Feature ft) {
    pipe2d_.setFeatures(ft);
}

vector<int> Engine::match (Model model, Object object) {
    return match (model.descriptors_, object.views_);
}

vector<int> Engine::match (Object model, Object object) {
    return match (model.views_, object.views_);
}

vector<int> Engine::match (Mat model_descriptors, vector<View> object_views) {
    vector<int> results (object_views.size());

    int number_matches = 0;
    for (size_t i = 0; i < object_views.size(); ++i) {
        number_matches = pipe2d_.match (model_descriptors, object_views[i].descriptors_);
        results[i] = number_matches;
    }
    return results;
}

vector<int> Engine::match (vector<View> model_views, vector<View> object_views) {
    vector<int> results (object_views.size());

    for (size_t i = 0; i < object_views.size(); ++i) {
        vector<int> number_matches (model_views.size());
        for (size_t j = 0; j < model_views.size(); ++j) {
            number_matches[j] = pipe2d_.match (model_views[j].descriptors_, object_views[i].descriptors_);
        }
        results[i] = *max_element(number_matches.begin(), number_matches.end());
    }
    return results;
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

int Engine::getIdxFromAngle (Object object, float angle) {
    float min_diff = 360.0;
    int min_idx = 0.0;
    for (size_t i = 0; i < object.views_.size(); i+=3) {
        //cout << object.views_[i].angle_ << endl;
        float diff = fabs(object.views_[i].angle_ - angle);
        if (diff < min_diff) {
            min_diff = diff;
            min_idx = i;
        }
    }
    return min_idx;
}
