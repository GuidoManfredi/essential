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
    //cout << object_views.size() << endl;
    //bool eq = false;
    for (size_t i = 0; i < object_views.size(); ++i) {
        vector<int> number_matches (model_views.size());
        vector<float> rotation_error (model_views.size());
        int start = cv::getTickCount(); // Start counting time
        for (size_t j = 0; j < model_views.size(); ++j) {
            vector<DMatch> matches;
            pipe2d_->match (model_views[j].descriptors_, object_views[i].descriptors_, matches);

            float pose_rad = 0;
            if (matches.size() > 6)
                pose_rad = pipe2d_->estimate_pose2 (model_views[j].keypoints_, object_views[i].keypoints_, matches);
                //float pose_rad = pipe2d_->estimate_pose (model_views[j].keypoints_, object_views[i].points_, matches);

            rotation_error[j] = pose_rad;
            number_matches[j] = matches.size();
        }
        int end = cv::getTickCount();
        float time_period = 1 / cv::getTickFrequency();
        errors[i].N_ = *max_element(number_matches.begin(), number_matches.end());

        //cout << "Max matches: " << errors[i].N_ << endl;
        //cout << object_views[i].keypoints_.size() << endl;
        if (object_views[i].keypoints_.size() == 0)
            errors[i].P_ = 0;
        else {
            errors[i].P_ = static_cast<float>(errors[i].N_) / static_cast<float>(object_views[i].keypoints_.size()) * 100;
            //cout << errors[i].N_ << " " << object_views[i].keypoints_.size() << endl;
            //cout << static_cast<float>(errors[i].N_) << " " << static_cast<float>(object_views[i].keypoints_.size()) << endl;
            //cout << errors[i].P_ << endl;
        }
        //errors[i].Rerr_ = *min_element(rotation_error.begin(), rotation_error.end());
        errors[i].angle_ = object_views[i].angle_;
        errors[i].time_ = (end - start) * time_period;
        //cout << errors[i].time_ << endl;
    }

    return 0;
}

Model Engine::modelFromObject (Object object, vector<int> model_images) {
    Model model;

    for (size_t i = 0; i < model_images.size(); ++i) {
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

    res.views_.resize(images.size());
    for (size_t i = 0; i < images.size(); ++i) {
        int idx = images[i];
        //cout << object.views_[idx].descriptors_.rows << endl;
        res.views_[i] = object.views_[idx];
        //cout << res.views_[i].descriptors_.rows << endl;
    }

    //cout << res.views_[0].descriptors_.rows << endl;
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
            //cout << object.views_[i].angle_ << " " << angle << " " << diff << " " << i << endl;
            if (diff < min_diff) {
                min_diff = diff;
                min_idx = i;
            }
        }
    }
    //cout << min_diff << endl;
    //cout << min_idx << endl;
    return min_idx;
}

vector<Error> Engine::getMean (vector<vector<Error> > errors) {
    //int num_views = errors[0].size();
    //cout << errors[0].size() << endl;
    //int num_views = 120;
    int num_angles = 360;
    int num_objects = errors.size();
    vector<Error> mean;
    mean.resize(num_angles);
    vector<int> count;
    count.resize(num_angles);

    float total_time = 0.0;
    for (size_t o = 0; o < num_objects; ++o) {
        //count.clear();
        for (size_t v = 0; v < errors[o].size(); ++v) {
            int idx = static_cast<int>(errors[o][v].angle_);
            //cout << errors[o][v].angle_ << " " << idx << endl;
            mean[idx].N_ += errors[o][v].N_;
            mean[idx].P_ += errors[o][v].P_;
            mean[idx].angle_ += errors[o][v].angle_;
            mean[idx].time_ += errors[o][v].time_;
            //cout << errors[o][v].angle_ << endl;
            //cout << o << " " << v << " " << idx << " " << errors[o][v].N_ <<endl;
            ++count[idx];
        }
        //cout << count << endl;
    }

    for (size_t i = 0; i < num_angles; ++i)
        if (count[i] != 0) {
            mean[i].N_ /= count[i];
            mean[i].P_ /= count[i];
            mean[i].angle_ /= count[i];
            mean[i].time_ /= count[i];
        } else {
            mean[i].N_ = 0;
            mean[i].P_ = 0;
            mean[i].angle_ = 0;
            mean[i].time_ = 0;
        }

    return mean;
}

void Engine::saveTimeSize (string output, float time, int size) {
    fstream stream;
    stream.open(output.c_str(), std::fstream::app|std::fstream::out);
    //cout << time << " " << size << endl;
    stream << time << ";" << size << endl;

    stream.close();
}

void Engine::save(std::string out_basename, std::vector<Error> error) {
    std::string name = out_basename + "_error.csv";

    fstream stream;
    stream.open(name.c_str(), std::fstream::out);

    for (size_t i = 0; i < error.size(); ++i) {
        stream << error[i].N_ << "," << error[i].P_ << "," << error[i].angle_ << endl;
    }
    stream.close();
}

int Engine::getObjectSize(Object obj) {
    int size = 0;
    for (size_t i = 0; i < obj.views_.size(); ++i) {
        size += obj.views_[i].descriptors_.rows
                * obj.views_[i].descriptors_.cols
                * sizeof(float);
        //cout << size << endl;
    }

    return size;
}
