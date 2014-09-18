#include <iostream>

#include "FilesManager.h"
#include "Engine.h"

// TODO faire que compute traite correctement toutes les classes dans sa boucle "for"
using namespace std;
using namespace cv;

// cereal_box, food_bag, food_box, food_can, instant_noodles, soda_can, toothpast

// 570 for IR
// 535 for RGB
//Mat K =  (Mat_<double>(3,3) << 570, 0, 320, 0, 570, 240, 0, 0, 1);
Mat K =  (Mat_<double>(3,3) << 535, 0, 320, 0, 535, 240, 0, 0, 1);
Pipeline2D* pipe2d = new Pipeline2D(K);
FilesManager files(K, pipe2d);
Engine engine(pipe2d);
/*
Object common (string object_path) {
    cout << "Loading data and creating object" << endl;
    Object object = files.loadObject (object_path);
    cout << "Sorting object" << endl;
    engine.sortViewByAngle(object);

    idx.push_back(engine.getIdxFromAngle(object, 0.0, 1));
    //idx.push_back(engine.getIdxFromAngle(object, 45.0));
    idx.push_back(engine.getIdxFromAngle(object, 90.0, 1));
    //idx.push_back(engine.getIdxFromAngle(object, 135.0));
    idx.push_back(engine.getIdxFromAngle(object, 180.0, 1));
    //idx.push_back(engine.getIdxFromAngle(object, 225.0));
    idx.push_back(engine.getIdxFromAngle(object, 270.0, 1));
    return object;
}

void test (string object_path) {
    Object object = common(object_path);
    cout << "Creation partial model" << endl;
    Object model = engine.objectFromObject (object, idx);
    cout << "Matching model to object" << endl;
    vector<int> number_matches;
    vector<float> percent_matches;
    vector<float> rotation_error;
    engine.match (model, object,
                    number_matches,
                    percent_matches,
                    rotation_error);
    engine.save("output", number_matches, percent_matches);

    cout << "Matches:" << endl;
    for (size_t i = 0; i < results.size(); ++i) {
        if (object.views_[i].keypoints_.size() != 0.0) {
            cout << results[i] << " ";
            cout << float(results[i])/float(object.views_[i].keypoints_.size()) * 100 << " ";
            cout << endl;
        }
        else
            cout << 0.0 << " ";
    }
    cout << endl;
    cout << "Rotation error:" << endl;
    for (size_t i = 0; i < rotation_error.size(); ++i)
        cout << rotation_error[i] << " ";
    cout << endl;

}
*/
vector<int> getIdxFromAngles (Object object, vector<int> angles) {
    vector<int> idx;
    for (size_t i = 0; i < angles.size(); ++i)
        idx.push_back(engine.getIdxFromAngle(object, angles[i], 1));
    return idx;
}

float computeObject (string object_path, vector<int> angles,
                    vector<Error> &errors) {
    //cout << "Processing " << object_path << endl;
    //cout << "Loading data and creating object" << endl;
    Object object = files.loadObject (object_path);
    //cout << "Sorting object" << endl;
    engine.sortViewByAngle(object);
    //cout << "Getting model angles" << endl;
    vector<int> idx = getIdxFromAngles(object, angles);
    //cout << "Creation partial model" << endl;
    Object model = engine.objectFromObject (object, idx);
    //cout << model.views_[0].descriptors_.rows << endl;
    //cout << "Matching model to object" << endl;
    engine.match (model, object, errors);
    //cout << model.views_[0].descriptors_.rows << endl;
    //cout << "Finished" << endl;
    return engine.getObjectSize(model);
}

int computeClass (string base_object_path, int num_objects, vector<int> angles,
                    vector<vector<Error> > &errors, int &objects_size) {
    errors.clear();
    objects_size = 0;

    int num_views = 120;
    errors.resize(num_objects);
    //for (size_t i = 0; i < num_objects; ++i)
        //errors[i].resize(num_views);

    for (size_t i = 0; i < num_objects; ++i) {
        std::stringstream ss;
        ss << base_object_path << (i + 1);
        string object_path = ss.str();

        vector<Error> tmp_error;
        objects_size += computeObject (object_path, angles, tmp_error);

        for (size_t n = 0; n < tmp_error.size(); ++n)
            errors[i].push_back(tmp_error[n]);
    }

    return num_objects;
}

void compute(string dataset_path, vector<string> classes, Feature ft, vector<int> angles, int idx) {
    cout << "Computing...";
    files.setFeatures(ft);

    int total_number_objects = 0;
    int mean_size = 0;
    vector<vector<Error> > error;
    vector<vector<Error> > tmp_error;
    for (size_t i = 0; i < classes.size(); ++i) {
        string class_path = dataset_path + classes[i] + "/";
        //cout << class_path << endl;
        string full_path = class_path + classes[i] + "_";
        //cout << full_path << endl;
        int num_objects = files.getNumFolders(class_path);
        //cout << num_objects << endl;
        int tmp_size = 0;
        total_number_objects += computeClass(full_path, num_objects, angles, tmp_error, tmp_size);
        error.insert( error.end(), tmp_error.begin(), tmp_error.end() );
        mean_size += tmp_size;
    }
    cout << "...done." << endl;
    //cout << "Computing mean" << endl;
    //vector<Error> mean = engine.getMean(tmp_error);
    //cout << error.size() << endl;
    vector<Error> mean = engine.getMean(error);
    mean_size /= total_number_objects;

    std::stringstream ss;
    ss << "results/" << classes[0] << "_feature_" << files.getFeatures() << "_modelsize_" << angles.size() << "_" << idx;
    //cout << ss.str() << endl;
    engine.save(ss.str(), mean);
    //cout << mean[0].time_ << " " << mean_size << endl;
    engine.saveTimeSize("results/time_size.csv", mean[0].time_, mean_size);
    cout << "Saved to " << ss.str() << endl;
}

void experiment_best_feature (string dataset_path, vector<string> classes) {
    vector<int> angles;
    angles.push_back(90);

    // ASIFT -> ... ?
    for (int ft = 1; ft < 6; ++ft)
    //for (int ft = 0; ft < 1; ++ft)
        compute(dataset_path, classes, static_cast<Feature>(ft), angles, static_cast<Feature>(ft));
}

void experiment_best_angles_features (string dataset_path, vector<string> classes) {
    // ASIFT -> FREAK
    for (int ft = 1; ft < 6; ++ft) {
        vector<int> angles(1);
        for (int angle = 0; angle < 360; angle += 9) {
            angles[0] = angle;
            compute(dataset_path, classes, static_cast<Feature>(ft), angles, angle);
        }
    }
}

void experiment_num_views (string dataset_path, vector<string> classes) {
    Feature ft = eSIFT;

    float step = 90.0;
    float factor = 2.0;
    while (step > 15.0) {
        vector<int> angles;
        int num_views = static_cast<int>(360.0/step);
        for (int angle = 0; angle < num_views; angle += step) {
            angles.push_back(angle);
        }

        compute(dataset_path, classes, ft, angles, step);

        step = step / factor;
    }
}

int main() {
    string dataset_path = "/home/gmanfred/devel/datasets/washington_rgbd/light-rgbd-dataset/";

    vector<string> classes;
    classes.push_back("food_jar");
    /*
    classes.push_back("cereal_box");
    classes.push_back("food_bag");
    classes.push_back("food_box");
    classes.push_back("food_can");
    classes.push_back("instant_noodles");
    classes.push_back("soda_can");
    classes.push_back("toothpaste");
    classes.push_back("water_bottle");
    */

    //experiment_best_feature (dataset_path, classes);
    experiment_best_angles_features(dataset_path, classes);
    //experiment_num_views (dataset_path, classes);

    /*
    string dataset_path = "/home/gmanfred/devel/datasets/washington_rgbd/rgbd-dataset/";
    vector<string> classes;
    //classes.push_back("cereal_box");
    classes.push_back("food_box");

    vector<int> angles;
    angles.push_back(0);
    //angles.push_back(45);
    //angles.push_back(90);
    //angles.push_back(135);
    //angles.push_back(180);
    //angles.push_back(225);
    //angles.push_back(270);
    //angles.push_back(315);

    Feature ft = eSIFT;

    compute(dataset_path, classes, ft, angles);
    */
    return 0;
}
