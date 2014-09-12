#include <iostream>

#include "FilesManager.h"
#include "Engine.h"

// TODO Faire en sorte que le model contiene des images qui matchent sont pas dans angles purrisµ.
// TODO Save the result to a file.
using namespace std;
using namespace cv;

// calculator, cell_phone, cereal_box, food_bag, food_box, food_can, food_jar,
//  instant_noodles, keyboard, kleenex, notebook, soda_can, toothpaste, water_bottle

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

void computeObject (string object_path, vector<int> angles,
                    vector<int> &number_matches,
                    vector<float> &percent_matches,
                    vector<float> &rotation_error) {
    cout << "Processing " << object_path << endl;
    cout << "Loading data and creating object" << endl;
    Object object = files.loadObject (object_path);
    cout << "Sorting object" << endl;
    engine.sortViewByAngle(object);
    cout << "Getting model angles" << endl;
    vector<int> idx = getIdxFromAngles(object, angles);
    cout << "Creation partial model" << endl;
    Object model = engine.objectFromObject (object, idx);
    cout << "Matching model to object" << endl;
    engine.match (model, object,
                    number_matches,
                    percent_matches,
                    rotation_error);
    cout << "Finished" << endl;
}

void computeClass (string base_object_path, int max, vector<int> angles) {
    vector<int> number_matches, mean_number_matches;
    vector<float> percent_matches, rotation_error, mean_percent_matches, mean_rotation_error;

    // 120 = 360 / 9 * 3
    mean_number_matches.resize(120);
    mean_percent_matches.resize(120);
    mean_rotation_error.resize(120);

    char tmp[21];
    for (size_t i = 1; i <= max; ++i) {
        std::stringstream ss;
        ss << base_object_path << i;
        string object_path = ss.str();
        computeObject (object_path, angles,
                       number_matches, percent_matches, rotation_error);
        for (size_t n = 0; n < 120; ++n) {
            mean_number_matches[n] += number_matches[n];
            mean_percent_matches[n] += percent_matches[n];
            mean_rotation_error[n] += rotation_error[n];
        }
    }

    for (size_t n = 0; n < 120; ++n) {
        mean_number_matches[n] /= 120;
        mean_percent_matches[n] /= 120;
        mean_rotation_error[n] /= 120;
    }

    std::stringstream ss;
    ss << base_object_path << "feature_" << files.getFeatures() << "_modelsize_" << angles.size();
    engine.save(ss.str(), mean_number_matches, mean_percent_matches);
}

void experiment() {
    //files.setFeatures(eSURF);
    files.setFeatures(eSIFT);
    //files.setFeatures(eASIFT);

    vector<int> idx;
    idx.push_back(0.0);
    idx.push_back(90.0);
    idx.push_back(180.0);
    idx.push_back(270.0);

    computeClass("/home/gmanfred/devel/datasets/washington_rgbd/rgbd-dataset/cereal_box/cereal_box_", 5, idx);
}

int main() {
    //files.setFeatures(eASIFT);
    //test ("/home/gmanfred/devel/datasets/washington_rgbd/rgbd-dataset/cereal_box/cereal_box_5");
    //test ("/home/gmanfred/devel/datasets/washington_rgbd/rgbd-dataset/food_can/food_can_6");
    //test ("/home/gmanfred/devel/datasets/washington_rgbd/rgbd-dataset/food_can/food_can_2");
    //test ("/home/gmanfred/devel/datasets/washington_rgbd/rgbd-dataset/food_box/food_box_2");
    //test ("/home/gmanfred/devel/datasets/washington_rgbd/rgbd-dataset/test_object");
    //test ("/home/gmanfred/devel/datasets/washington_rgbd/rgbd-dataset/test_object2");
    experiment();
    return 0;
}
