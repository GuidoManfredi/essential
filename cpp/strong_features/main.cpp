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
                    vector<Error> &errors) {
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
    engine.match (model, object, errors);
    cout << "Finished" << endl;
}

void computeClass (string base_object_path, int max, vector<int> angles,
                    vector<vector<Error> > &errors) {
    errors.resize(max);
    for (size_t i = 0; i < max; ++i)
        errors[i].resize(120);

    for (size_t i = 1; i <= max; ++i) {
        std::stringstream ss;
        ss << base_object_path << i;
        string object_path = ss.str();

        vector<Error> tmp_error;
        computeObject (object_path, angles, tmp_error);
        cout << tmp_error.size() << endl;
        for (size_t n = 0; n < 120; ++n) {
            //cout << n << " " << i << endl;
            errors[i-1][n] = tmp_error[n];
        }
    }
}

void experiment(string dataset_path, vector<string> classes, Feature ft, vector<int> angles) {
    files.setFeatures(ft);

    vector<vector<Error> > tmp_error;
    for (size_t i = 0; i < classes.size(); ++i) {
        string class_path = dataset_path + classes[i] + "/";
        //cout << class_path << endl;
        string full_path = class_path + classes[i] + "_";
        //cout << full_path << endl;
        int num_objects = files.getNumFolders(class_path);
        //int num_objects = 1;
        //cout << num_objects << endl;
        computeClass(full_path, num_objects, angles, tmp_error);
    }

    //cout << "Computing mean" << endl;
    vector<Error> mean = engine.getMean(tmp_error);

    std::stringstream ss;
    ss << classes[0] << "_feature_" << files.getFeatures() << "_modelsize_" << angles.size();
    //cout << ss.str() << endl;
    engine.save(ss.str(), mean);
}

int main() {
    string dataset_path = "/home/gmanfred/devel/datasets/washington_rgbd/rgbd-dataset/";
    vector<string> classes;
    classes.push_back("cereal_box");

    vector<int> angles;
    angles.push_back(0);
    angles.push_back(45);
    angles.push_back(90);
    angles.push_back(135);
    angles.push_back(180);
    angles.push_back(225);
    angles.push_back(270);
    angles.push_back(315);

    Feature ft = eSIFT;

    experiment(dataset_path, classes, ft, angles);
    return 0;
}
