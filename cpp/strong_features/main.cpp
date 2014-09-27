#include <iostream>

#include "FilesManager.h"
#include "Engine.h"

// TODO faire que compute traite correctement toutes les classes dans sa boucle "for"
using namespace std;
using namespace cv;

// cereal_box, food_bag, food_box, food_can, instant_noodles, soda_can, toothpast, water_bottle

// 570 for IR
// 535 for RGB
//Mat K =  (Mat_<double>(3,3) << 570, 0, 320, 0, 570, 240, 0, 0, 1);
Mat K =  (Mat_<double>(3,3) << 535, 0, 320, 0, 535, 240, 0, 0, 1);
Pipeline2D* pipe2d = new Pipeline2D(K);
FilesManager files(K, pipe2d);
Engine engine(pipe2d);

string feature2string (Feature ft) {
    string feature;
    switch (ft) {
        case 0: {
            feature = "ASIFT";
            break;
        }
        case 1: {
            feature = "SIFT";
            break;
        }
        case 2: {
            feature = "SURF";
            break;
        }
        case 3: {
            feature = "ORB";
            break;
        }
        case 4: {
            feature = "FREAK";
            break;
        }
        case 5: {
            feature = "BRISK";
            break;
        }
        case 6: {
            feature = "BRIEF";
            break;
        }
        default: {
            cout << "error: feature2string: invalide feature." << endl;
        }
    }
    return feature;
}

vector<int> getIdxFromAngles (Object object, vector<int> angles) {
    vector<int> idx;
    for (size_t i = 0; i < angles.size(); ++i)
        idx.push_back(engine.getIdxFromAngle(object, angles[i], 1));
    return idx;
}

void modelObject (string class_path, string object_name, string output_folder, Feature ft) {
    files.setFeatures(ft);

    string object_path = class_path + "/" + object_name;
    cout << object_path << endl;
    Object object = files.createObject (object_path);
    engine.sortViewByAngle(object);

    string output_path = output_folder + object_name + ".yaml";
    cout << output_path << endl;
    FileStorage fs(output_path, FileStorage::WRITE);
    fs << object_name << object;
    fs.release();
}

void modelObject (string class_path, string object_name, string output_folder) {
    string object_path = class_path + "/" + object_name;
    //cout << object_path << endl;
    Object object = files.createObject (object_path);
    engine.sortViewByAngle(object);

    Feature eft = static_cast<Feature>(object.features_type_);
    string feature = feature2string(eft);
    string output_path = output_folder + "/" + object_name + "_" + feature + ".yaml";
    cout << output_path << endl;
    FileStorage fs(output_path, FileStorage::WRITE);
    fs << object_name << object;
    fs.release();
}

void modelClass (string dataset_path, string class_name, string output_folder, Feature ft) {
    files.setFeatures(ft);

    string class_path = dataset_path + "/" + class_name;
    int num_objects = files.getNumFolders(class_path);
    for (size_t i = 0; i < num_objects; ++i) {
        std::stringstream ss; ss << class_name << "_" << (i + 1); string object_name = ss.str();
        modelObject (class_path, object_name, output_folder);
    }
}

void modelAll (string dataset_path, vector<string> class_names, string output_folder) {
    for (int i = 0; i < 7; ++i) {
        for (size_t n = 0; n < class_names.size(); ++n) {
            Feature ft = static_cast<Feature>(i);
            modelClass (dataset_path, class_names[n], output_folder, ft);
        }
    }
}

float computeObject (string objects_path, string object_name, Feature feature, vector<int> angles,
                    vector<Error> &errors) {
    string ft = feature2string(feature);
    string file_path = objects_path + "/" + object_name + "_" + ft + ".yaml";
    cout << "Loading " << file_path << endl;
    Object object;
    FileStorage fs(file_path, FileStorage::READ);
    //cout << objects_path + object_name << endl;
    fs[object_name] >> object;
    //cout << object.views_[0].keypoints_.size() << endl;
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

int computeClass (string dataset_path, string class_name, string objects_path, Feature feature, vector<int> angles,
                    vector<vector<Error> > &errors, float &models_size) {
    string class_path = dataset_path + "/" + class_name;
    //cout << class_path << endl;
    int num_objects = files.getNumFolders(class_path);
    //cout << num_objects << endl;
    models_size = 0;
    int num_views = 120;
    errors.resize(num_objects);

    for (size_t i = 0; i < num_objects; ++i) {
        std::stringstream ss; ss << class_name << "_" << (i + 1); string object_name = ss.str();
        //cout << object_name << endl;
        vector<Error> tmp_error;
        models_size += computeObject (objects_path, object_name, feature, angles, tmp_error);
        for (size_t n = 0; n < tmp_error.size(); ++n)
            errors[i].push_back(tmp_error[n]);
    }

    return num_objects;
}

void computeAll(string dataset_path, vector<string> class_names, string objects_path, vector<int> angles, Feature feature, int more) {
    cout << "Computing..." << endl;
    vector<Error> mean;
    float mean_size = 0.0;
    int number_objects = 0;
    for (size_t n = 0; n < class_names.size(); ++n) {
        vector<vector<Error> > tmp_errors;
        float tmp_model_size;
        number_objects = computeClass (dataset_path, class_names[n], objects_path, feature, angles,
                                       tmp_errors, tmp_model_size);
        mean = engine.getMean(tmp_errors);
        mean_size = tmp_model_size / number_objects;

        std::stringstream ss;
        ss << "results/" << class_names[n] << "_feature_" << feature << "_modelsize_" << angles.size() << "_" << more;
        string result_path = ss.str();
        engine.save(result_path, mean);
        engine.saveTimeSize(result_path + "_time_size.csv", mean[0].time_, mean_size);
    }
}

void experiment_best_feature (string dataset_path, vector<string> class_names, string objects_path) {
    vector<int> angles;
    angles.push_back(90);
    //angles.push_back(180);
    angles.push_back(270);

    // SIFT -> BRISK
    for (int ft = 0; ft < 7; ++ft)
    //for (int ft = 0; ft < 1; ++ft)
        computeAll(dataset_path, class_names, objects_path, angles, static_cast<Feature>(ft), 0);
}

void experiment_best_angles (string dataset_path, vector<string> class_names, string objects_path) {
    // ASIFT -> FREAK
    vector<int> angles(1);
    //for (int ft = 0; ft < 7; ++ft) {
    int ft = 5; {
        for (int angle = 0; angle < 360; angle += 9) {
            angles[0] = angle;
            cout << angle << endl;
            computeAll(dataset_path, class_names, objects_path, angles, static_cast<Feature>(ft), angle);
        }
    }
}

void experiment_num_views (string dataset_path, vector<string> class_names, string objects_path) {
    //Feature ft = eASIFT;
    Feature ft = eSIFT;

    float step = 90.0;
    float factor = 2.0;
    while (step > 10.0) {
        vector<int> angles;
        for (int angle = 0; angle < 360; angle += step)
            angles.push_back(angle);

        computeAll (dataset_path, class_names, objects_path, angles, ft, step);

        step = step / factor;
    }
}

int main() {
    //string dataset_path = "/home/gmanfred/devel/datasets/washington_rgbd/light-rgbd-dataset";
    string dataset_path = "/home/gmanfred/devel/datasets/washington_rgbd/rgbd-dataset";
    string objects_path = "results/objects";

    vector<string> classes;
    classes.push_back("cereal_box");
    classes.push_back("food_bag");
    classes.push_back("instant_noodles");
    classes.push_back("soda_can");
    classes.push_back("food_box"); // TODO TEST AND FIX THIS ONE
    classes.push_back("food_can");
    classes.push_back("food_jar");
    classes.push_back("water_bottle");

    //Feature ft = eSIFT;
    //modelObject(dataset_path + "cereal_box", "cereal_box_1", objects_path, ft);
    //modelClass(dataset_path, "cereal_box", objects_path, ft);
    //modelAll (dataset_path, classes, objects_path);

    /*
    vector<int> angles;
    angles.push_back(90);
    vector<vector<Error> > errors;
    float model_size;
    //computeObject(objects_path, "cereal_box_1", angles, errors);
    computeClass(dataset_path, "cereal_box", objects_path, angles, errors, model_size);
    for (size_t i = 0; i < errors[2].size(); ++i)
        cout << errors[2][i].P_ << endl;
    */

    //experiment_best_feature (dataset_path, classes, objects_path);
    //experiment_best_angles (dataset_path, classes, objects_path);
    experiment_num_views (dataset_path, classes, objects_path);

    return 0;
}
