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
vector<int> idx;

Object common (string object_path) {
    cout << "Loading data and creating object" << endl;
    Object object = files.loadObject (object_path);
    cout << "Sorting object" << endl;
    engine.sortViewByAngle(object);

    idx.push_back(engine.getIdxFromAngle(object, 0.0, 1));
    //idx.push_back(engine.getIdxFromAngle(object, 45.0));
    idx.push_back(engine.getIdxFromAngle(object, 90.0, 1));
    //idx.push_back(engine.getIdxFromAngle(object, 135.0));
    idx.push_back(engine.getIdxFromAngle(object, 200.0, 1));
    //idx.push_back(engine.getIdxFromAngle(object, 225.0));
    idx.push_back(engine.getIdxFromAngle(object, 300.0, 1));
    return object;
}

void test (string object_path) {
    Object object = common(object_path);
    cout << "Creation partial model" << endl;
    Object model = engine.objectFromObject (object, idx);
    cout << "Matching model to object" << endl;
    vector<float> rotation_error;
    vector<int> results = engine.match (model, object,
                                        rotation_error);
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

int main() {
    files.setFeatures(eSIFT);
    //files.setFeatures(eASIFT);
    test ("/home/gmanfred/devel/datasets/washington_rgbd/rgbd-dataset/cereal_box/cereal_box_2");
    //test ("/home/gmanfred/devel/datasets/washington_rgbd/rgbd-dataset/food_can/food_can_2");
    //test ("/home/gmanfred/devel/datasets/washington_rgbd/rgbd-dataset/food_can/food_can_2");
    //test ("/home/gmanfred/devel/datasets/washington_rgbd/rgbd-dataset/food_box/food_box_2");
    //test ("/home/gmanfred/devel/datasets/washington_rgbd/rgbd-dataset/test_object");
    //test ("/home/gmanfred/devel/datasets/washington_rgbd/rgbd-dataset/test_object2");
    return 0;
}
