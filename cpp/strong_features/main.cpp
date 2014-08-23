#include "FilesManager.h"
#include "Engine.h"

using namespace std;

void testAll (string object_path) {
    FilesManager files;
    Engine engine;

    cout << "Loading data and creating object" << endl;
    Object object = files.loadObject (object_path);
    vector<int> idx;
    idx.push_back(0);
    //idx.push_back(20);
    //idx.push_back(40);
    //idx.push_back(60);
    cout << "Sorting object" << endl;
    engine.sortViewByAngle(object);
    cout << "Creation partial model" << endl;
    Model model = engine.modelFromObject (object, idx);
    cout << "Matching model to object" << endl;
    vector<int> results = engine.match (model, object);
    cout << "Results:" << endl;
    for (size_t i = 0; i < results.size(); ++i) {
        cout << results[i] << " ";
    }
    cout << endl;
}

int main() {
    //testAll ("/home/gmanfred/devel/datasets/washington_rgbd/rgbd-dataset/cereal_box/cereal_box_1");
    //testAll ("/home/gmanfred/devel/datasets/washington_rgbd/rgbd-dataset/food_can/food_can_1");
    //testAll ("/home/gmanfred/devel/datasets/washington_rgbd/rgbd-dataset/food_can/food_can_2");
    testAll ("/home/gmanfred/devel/datasets/washington_rgbd/rgbd-dataset/food_box/food_box_2");
    return 0;
}
