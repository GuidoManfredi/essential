#include <ros/ros.h>
#include "../pom/POM.h"
#include "../pom/FilesManager.h"

using namespace std;
using namespace cv;

Object model (string input_dir, string output_dir);

POM modeler;

// rosrun pom create_object /home/gmanfred/devel/datasets/my_objects/purfruit /home/gmanfred/devel/datasets/my_objects/models
int main (int argc, char** argv) {
    assert(argc == 3 && "Usage: craete_object images_dir output_dir");
    Object object = model (argv[1], argv[2]);
    return 0;
}

Object model (string input_dir, string output_dir) {
    ROS_INFO("Loading images...");
    FilesManager fm;
    vector<Mat> images;
    vector<int> faces;
    fm.getImagesAndFaces (input_dir, images, faces);
    ROS_INFO("Reading corners file");
    vector<vector<Point2f> > corners = fm.getCorners (input_dir + "/corners.txt");
    ROS_INFO("Reading dimensions file");
    Point3f dimensions = fm.getDimensions (input_dir + "/dimensions.txt");
    ROS_INFO("Loaded files from %s", input_dir.c_str());

    ROS_INFO("Modeling...");
    Object object = modeler.model (images, faces, corners, dimensions);

    ROS_INFO("Model create. Saving object...");
    string object_name = fm.getDirName (input_dir);
    string object_path = output_dir + "/" + object_name + ".yaml";
    ROS_INFO("Saving to %s", object_path.c_str());
    FileStorage fs(object_path, FileStorage::WRITE);
    fs << object_name << object;
    fs.release();

    return object;
}
