#include <ros/ros.h>
#include "../pom/POM.h"
#include "../pom/FilesManager.h"

using namespace std;
using namespace cv;

Object model (SHAPE shape, string input_dir, string output_dir);

POM modeler;

// rosrun pom create_object 0 /home/gmanfred/devel/datasets/my_objects/pom/purfruit /home/gmanfred/devel/datasets/my_objects/pom/models
// pcd_viewer model.pcd (to visualize the pointcloud in color press shift + 5)
int main (int argc, char** argv) {
    assert(argc == 4 && "Usage: create_object shape images_dir output_dir");
    SHAPE shape = PAVE;
    switch (atoi(argv[1])){
        case 0:
            shape = PAVE;
        break;
        case 1:
            shape = CYL;
        break;
        default:
            ROS_INFO("Shape not recognised: %d", shape);
    }
    Object object = model (shape, argv[2], argv[3]);
    return 0;
}

Object model (SHAPE shape, string input_dir, string output_dir) {
    ROS_INFO("Loading images...");
    FilesManager fm;
    vector<Mat> images;
    vector<int> faces;
    fm.getImagesAndFaces (input_dir, images, faces);
    ROS_INFO("... loaded %d images.", images.size());
    ROS_INFO("Reading corners file");
    vector<vector<Point2f> > corners = fm.getCorners (input_dir + "/corners.txt");
    ROS_INFO("Reading dimensions file");
    Point3f dimensions = fm.getDimensions (input_dir + "/dimensions.txt");
    ROS_INFO("Loaded files from %s", input_dir.c_str());

    ROS_INFO("Modeling...");
    Object object = modeler.model (shape, images, faces, corners, dimensions);

    ROS_INFO("Model create. Saving object...");
    string object_name = fm.getDirName (input_dir);
    string object_path = output_dir + "/" + object_name + ".yaml";
    ROS_INFO("Saving to %s", object_path.c_str());
    FileStorage fs(object_path, FileStorage::WRITE);
    fs << object_name << object;
    fs.release();

    return object;
}
