#include "../pom/POM.h"
#include "../pom/FilesManager.h"

using namespace std;
using namespace cv;

Object model (string dir);

POM modeler;

// ./bin/create_object /home/gmanfred/devel/datasets/my_objects/purfruit
int main (int argc, char** argv) {
    Object object = model (argv[1]);
    return 0;
}

Object model (string dir) {
    cout << "Loading images" << endl;
    FilesManager fm;
    vector<Mat> images = fm.getImages (dir);
    vector<vector<Point2f> > corners = fm.getCorners (dir + "/corners.txt");
    Point3f dimensions = fm.getDimensions (dir + "/dimensions.txt");
    cout << "Loaded files in " << dir << endl;

    cout << "Modeling" << endl;
    Object object = modeler.model (images, corners, dimensions);

    cout << "Saving object" << endl;
    string object_name = fm.getDirName (dir);
    string object_filename = object_name + ".yaml";
    cout << "Saving to " << object_filename << endl;
    FileStorage fs(object_filename, FileStorage::WRITE);
    fs << object_name << object;
    fs.release();

    return object;
}
