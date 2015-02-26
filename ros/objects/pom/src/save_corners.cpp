#include <assert.h>

#include "SaveCorners.h"

// rosrun pom save_corners /home/gmanfred/devel/datasets/my_objects/pom/lait
int main (int argc, char** argv) {
    assert (argc == 2 && "Usage : save_corners object_dir");
    SaveCorners sc (argv[1]);
    sc.saveCornersObject ();
}
