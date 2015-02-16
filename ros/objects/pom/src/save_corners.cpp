#include <assert.h>

#include "SaveCorners.h"

// rosrun pom save_corners /home/gmanfred/devel/datasets/rfia2014/coil/advil
int main (int argc, char** argv) {
    assert (argc == 2 && "Usage : save_corners object_dir");
    SaveCorners sc (argv[1]);
    sc.saveCornersObject ();
}
