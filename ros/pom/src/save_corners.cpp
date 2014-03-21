#include <assert.h>

#include "SaveCorners.h"

// ./bin/save_corners /home/gmanfred/devel/datasets/rfia2014/coil/advil 0 90

int main (int argc, char** argv) {
    assert (argc == 3 && "Usage : save_corners object_dir start_number step (in degrees)");
    SaveCorners sc (argv[1]);
    sc.saveCornersObject (argv[1], atoi(argv[2]), atoi(argv[3]));
}
