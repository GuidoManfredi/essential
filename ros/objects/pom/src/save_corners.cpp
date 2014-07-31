#include <assert.h>

#include "SaveCorners.h"

// ./bin/save_corners /home/gmanfred/devel/datasets/rfia2014/coil/advil 0 90

int main (int argc, char** argv) {
    assert (argc == 2 && "Usage : save_corners object_dir");
    SaveCorners sc (argv[1]);
    sc.saveCornersObject ();
}
