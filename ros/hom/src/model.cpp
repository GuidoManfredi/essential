#include "HOM.h"

int main (int argc, char** argv) {
    assert(argc == 1 && "Usage: model image_path);

    HOM modeler;
    Face face = modeler.createFace(argv[1]);
    modeler.save(face);
    
    return 0;
}


