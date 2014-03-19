#include "OBBComputer.h"
#include "Trainer.h"

using namespace std;

int main (int argc, char** argv)
{
	if (argc != 2)
		cout << "Usage: exp dataset_path" << endl;

	// Computing LAB and XYZ bounding boxes of all poses
	OBBComputer alpha("alpha");
	alpha.process_root(argv[1]);
	
	cout << "Completed." << endl;
	cout << " Results saved in " << alpha.get_xyz_name() << " and " << alpha.get_lab_name() << endl;
	return 0;
}
