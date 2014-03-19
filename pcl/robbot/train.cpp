#include "Trainer.h"
#include "TrainerColor.h"

using namespace std;

int main (int argc, char** argv)
{
	if (argc != 4)
	{
		cout << "Usage: exp dataset_path results_filename1 results_filename1" << endl;
		return -1;
	}

	// Getting results from last step and computing mean and covariance
	Trainer trainer_xyz("xyz", argv[2]);
	TrainerColor trainer_lab("lab", argv[3]);
	trainer_xyz.process_root (argv[1], 0.0, 0.75);
	trainer_lab.process_root (argv[1], 0.0, 0.75);
	
	return 0;
}
