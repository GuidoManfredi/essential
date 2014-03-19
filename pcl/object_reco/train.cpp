#include "Trainer.hpp"

int
main (int argc, char** argv)
{
	if(argc != 3)
	{
		cout << "Usage : main model trained" << endl;
		return (-1);
	}

	Trainer t;
	t.load_model(argv[1]);
	t.train_model();
	t.save_model(argv[2]);
	
	return (0);
}
