#include "Clean.h"

int main(int argc, char** argv)
{
	if (argc != 2)
	{
		cout << "Usage : clean dir_to_clean" << endl;
		return -1;
	}

	Clean cl;
	cl.clean (argv[1]);
	
	return 0;
}
