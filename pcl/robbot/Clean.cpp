#include "Clean.h"

void Clean::clean (std::string dir)
{
	rec_erase_processed (dir);
}

void Clean::rec_erase_processed (std::string dir)
{
	if (bf::is_directory (dir))
  {
  	bf::directory_iterator end_itr;
    for (bf::directory_iterator i (dir); i != end_itr; ++i)
    {
    	//cout << "Processing " << i->path().string() << endl;
			if ( bf::is_directory (i->path().string()) )
			{    	
				rec_erase_processed(i->path().string());
			}
			else if ( get_extension (i->path().string()).compare ("txt") == 0 )
			{
				cout << "Removing " << i->path().string() << endl;
				bf::remove (i->path().string());
			}
    }
  }
}
