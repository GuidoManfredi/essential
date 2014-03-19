#ifndef DEF_ROBBOT_CLEAN
#define DEF_ROBBOT_CLEAN

#include <boost/filesystem.hpp>
#include "string_tools.h"

using namespace std;
namespace bf = boost::filesystem;

class Clean
{
public:
	void clean (std::string dir);
protected:
	void rec_erase_processed (std::string dir);
};

#endif
