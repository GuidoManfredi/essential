#include "string_tools.h"

using namespace std;

std::string get_extension (std::string path)
{
	std::vector<std::string> splitted;
	boost::split (splitted, path, boost::algorithm::is_any_of("."));
	int n = splitted.size ();
	
	return splitted[n-1];
}

std::string path2name (std::string path)
{
	std::vector<std::string> splitted;
	boost::split (splitted, path, boost::algorithm::is_any_of("/"));
	int n = splitted.size ();
	
	return splitted[n-1];
}

std::string path2updir (std::string updir_path)
{
	string updir;

	std::vector<std::string> splitted;
	boost::split (splitted, updir_path, boost::algorithm::is_any_of("/"));
	
	for (size_t i=0; i<splitted.size () - 1; ++i)
			updir += splitted[i] + "/";
	
	return updir;
}

std::string path2class (std::string class_path)
{
	std::string classs, instance, pose;
	split_path (class_path, classs, instance, pose);
	
	return classs;
}

std::string path2instance (std::string class_path)
{
	std::string classs, instance, pose;
	split_path (class_path, classs, instance, pose);
	
	return instance;
}

std::string path2pose (std::string class_path)
{
	std::string classs, instance, pose;
	split_path (class_path, classs, instance, pose);
	
	return pose;
}

void split_path (std::string full_path,
									std::string &class_path, std::string &instance_path, std::string &pose_path)
{
	std::vector<std::string> splitted;
	boost::split (splitted, full_path, boost::algorithm::is_any_of("/"));
	
	int n = splitted.size ();
	
	if ( n >= 3)
	{
		pose_path = splitted[n-1];
		instance_path = splitted[n-2];
		class_path = splitted[n-3];
	}
}
