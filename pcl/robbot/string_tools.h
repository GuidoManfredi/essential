#ifndef DEF_ROBBOT_STRINGTOOLS
#define DEF_ROBBOT_STRINGTOOLS

#include <vector>
#include <boost/filesystem.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/algorithm/string.hpp>

std::string get_extension (std::string path);

std::string path2name (std::string path);
std::string path2updir (std::string updir_path);									
std::string path2class (std::string pose_path);
std::string path2instance (std::string pose_path);
std::string path2pose (std::string pose_path);									
void split_path (std::string full_path,
									std::string &class_path, std::string &instance_path, std::string &pose_path);							

#endif
