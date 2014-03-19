#ifndef DEF_FILE_TOOLS
#define DEF_FILE_TOOLS

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

namespace bf = boost::filesystem;

void createDir (std::string & dir);

void removeDir (std::string & dir);

void
getFileByNameAndExt (bf::path & dir, std::vector<std::string> & filenames, std::string basename, std::string ext);

void getDirRec (bf::path & dir, std::string & rel_path_so_far, std::vector<std::string> & relative_paths);

bool isleafDirectory (bf::path & path);

#endif
