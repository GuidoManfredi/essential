#ifndef DEF_CAROD_FILE_TOOLS
#define DEF_CAROD_FILE_TOOLS

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

namespace bf = boost::filesystem;

void createDir (std::string & dir);

void removeDir (std::string & dir);

void
getFileByNameAndExt (bf::path dir, std::vector<std::string> & filenames, std::string basename, std::string ext);

bool isleafDirectory (bf::path & path);

bool get_subdirectories (std::string dir, std::vector<std::string> &subdirs);

#endif // DEF_CAROD_FILE_TOOLS

