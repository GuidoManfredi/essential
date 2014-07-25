#pragma once

#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

void generateListFromFolder(std::string folder_path, std::string list_name,
                            std::string extension, bool append, int label);

std::string getDirName (std::string train_dir);
