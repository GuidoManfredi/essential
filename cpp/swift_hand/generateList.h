#pragma once

#include <fstream>

void generateMarcelTrainLists();
void generateMarcelTestLists();
void generateList(std::string base_folder, std::string base_name, int start_idx, int end_idx, std::string ext, std::string label);
