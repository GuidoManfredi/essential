#include "generateList.h"

using namespace std;

void generateList(string base_folder, string base_name, int start_idx, int end_idx, string ext, string label) {
    ofstream file;
    string list_path = base_folder + "/list.txt";
    file.open(list_path.c_str());
    for (int i = start_idx; i <= end_idx; ++i) {
        if (i < 10)
            file << base_folder << base_name << "000" << i << ext << " " << label << endl;
        else if (i < 100)
            file << base_folder << base_name << "00" << i << ext << " " << label << endl;
        else if (i < 1000)
            file << base_folder << base_name << "0" << i << ext << " " << label << endl;
        else
            file << base_folder << base_name << i << ext << " " << label << endl;
    }
    file.close();
}
