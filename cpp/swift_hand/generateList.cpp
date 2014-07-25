#include "generateList.h"

using namespace std;
using namespace boost::filesystem;

void generateListFromFolder(string folder_path, string list_name, string ext, bool append, int label) {
    ofstream file;
    //string dir_name = getDirName(folder_path);
    //string file_name = dir_name + "_list.txt";
    //file.open(file_name.c_str());
    if (append)
        file.open(list_name.c_str(), fstream::app);
    else
        file.open(list_name.c_str());

    vector<string> path;
    boost::filesystem::path bf_images_path(folder_path);
    if ( is_directory(bf_images_path) ) {
        typedef vector<boost::filesystem::path> vec;
        vec v;
        copy(directory_iterator(bf_images_path), directory_iterator(), back_inserter(v));
        sort(v.begin(), v.end());

        for (vec::const_iterator it (v.begin()); it != v.end(); ++it)
            if (extension(*it) == ext)
                file << it->string() << " " << label << endl;
    }
    else
        cout << folder_path << " is not a directory." << endl;
}

string getDirName (string train_dir) {
    boost::filesystem::path bf_images_path(train_dir);
    return bf_images_path.stem().string();
}
