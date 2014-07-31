#include "FilesManager.h"

using namespace std;
using namespace boost::filesystem;
using namespace cv;

FilesManager::FilesManager () {}

void FilesManager::getPathsFromList(std::string list_path,
                                     std::vector<std::string> &paths) {
    paths.clear();
    vector<string> list;
    string line;
    ifstream myfile (list_path.c_str());
    if (myfile.is_open()) {
        string path;
        string label;
        while ( getline (myfile,line) ) {
            stringstream ss(line);
            ss >> path;
            paths.push_back(path);
        }
        myfile.close();
    }
    else cout << "Unable to open " << list_path << endl;

    return;
}


vector<Mat> FilesManager::getImages (string images_path) {
    vector<Mat> images;
    boost::filesystem::path bf_images_path(images_path);
    if ( is_regular_file(bf_images_path) ) {
        Mat image = getImage (bf_images_path.string());
        images.push_back (image);
    }
    else if ( is_directory(bf_images_path) ) {
        typedef vector<path> vec;
        vec v;
        copy(directory_iterator(bf_images_path), directory_iterator(), back_inserter(v));
        sort(v.begin(), v.end());

        for (vec::const_iterator it (v.begin()); it != v.end(); ++it)
        {
            Mat image = getImage (it->string());
            if (image.data) {
                images.push_back (image);
            }
        }
    }
    return images;
}

string FilesManager::getName (string file_or_dir_path) {
    boost::filesystem::path path(file_or_dir_path);
    return path.stem().string();
}
////////////////////////////////////////////////////////////////////////////////
//  PRIVATE METHODS
////////////////////////////////////////////////////////////////////////////////
Mat FilesManager::getImage (string image_path) {
    Mat image;
    boost::filesystem::path bf_images_path(image_path);
    if (extension(bf_images_path) == ".png"
        || extension(bf_images_path) == ".jpg") {
        image = imread (bf_images_path.string(), CV_LOAD_IMAGE_COLOR);
        if(! image.data )
            cout << "Could not open or find the image at " << bf_images_path.string() << endl;
    } else {
        cout << "Did not read " << bf_images_path.string() << endl;
    }
    return image;
}