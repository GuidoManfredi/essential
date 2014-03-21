#include <iostream>
#include <fstream>
#include <assert.h>
#include <boost/filesystem.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../pompod/POM.h"
#include "../pompod/Object.h"
#include "../pompod/Preprocessor.h"

using namespace std;
using namespace boost::filesystem;
using namespace cv;

// ./bin/create_object /home/gmanfred/devel/datasets/purfruit_no_synth

POM pom;
Preprocessor preprocessor;

void pomTrain (string train_dir, bool with_synthetic);
vector<Mat> getImages (string images_path);
Mat getImage (string image_path);
string getDirName (string train_dir);

int main(int argc, char **argv) {
    assert (argc == 3 && "Usage : create_model training_dir(.png/.jpg files) generate_synthetic_views");
    pomTrain (argv[1], atoi(argv[2]));
    return 0;
}

void pomTrain (string train_dir, bool with_synthetic) {
    cout << "Start Training" << endl;

    vector<Mat> train_images = getImages (train_dir);

	// OBJECT
	string object_name = getDirName (train_dir);
	string object_filename = object_name + ".yaml";
    Object object;
    object = pom.createObject (train_images, with_synthetic); // generate synthetic from train images
    cv::FileStorage fs (object_filename, FileStorage::WRITE);


    fs << object_name << object;
    fs.release();

    cout << "Training done" << endl;
}

Mat getImage (string image_path) {
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

vector<Mat> getImages (string images_path) {
    vector<Mat> images;
    boost::filesystem::path bf_images_path(images_path);
    if ( is_regular_file(bf_images_path) ) {
        Mat image = getImage (bf_images_path.string());
        Mat cropped = preprocessor.preprocess (image);
        images.push_back (cropped);
    }
    else if ( is_directory(bf_images_path) ) {
        directory_iterator end_itr;
        for ( directory_iterator itr(bf_images_path); itr != end_itr; ++itr ) {
            Mat image = getImage (itr->path().string());
            if (image.data) {
                Mat cropped = preprocessor.preprocess (image);
                //imshow ("Cropped", cropped);
                //waitKey(0);
                images.push_back (cropped);
            }
        }
    }
    return images;
}

string getDirName (string train_dir) {
    boost::filesystem::path bf_images_path(train_dir);
    return bf_images_path.stem().string();
}
