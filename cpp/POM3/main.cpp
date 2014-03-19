#include <fstream>

#include <boost/filesystem.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "POM.h"
#include "POD.h"

/** TODO
 *  Sauvegarde et lecture d'objet/face dans un fichier par POM et POD. (Attendre version finale,
                                                                        sinon trop de conneries
                                                                        debug dans les structures
                                                                        de données).
 *  Integration dans ROS
 *  Ajouter localisation (sauvegarder corners dans face et objets et les faire
                          remonter dans synthesizer).
 *  Permettre differentes methodes d'obtenir des images synthetiques (affine ou
    projective, et plusieurs types de affine).
**/

using namespace std;
using namespace boost::filesystem;
using namespace cv;

Pipeline2D pipeline2d;
Face face;

void test_pompod ();
void test_vocabulary ();
void test_synthesizer ();
void test_image_getters (char *file, char *dir);
void process_query (Mat query_image);

vector<string> readFile (string image_path) {
    ifstream myfile (image_path.c_str());
    cout << "Opening " << image_path << endl;

    vector<string> files;
    if (myfile.is_open()) {
        string line;
        while ( getline (myfile,line) ) {
            files.push_back (line);
        }
        myfile.close();
    } else {
        cout << "Unable to open file";
    }

    return files;
}

Mat getImage (string image_path) {
    Mat image;
    boost::filesystem::path bf_images_path(image_path);
    if (extension(bf_images_path) == ".jpg"
        || extension(bf_images_path) == ".png") {
        image = imread (bf_images_path.string(), CV_LOAD_IMAGE_GRAYSCALE);
        if(! image.data )
            cout <<  "Could not open or find the image at "
                 << bf_images_path.string() << std::endl ;
    } else {
        cout << "Could not read " << bf_images_path.string() << "." << endl;
    }
    return image;
}

vector<Mat> getImages (string images_path) {
    vector<Mat> images;
    boost::filesystem::path bf_images_path(images_path);
    if ( is_regular_file(bf_images_path) ) {
        Mat image = getImage (bf_images_path.string());
        images.push_back (image);
    }
    else if ( is_directory(bf_images_path) ) {
        directory_iterator end_itr;
        for ( directory_iterator itr(bf_images_path); itr != end_itr; ++itr ) {
            Mat image = getImage (itr->path().string());
            images.push_back (image);
        }
    }
    return images;
}

int main()
{
    test_pompod ();
    //test_vocabulary ();
    //test_synthesizer ();
    //test_image_getters ("/home/gmanfred/Desktop/warp_0.jpg", "/home/gmanfred/devel/datasets/Caltech101_color/accordion");
    return 0;
}

void test_image_getters (char *file, char *dir)  {
    Mat image = getImage (file);
    imshow ("Test", image);
    waitKey(0);

    vector<Mat> images = getImages (dir);
    for ( size_t i = 0; i < images.size(); ++i ) {
        imshow ("Test", images[i]);
        waitKey(100);
    }

    images = getImages (file);
    for ( size_t i = 0; i < images.size(); ++i ) {
        imshow ("Test", images[i]);
        waitKey(0);
    }
}

void test_pompod () {
    //bool synthetic = true;
    bool synthetic = false;
    if ( synthetic ) {
        string training ("/home/gmanfred/Desktop/kango.jpg");
        Mat train_image = imread (training, CV_LOAD_IMAGE_GRAYSCALE);
        POM pom;
        face = pom.createFaceWithSynthetic (train_image);
    } else {
        string training ("/home/gmanfred/devel/datasets/kango/");
        vector<Mat> train_images = getImages (training);
        POM pom;
        face = pom.createFace (train_images);
    }

    POD pod;
    pod.addFace (face);

    VideoCapture capture;
    capture.open(0);
    if (!capture.isOpened())
        cout << "capture device failed to open!" << endl;

    Mat image, gray;
    while (1) {
        capture >> image;
        pod.process (image);
    }
}

void test_vocabulary () {
    // Training
    POM engine;
    vector<string> filespath = readFile ("/home/gmanfred/devel/these/projects/POM3/");
    vector<Face> faces;
    for (size_t i = 0; i < filespath.size(); ++i ) {
        vector<Mat> images = getImages (filespath[i]);
        faces.push_back (engine.createFace (images));
    }
    vector<Mat> vocabularies;
    for (size_t i = 0; i < faces.size(); ++i) {
        vocabularies.push_back (faces[i].vocabulary_);
    }

    // Query
    Mat image = imread ("/home/gmanfred/devel/datasets/Caltech101_color/accordion/image_0001.jpg", CV_LOAD_IMAGE_GRAYSCALE);

    Pipeline2D pipe;
    pipe.setVocabulary(vocabularies[0]);
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors, BoW;
    pipe.computeBoW (image, keypoints, descriptors, BoW);

    std::vector<cv::DMatch> matches;
    /*
    pipe.matchBoW (BoW, faces[0].bow_descriptors_,
                   matches);
                   */
    /*
    vector<DMatch> matches;
    pipe.matchVocabulary (query_vocabulary, vocabularies, matches);
    */
}

void test_synthesizer () {
    char *path = "/home/gmanfred/Desktop/warp_0.jpg";
    Mat image = imread (path, CV_LOAD_IMAGE_GRAYSCALE);
    SyntheticViewGenerator generator;
    std::vector<Mat> views, affines;
    std::vector<cv::Point2f> corners = generator.getCorners (image);
    generator.generateViews(image, corners, views, affines);

    cout << "Generated " << views.size() << " views." << endl;
    namedWindow ("Views", CV_WINDOW_AUTOSIZE);
    for (size_t i = 0; i < views.size(); ++i) {
        imshow ("Views", views[i]);
        waitKey (100);
    }
}
