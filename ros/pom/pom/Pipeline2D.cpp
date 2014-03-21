#include <cassert>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include "Pipeline2D.h"

using namespace std;
using namespace cv;

Pipeline2D::Pipeline2D() {
    char * argv[] ={ "-fo", "-1", "-v", "0"};
    sift_.ParseParam(4, argv);
    //char * argv[] = {"-fo", "0", "-v", "0", "-s", "2"};
    //sift_.ParseParam (6, argv);
    int support = sift_.CreateContextGL();
    if(support != SiftGPU::SIFTGPU_FULL_SUPPORTED) {
        cout << "SiftGPU not supported" << endl;
    };

    minNumberMatchesAllowed_                  = 8;
}

void Pipeline2D::getGray(const cv::Mat& image, cv::Mat& gray)
{
    assert(!image.empty());
    if (image.channels()  == 3)
        cvtColor(image, gray, CV_BGR2GRAY);
    else if (image.channels() == 4)
        cvtColor(image, gray, CV_BGRA2GRAY);
    else if (image.channels() == 1)
        gray = image;
}

vector<Point2f> Pipeline2D::getCorners(Mat image) {
    vector<Point2f> corners (4);
    corners[0] = Point2f (0, 0);
    corners[1] = Point2f (image.cols, 0);
    corners[2] = Point2f (image.cols, image.rows);
    corners[3] = Point2f (0, image.rows);
    return corners;
}
////////////////////////////////////////////////////////////////////////////////
// Features Part
////////////////////////////////////////////////////////////////////////////////
bool Pipeline2D::detectFeatures(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints)
{
    assert(!image.empty());
    assert(image.channels() == 1);
    keypoints.clear();

    int width = image.cols;
    int height = image.rows;
    unsigned char *data = image.data;
    //sift.RunSIFT (width, height, data, GL_RGBA, GL_UNSIGNED_BYTE);
    sift_.RunSIFT (width, height, data, GL_LUMINANCE, GL_UNSIGNED_BYTE);

    int num_keypoints = sift_.GetFeatureNum();

    vector<SiftGPU::SiftKeypoint> keys(num_keypoints);
    sift_.GetFeatureVector(&keys[0], NULL);
    // converting to opencv
    for ( int i = 0; i < num_keypoints; ++i ) {
        KeyPoint kpt = GPUkpt2kpt (keys[i]);
        keypoints.push_back (kpt);
    }
    if ( num_keypoints == 0 )
        return false;
    return true;
}

bool Pipeline2D::describeFeatures(const cv::Mat image, std::vector<cv::KeyPoint> keypoints, cv::Mat& descriptors)
{
    assert(!image.empty());
    assert(image.channels() == 1);

    int num_keypoints = sift_.GetFeatureNum();
    if ( num_keypoints != 0 ) {
	    vector<float> gpu_descriptors(128*num_keypoints);
	    sift_.GetFeatureVector(NULL, &gpu_descriptors[0]);
	    // converting to opencv
	    descriptors = GPUdesc2desc (gpu_descriptors);
    	return true;
    }

    return false;
}

bool Pipeline2D::describeFeatures(const cv::Mat image, std::vector<int> keypoints_index, cv::Mat& descriptors)
{
    assert(!image.empty());
    assert(image.channels() == 1);

    int num_keypoints = sift_.GetFeatureNum();
    if ( num_keypoints != 0 ) {
	    vector<float> gpu_descriptors(128*num_keypoints);
	    sift_.GetFeatureVector(NULL, &gpu_descriptors[0]);
	    // converting to opencv
	    descriptors = GPUdesc2desc (gpu_descriptors, keypoints_index);
    	return true;
    }

    return false;
}

bool Pipeline2D::extractFeatures(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors)
{
    detectFeatures(image, keypoints);
    describeFeatures(image, keypoints, descriptors);
    return true;
}

bool Pipeline2D::match (const cv::Mat &desc1, const cv::Mat &desc2,
                        std::vector<cv::DMatch>& matches) {
    matchNoMinimum (desc1, desc2, matches);
    if ( matches.size() > minNumberMatchesAllowed_ )
        return true;

    return false;
}

void Pipeline2D::matchNoMinimum (const cv::Mat &desc1, const cv::Mat &desc2,
                           std::vector<cv::DMatch>& matches) {
    assert (desc1.data && desc2.data);
    matches.clear();
    SiftMatchGPU matcher_;
    matcher_.SetMaxSift (4096);
    if(matcher_.VerifyContextGL() == 0) return;
    vector<float> des1 = desc2GPUdesc (desc1);
    vector<float> des2 = desc2GPUdesc (desc2);

    matcher_.SetDescriptors(0, desc1.rows, &des1[0]);
    matcher_.SetDescriptors(1, desc2.rows, &des2[0]);
    //Match and read back result to input buffer
    int match_buf[4096][2];

    int nmatch = matcher_.GetSiftMatch(4096, match_buf);
    //cout << nmatch << endl;
    // convert to opencv
    for ( size_t i = 0; i < nmatch; ++i) {
        DMatch m;
        m.queryIdx = match_buf[i][0];
        m.trainIdx = match_buf[i][1];
        matches.push_back (m);
    }
    //cout << "Descs " << des1.size()/128 << " " << des2.size()/128 << endl;
}

Mat Pipeline2D::getDescriptorsFromIndices (Mat train_descriptors,
                                           vector<int> train_descriptors_indices) {
    Mat result;
    for ( size_t i = 0; i < train_descriptors_indices.size(); ++i) {
        result.push_back (train_descriptors.row(i));
    }
    return result;
}

KeyPoint Pipeline2D::GPUkpt2kpt (SiftGPU::SiftKeypoint key) {
    KeyPoint kpt;
    kpt.pt.x = key.x;
    kpt.pt.y = key.y;
    kpt.octave = key.s;
    kpt.angle = key.o;
    return kpt;
}

Mat Pipeline2D::GPUdesc2desc (vector<float> descriptors) {
    int num_descriptors = descriptors.size()/128;
    Mat desc (num_descriptors, 128, CV_32F); // 128 columns
    for ( size_t i = 0; i < num_descriptors; ++i ) {
        for ( size_t j = 0; j < 128; ++j) {
            desc.at<float> (i, j) = descriptors[j+i*128];
        }
    }
    return desc;
}

Mat Pipeline2D::GPUdesc2desc (vector<float> descriptors, vector<int> kpts_index) {
    int num_descriptors = kpts_index.size();
    Mat desc (num_descriptors, 128, CV_32F); // 128 columns
    for ( size_t i = 0; i < num_descriptors; ++i ) {
        for ( size_t j = 0; j < 128; ++j) {
            desc.at<float> (i, j) = descriptors[j+kpts_index[i]*128];
        }
    }
    return desc;
}

vector<float> Pipeline2D::desc2GPUdesc (Mat descs) {
    unsigned int r = descs.rows;
    unsigned int c = descs.cols;
    vector<float> descriptors;
    for ( size_t i = 0; i < r; ++i ) {
        for ( size_t j = 0; j < c; ++j) {
            descriptors.push_back(descs.at<float>(i, j));
        }
    }
    return descriptors;
}

