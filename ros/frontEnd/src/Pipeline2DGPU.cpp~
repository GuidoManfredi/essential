#include "Pipeline2DGPU.h"

using namespace std;
using namespace cv;

Pipeline2DGPU::Pipeline2DGPU () {
    char * argv[] ={ "-fo", "-1", "-v", "0"};
    sift.ParseParam(4, argv); 
    int support = sift.CreateContextGL();  
    if(support != SiftGPU::SIFTGPU_FULL_SUPPORTED) {
        cout << "SiftGPU not supported" << endl;
    };

    //matcher_.SetMaxSift (4096);

    minNumberMatchesAllowed_                  = 8;
    matches_kept_                             = 0.8;
}

void Pipeline2DGPU::getGray(const cv::Mat& image, cv::Mat& gray)
{
    assert(!image.empty());
    if (image.channels()  == 3)
        cvtColor(image, gray, CV_BGR2GRAY);
    else if (image.channels() == 4)
        cvtColor(image, gray, CV_BGRA2GRAY);
    else if (image.channels() == 1)
        gray = image;
}

bool Pipeline2DGPU::detectFeatures(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints)
{
    assert(!image.empty());
    assert(image.channels() == 1);
    keypoints.clear();
    
    int width = image.cols;
    int height = image.rows;
    unsigned char *data = image.data;
    //sift.RunSIFT (width, height, data, GL_RGBA, GL_UNSIGNED_BYTE);
    sift.RunSIFT (width, height, data, GL_LUMINANCE, GL_UNSIGNED_BYTE);
    
    int num_keypoints = sift.GetFeatureNum();
    
    vector<SiftGPU::SiftKeypoint> keys(num_keypoints);
    sift.GetFeatureVector(&keys[0], NULL);
    // converting to opencv
    for ( size_t i = 0; i < num_keypoints; ++i ) {
        KeyPoint kpt = GPUkpt2kpt (keys[i]);
        keypoints.push_back (kpt);
    }
    if ( num_keypoints == 0 )
        return false;
    return true;
}

bool Pipeline2DGPU::describeFeatures(const cv::Mat image, std::vector<cv::KeyPoint> keypoints, cv::Mat& descriptors)
{
    assert(!image.empty());
    assert(image.channels() == 1);
    
    int num_keypoints = sift.GetFeatureNum();
    if ( num_keypoints != 0 ) {
	    vector<float> gpu_descriptors(128*num_keypoints);
	    sift.GetFeatureVector(NULL, &gpu_descriptors[0]);
	    // converting to opencv
	    descriptors = GPUdesc2desc (gpu_descriptors);
    	return true;
    }

    return false;
}

bool Pipeline2DGPU::describeFeatures(const cv::Mat image, std::vector<int> keypoints_index, cv::Mat& descriptors)
{
    assert(!image.empty());
    assert(image.channels() == 1);
    
    int num_keypoints = sift.GetFeatureNum();
    if ( num_keypoints != 0 ) {
	    vector<float> gpu_descriptors(128*num_keypoints);
	    sift.GetFeatureVector(NULL, &gpu_descriptors[0]);
	    // converting to opencv
	    descriptors = GPUdesc2desc (gpu_descriptors, keypoints_index);
    	return true;
    }

    return false;
}

bool Pipeline2DGPU::match(const cv::Mat &desc1, const cv::Mat &desc2,
                           std::vector<cv::DMatch>& matches) {
    assert (desc1.data && desc2.data);
    matches.clear();
    SiftMatchGPU matcher_;
    matcher_.SetMaxSift (4096);
    if(matcher_.VerifyContextGL() == 0) return false;
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
        //cout << match_buf[i][0] << " " << match_buf[i][1] << endl;
        matches.push_back (m);
    }
    //cout << "Descs " << des1.size()/128 << " " << des2.size()/128 << endl;
    if ( nmatch > minNumberMatchesAllowed_ )
        return true;

    return false;
}
               
////////////////////////////////////////////////////////////////////////////////
//  PRIVATE METHOD
////////////////////////////////////////////////////////////////////////////////

KeyPoint Pipeline2DGPU::GPUkpt2kpt (SiftGPU::SiftKeypoint key) {
    KeyPoint kpt;
    kpt.pt.x = key.x;
    kpt.pt.y = key.y;
    kpt.octave = key.s;
    kpt.angle = key.o;
    return kpt;
}   

Mat Pipeline2DGPU::GPUdesc2desc (vector<float> descriptors) {
    int num_descriptors = descriptors.size()/128;
    Mat desc (num_descriptors, 128, CV_32F); // 128 columns
    for ( size_t i = 0; i < num_descriptors; ++i ) {
        for ( size_t j = 0; j < 128; ++j) {
            desc.at<float> (i, j) = descriptors[j+i*128];
        }
    }
    return desc;
}

Mat Pipeline2DGPU::GPUdesc2desc (vector<float> descriptors, vector<int> kpts_index) {
    int num_descriptors = kpts_index.size();
    Mat desc (num_descriptors, 128, CV_32F); // 128 columns
    for ( size_t i = 0; i < num_descriptors; ++i ) {
        for ( size_t j = 0; j < 128; ++j) {
            desc.at<float> (i, j) = descriptors[j+kpts_index[i]*128];
        }
    }
    return desc;
}

vector<float> Pipeline2DGPU::desc2GPUdesc (Mat descs) {
    int r = descs.rows;
    int c = descs.cols;
    vector<float> descriptors;
    for ( size_t i = 0; i < r; ++i ) {
        for ( size_t j = 0; j < c; ++j) {
            descriptors.push_back(descs.at<float>(i, j));
        }
    }
    return descriptors;
}
