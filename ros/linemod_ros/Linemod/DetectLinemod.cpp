#include "DetectLinemod.h"
/*
using namespace std;
using namespace pcl;
using namespace pcl::io;

DetectLinemod::DetectLinemod(float grad_mag_thresh, float detect_thresh) {
    line_rgbd_.setGradientMagnitudeThreshold (grad_mag_thresh);
    line_rgbd_.setDetectionThreshold (detect_thresh);
}

bool
DetectLinemod::loadCloud (const std::string & filename, PointCloudXYZRGBA & cloud) {
    cout << "Loading " << filename.c_str () << endl;
    if (loadPCDFile (filename, cloud) < 0)
        return false;

    cout << "Available dimensions: " << pcl::getFieldsList(cloud).c_str() << endl;
    return true;
}

bool
DetectLinemod::loadModel (const vector<std::string> & lmt_filenames) {
     // Load the template LMT and PCD files
    for (size_t i = 0; i < lmt_filenames.size (); ++i) {
        line_rgbd_.loadTemplates (lmt_filenames[i]);
    }
}

int
DetectLinemod::detect(PointCloudXYZRGBA::Ptr cloud) {
    // Detect objects
    line_rgbd_.setInputCloud (cloud);
    line_rgbd_.setInputColors (cloud);
    std::vector<LineRGBD<PointType>::Detection> detections;
    line_rgbd_.detect (detections);
    for (size_t i = 0; i < detections.size (); ++i) {
        const LineRGBD<PointXYZRGBA>::Detection & d = detections[i];
        const BoundingBoxXYZ & bb = d.bounding_box;
        cout << d.detection_id << " " <<  d.template_id << " " << d.response << " "
             << bb.x << " " << bb.y << " " << bb.z << " "
             << bb.width << " " << bb.height << " " << bb.depth << endl;
    }

    return 0;
}
*/
