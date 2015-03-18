#pragma once

#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

class OnlineTrain {
    OnlineTrain();

    // Computes the foreground mask, add the template to the underlying linemod and
    //  save the templates in sqmmt and pcd files.
    void
    compute (const PointCloudXYZRGBA::ConstPtr & input, float min_depth, float max_depth, float max_height,
            const std::string & template_pcd_filename, const std::string & template_sqmmt_filename);
    // Remove point too close, too far, segment the main plane in view and remove
    //  point too high above this plane. Returns a mask.
    std::vector<bool>
    maskForegroundPoints (const PointCloudXYZRGBA::ConstPtr & input,
                        float min_depth, float max_depth, float max_height);

};
