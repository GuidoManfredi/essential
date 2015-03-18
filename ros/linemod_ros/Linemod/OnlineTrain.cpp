#include "OnlineTrain.h"

OnlineTrain::OnlineTrain() {}

void
compute (const PointCloudXYZRGBA::ConstPtr & input, float min_depth, float max_depth, float max_height,
const std::string & template_pcd_filename, const std::string & template_sqmmt_filename)
{
    // Segment the foreground object
    std::vector<bool> foreground_mask = maskForegroundPoints (input, min_depth, max_depth, max_height);
    // Save the masked template cloud (masking with NaNs to preserve its organized structure)
    PointCloudXYZRGBA template_cloud (*input);
    for (size_t i = 0; i < foreground_mask.size (); ++i) {
        if (!foreground_mask[i]) {
            pcl::PointXYZRGBA & p = template_cloud.points[i];
            p.x = p.y = p.z = std::numeric_limits<float>::quiet_NaN ();
        }
    }
    pcl::io::savePCDFile (template_pcd_filename, template_cloud);
    // Create a LINEMOD template
    pcl::LINEMOD linemod;
    trainTemplate (input, foreground_mask, linemod);
    // Save the LINEMOD template
    std::ofstream file_stream;
    file_stream.open (template_sqmmt_filename.c_str (), std::ofstream::out | std::ofstream::binary);
    linemod.getTemplate (0).serialize (file_stream);
    file_stream.close ();
}

std::vector<bool>
OnlineTrain::maskForegroundPoints (const PointCloudXYZRGBA::ConstPtr & input,
                        float min_depth, float max_depth, float max_height) {
    std::vector<bool> foreground_mask (input->size (), false);
    // Mask off points outside the specified near and far depth thresholds
    pcl::IndicesPtr indices (new std::vector<int>);
    for (size_t i = 0; i < input->size (); ++i) {
        const float z = input->points[i].z;
        if (min_depth < z && z < max_depth) {
            foreground_mask[i] = true;
            indices->push_back (static_cast<int> (i));
        }
    }

     // Find the dominant plane between the specified near/far thresholds
    const float distance_threshold = 0.02f;
    const int max_iterations = 500;
    pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (distance_threshold);
    seg.setMaxIterations (max_iterations);
    seg.setInputCloud (input);
    seg.setIndices (indices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    seg.segment (*inliers, *coefficients);

    // Mask off the plane inliers
    for (size_t i = 0; i < inliers->indices.size (); ++i)
        foreground_mask[inliers->indices[i]] = false;

    // Mask off any foreground points that are too high above the detected plane
    const std::vector<float> & c = coefficients->values;
    for (size_t i = 0; i < input->size (); ++i) {
        if (foreground_mask[i]) {
            const pcl::PointXYZRGBA & p = input->points[i];
            float d = fabsf (c[0]*p.x + c[1]*p.y + c[2]*p.z + c[3]);
            foreground_mask[i] = (d < max_height);
        }
    }
    return foreground_mask;
}
