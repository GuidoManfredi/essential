#include "TrainLinemod.h"

#include <pcl/recognition/color_gradient_modality.h>
#include <pcl/recognition/surface_normal_modality.h>

using namespace std;

using namespace pcl;
using namespace pcl::io;

TrainLinemod::TrainLinemod() {}

bool
TrainLinemod::loadCloud (const std::string & filename, PointCloudXYZRGBA & cloud) {
    cout << "Loading " << filename.c_str () << endl;
    if (loadPCDFile (filename, cloud) < 0)
        return false;

    cout << "Available dimensions: " << pcl::getFieldsList(cloud).c_str() << endl;
    return true;
}

void
TrainLinemod::trainTemplate (const PointCloudXYZRGBA::ConstPtr & input, pcl::LINEMOD & linemod) {
    std::vector<bool> foreground_mask (input->size (), false);
    for (size_t i = 0; i < input->size (); ++i) {
        PointType pt = input->points[i];
        if (pt.x && pt.y && pt.z) // Object point
            foreground_mask[i] = true;
    }
    trainTemplate (input, foreground_mask, linemod);
}

void
TrainLinemod::trainTemplate (const PointCloudXYZRGBA::ConstPtr & input, const std::vector<bool> &foreground_mask,
pcl::LINEMOD & linemod) {
    pcl::ColorGradientModality<pcl::PointXYZRGBA> color_grad_mod;
    color_grad_mod.setInputCloud (input);
    color_grad_mod.processInputData ();
    pcl::SurfaceNormalModality<pcl::PointXYZRGBA> surface_norm_mod;
    surface_norm_mod.setInputCloud (input);
    surface_norm_mod.processInputData ();

    std::vector<pcl::QuantizableModality*> modalities (2);
    modalities[0] = &color_grad_mod;
    modalities[1] = &surface_norm_mod;

    size_t min_x (input->width), min_y (input->height), max_x (0), max_y (0);
    pcl::MaskMap mask_map (input->width, input->height);
    for (size_t j = 0; j < input->height; ++j) {
        for (size_t i = 0; i < input->width; ++i) {
            mask_map (i,j) = foreground_mask[j*input->width+i];
            if (foreground_mask[j*input->width+i]) {
                min_x = std::min (min_x, i);
                max_x = std::max (max_x, i);
                min_y = std::min (min_y, j);
                max_y = std::max (max_y, j);
            }
        }
    }

    std::vector<pcl::MaskMap*> masks (2);
    masks[0] = &mask_map;
    masks[1] = &mask_map;

    pcl::RegionXY region;
    region.x = static_cast<int> (min_x);
    region.y = static_cast<int> (min_y);
    region.width = static_cast<int> (max_x - min_x + 1);
    region.height = static_cast<int> (max_y - min_y + 1);

    printf ("%d %d %d %d\n", region.x, region.y, region.width, region.height);
    linemod.createAndAddTemplate (modalities, masks, region);
}

/* Add functions to save the computed models
 // Save the masked template cloud (masking with NaNs to preserve its organized structure)
PointCloudXYZRGBA template_cloud (*input);
for (size_t i = 0; i < foreground_mask.size (); ++i)
{
if (!foreground_mask[i])
{
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
*/
