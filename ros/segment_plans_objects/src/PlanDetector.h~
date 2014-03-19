#ifndef DEF_ROBBOT_PLAN_DETECTOR
#define DEF_ROBBOT_PLAN_DETECTOR

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/crop_box.h>

#include "opencv2/imgproc/imgproc.hpp"

#include "defines.h"

class PlanDetector
{
 public:
	PlanDetector ();
	~PlanDetector ();
	//////////////////////////////////////////////////////////////////////////////
	//
	// Public functions
	//
	//////////////////////////////////////////////////////////////////////////////
	int segment_plans (Cloud::ConstPtr cloud);
	// Segments the last pointcloud filtered from plans
	int segment_clusters (Cloud::ConstPtr cloud);
	int segment_clusters_all ();
	int segment_cloud (Cloud::ConstPtr cloud);
	int segment_rgb (cv::Mat in_img);
	cv::Mat* get_cluster_picture (Cloud::ConstPtr cloud, cv::Mat img);
	//////////////////////////////////////////////////////////////////////////////
	//
	// Getter functions
	//
	//////////////////////////////////////////////////////////////////////////////
	Cloud::ConstPtr get_prefered_plan ();
	Cloud::ConstPtr get_plan (size_t i);
	std::vector<Cloud::ConstPtr> get_plans ();	
	Cloud::ConstPtr get_cluster(size_t i);
	std::vector<Cloud::ConstPtr> get_clusters();
	cv::Mat* get_rgb_cluster(size_t i);
	std::vector<cv::Mat*> get_rgb_clusters();
	Cloud::ConstPtr get_filtered_cloud ();
	size_t get_num_plans();
	size_t get_num_clusters();
	size_t get_num_rgbs();
	
	Eigen::Vector3f get_prefered_plan_translation ();
	Eigen::Quaternionf get_prefered_plan_orientation ();
	Eigen::Vector4f get_prefered_plan_min ();
	Eigen::Vector4f get_prefered_plan_max ();
 private:
	double min_distance_to_z_axis (Cloud::ConstPtr plan);
	// Keep only points within "tresh" distance to the camera axis.
	void attention_cylinder (Cloud::ConstPtr in, float thresh, Cloud::Ptr out);
	void remove_plans (Cloud::ConstPtr in, Cloud::Ptr out);
	void compute_prefered_plan_pose ();
	void compute_plan_pose (Cloud::ConstPtr cloud, pcl::ModelCoefficients::ConstPtr coefs,
													Eigen::Vector3f &translation, Eigen::Vector3f &orientation,
													Eigen::Vector4f &min, Eigen::Vector4f &max, Eigen::Matrix3f &rot_mat);
	void cropbox_prefered_plan (Cloud::ConstPtr in, float offset, Cloud::Ptr out);
	cv::Rect get_roi (Cloud::ConstPtr cloud);
	void reproj(PointType p, int &i, int &j);
	//////////////////////////////////////////////////////////////////////////////
	//
	// Computation Machines
	//
	//////////////////////////////////////////////////////////////////////////////
	pcl::SACSegmentation< PointType > m_seg;
  pcl::VoxelGrid< PointType > m_vox;
	pcl::CropBox< PointType > m_cropbox;
	pcl::ExtractIndices< PointType > m_extract;
	pcl::search::KdTree< PointType >::Ptr m_tree;
  pcl::EuclideanClusterExtraction< PointType > m_ec;
	//////////////////////////////////////////////////////////////////////////////
	//
	// Temporary internal variable
	//
	//////////////////////////////////////////////////////////////////////////////
	// Input cloud subsampled and without plans
	Cloud::Ptr m_cloud_filtered;
  std::vector<pcl::PointIndices> m_cluster_indices;	
	//////////////////////////////////////////////////////////////////////////////
	//
	// Output variables
	//
	//////////////////////////////////////////////////////////////////////////////
	std::vector<pcl::ModelCoefficients::ConstPtr> m_vec_coefs;
	std::vector< Cloud::ConstPtr > m_vec_plans;
  std::vector< Cloud::ConstPtr > m_cloud_cluster;
	std::vector<cv::Mat*> m_rgb_images;
	// Prefered plan is the closest to the optical axis, i.e. the one the cameras
	// are looking more directly.
	int m_prefered_plan_idx;
	double m_max_distance_to_plan;
	bool m_plans_segmented;
	
	Eigen::Vector3f m_prefered_plan_translation;
	Eigen::Vector3f m_prefered_plan_orientation;
	Eigen::Matrix3f m_prefered_plan_rotation_matrix;
	Eigen::Vector4f m_prefered_plan_min;
	Eigen::Vector4f m_prefered_plan_max;
};

#endif
