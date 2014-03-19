#ifndef DEF_PIPELINE
#define DEF_PIPELINE

#include <iostream>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/features/shot_omp.h>
#include <pcl/search/flann_search.h>
#include <pcl/search/impl/flann_search.hpp>
#include <pcl/correspondence.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/sample_consensus/sac_model_registration.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/recognition/hv/hv_papazov.h>

#include "defines.h"

#define KPTS_RAD 0.01f
#define DESCS_RAD 0.02f
#define CG_SIZE 0.08f
#define CG_TRESH 5.0f

class Pipeline
{
public:
	Pipeline();
	~Pipeline();
	void run(pcl::PointCloud<PointType> model);

	void getNormals ( pcl::PointCloud<PointType>::Ptr model,
										pcl::PointCloud<NormalType>::Ptr &normals);
	void getKpts (pcl::PointCloud<PointType>::Ptr model,
								pcl::PointCloud<PointType>::Ptr kpts,
								float kpts_rad = KPTS_RAD);	
	void getDescs (pcl::PointCloud<PointType>::Ptr model,
								 pcl::PointCloud<NormalType>::Ptr normals,
								 pcl::PointCloud<PointType>::Ptr kpts,
								 pcl::PointCloud< DescriptorType>::Ptr descs,
								 float descs_rad = DESCS_RAD);
	void match (pcl::PointCloud<DescriptorType>::Ptr model,
							pcl::PointCloud<DescriptorType>::Ptr scene,
							pcl::CorrespondencesPtr &model_scene_corrs);
	void grouping (pcl::PointCloud<PointType>::Ptr model_kpts,
								 pcl::PointCloud<PointType>::Ptr scene_kpts,
								 pcl::CorrespondencesPtr model_scene_corrs,
								 std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >	&rototranslation,
								 std::vector<pcl::Correspondences> &clusters,
								 float cg_size = CG_SIZE,
								 float cg_tresh = CG_TRESH);											
void registration (pcl::PointCloud<PointType>::Ptr model_kpts,
									 pcl::PointCloud<PointType>::Ptr scene_kpts,
									 pcl::Correspondences cluster_in,
									 std::vector<int> &inliers,
									 Eigen::Matrix4f &T);
void reffinement (pcl::PointCloud<PointType>::Ptr model_kpts,
									pcl::PointCloud<PointType>::Ptr scene_kpts,
									Eigen::Matrix4f &T);									 
protected:
	pcl::NormalEstimationOMP<PointType, NormalType> m_norm_est;
	pcl::UniformSampling<PointType> m_kpts_est;
	pcl::SHOTEstimationOMP<PointType, NormalType,  DescriptorType> m_descr_est;
  pcl::GeometricConsistencyGrouping<PointType, PointType> m_gc_clusterer;	
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> m_icp;  
};

#endif
