#ifndef DEF_TRAINER
#define DEF_TRAINER

#include <iostream>
// Training Model Processing : views creation
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/apps/render_views_tesselated_sphere.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/impl/point_types.hpp>

#include "file_tools.h"
#include "Pipeline.hpp"

class Trainer
{
public:
	Trainer ();
	~Trainer ();
	void load_model (const std::string &filename);
	void assemble_views();	
	void computeCloudResolution (const pcl::PointCloud<PointType>::ConstPtr &cloud,
																double &kpts_rad,
																double &descs_rad);
	void train_model ();
	pcl::PointCloud<PointType>::Ptr getAssembled (float resolution);
	void save_model (std::string dirname);

protected:
	std::string m_modelname;

 	std::vector< pcl::PointCloud<PointType>::Ptr > m_views;
 	std::vector< Eigen::Matrix4f, Eigen::aligned_allocator< Eigen::Matrix4f > > m_poses;
 	std::vector<float> m_entropies;
 	
 	pcl::PointCloud<PointType>::Ptr m_assembled;
  std::map<float, pcl::PointCloud<PointType>::Ptr > m_voxelized_assembled; 	
 	
	Pipeline p;
	std::vector< pcl::PointCloud<PointType> > m_kpts;
	std::vector< pcl::PointCloud<DescriptorType> > m_descs;

	double m_kpts_radius;	
	double m_descs_radius;
};

#endif
