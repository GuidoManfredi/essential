#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>

#include "defines.h"
#include "Pipeline.hpp"

int
main (int argc, char** argv)
{
	if(argc != 3)
	{
		std::cout << "Usage : main model.pcd scene.pcd" << endl;
		return (-1);
	}

	//
	// Process clouds
	//
	Pipeline p;
	//
	// Pointcloud loading
	//
	pcl::PointCloud<PointType>::Ptr	scene (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<PointType>::Ptr model (new pcl::PointCloud<pcl::PointXYZ>);	
	pcl::io::loadPCDFile<PointType> (argv[1], *scene);
	pcl::io::loadPCDFile<PointType> (argv[2], *model);
	//
	// Normal extraction
	//
	std::cout << "Normal estimation ..." << endl;
	pcl::PointCloud<NormalType>::Ptr scene_normals (new pcl::PointCloud<NormalType> ());
	pcl::PointCloud<NormalType>::Ptr model_normals (new pcl::PointCloud<NormalType> ());
	p.getNormals(scene, scene_normals);
	p.getNormals(model, model_normals);
	std::cout << "... done" << endl;
	//
	// Keypoints extraction
	//
	std::cout << "Keypoints extraction ..." << endl;
	pcl::PointCloud<PointType>::Ptr scene_kpts (new pcl::PointCloud<PointType> ());
	pcl::PointCloud<PointType>::Ptr model_kpts (new pcl::PointCloud<PointType> ());
	p.getKpts(scene, scene_kpts, 0.03f);
	p.getKpts(model, model_kpts, 0.03f);
	std::cout << "... done" << endl;
	//
	// Descriptors extraction
	//
	std::cout << "Descriptors computation ..." << endl;
	pcl::PointCloud<DescriptorType>::Ptr scene_descs (new pcl::PointCloud<DescriptorType> ());
	pcl::PointCloud<DescriptorType>::Ptr model_descs (new pcl::PointCloud<DescriptorType> ());
	p.getDescs(scene, scene_normals, scene_kpts, scene_descs);
	p.getDescs(model, model_normals, model_kpts, model_descs);
	std::cout << "... done" << endl;
	//
	// Matching
	//
	std::cout << "Matching ..." << endl;
	pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());
	p.match(model_descs, scene_descs, model_scene_corrs);
	std::cout << "... done" << endl;
	
/*
	std::cout << T(0,0) << " " << T(1,0) << " " << T(2,0) << " " << T(3,0) << endl;
	std::cout << T(0,1) << " " << T(1,1) << " " << T(2,1) << " " << T(3,1) << endl;
	std::cout << T(0,2) << " " << T(1,2) << " " << T(2,2) << " " << T(3,2) << endl;
	std::cout << T(0,3) << " " << T(1,3) << " " << T(2,3) << " " << T(3,3) << endl;
*/
	return (0);
}

