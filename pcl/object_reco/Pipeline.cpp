#include "Pipeline.hpp"

////////////////////////////////////////////////////////////////////////////////
//
//  Constructor
//
////////////////////////////////////////////////////////////////////////////////
Pipeline::Pipeline()
{
}
////////////////////////////////////////////////////////////////////////////////
//
//  Destructor
//
////////////////////////////////////////////////////////////////////////////////
Pipeline::~Pipeline()
{
}
////////////////////////////////////////////////////////////////////////////////
//
//  Compute normals
//
////////////////////////////////////////////////////////////////////////////////
void Pipeline::getNormals (pcl::PointCloud<PointType>::Ptr model,
													 pcl::PointCloud<NormalType>::Ptr &normals)
{
	normals->clear();
	
	m_norm_est.setKSearch (10);
	m_norm_est.setInputCloud (model);
	m_norm_est.compute (*normals);
}
////////////////////////////////////////////////////////////////////////////////
//
// Extract Keypoints : Uniform sampling
//
////////////////////////////////////////////////////////////////////////////////
void Pipeline::getKpts (pcl::PointCloud<PointType>::Ptr model,
												pcl::PointCloud<PointType>::Ptr kpts,
												float kpts_rad)
{
	kpts->clear();

	pcl::PointCloud<int> kpts_idx;
					
	m_kpts_est.setRadiusSearch (kpts_rad);
	m_kpts_est.setInputCloud (model);
	m_kpts_est.compute (kpts_idx);
	pcl::copyPointCloud<PointType, PointType> (*model, kpts_idx.points, *kpts);
	
	std::cout << "Model total points: " << model->size () << "; Selected Keypoints: " << kpts->size () << std::endl;
}
////////////////////////////////////////////////////////////////////////////////
// 
// Compute Descriptors : SHOTS, shape only
//
////////////////////////////////////////////////////////////////////////////////
void Pipeline::getDescs (pcl::PointCloud<PointType>::Ptr model,
												 pcl::PointCloud<NormalType>::Ptr normals,
												 pcl::PointCloud<PointType>::Ptr kpts,
												 pcl::PointCloud<DescriptorType>::Ptr descs,
												 float descs_rad)
{
	descs->clear();

	m_descr_est.setRadiusSearch (descs_rad);
	m_descr_est.setInputCloud (kpts);
	m_descr_est.setInputNormals (normals);
	m_descr_est.setSearchSurface (model);
	m_descr_est.compute (*descs);
}
////////////////////////////////////////////////////////////////////////////////
//
// Match Descriptors : KDTree + FLANN + L2 distance
//
////////////////////////////////////////////////////////////////////////////////
void Pipeline::match (pcl::PointCloud<DescriptorType>::Ptr model,
											pcl::PointCloud<DescriptorType>::Ptr scene,
											pcl::CorrespondencesPtr &model_scene_corrs)
{
	pcl::search::FlannSearch<DescriptorType> matcher;
	matcher.setInputCloud (model);
	  
  for (size_t i = 0; i < scene->size (); ++i)
  {
		std::vector<int> neigh_indices (1);
		std::vector<float> neigh_sqr_dists (1);
		if (!pcl_isfinite (scene->at (i).descriptor[0])) //skipping NaNs
		{
			continue;
		}  
		int found_neighs = matcher.nearestKSearch (scene->at (i), 1, neigh_indices, neigh_sqr_dists);
		//  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
		if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) 
		{
			pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
			model_scene_corrs->push_back (corr);
		}
	}
	std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;
}
////////////////////////////////////////////////////////////////////////////////
// 
// Correspondence Grouping : Geometric consistency
//
////////////////////////////////////////////////////////////////////////////////
void Pipeline::grouping (pcl::PointCloud<PointType>::Ptr model_kpts,
												 pcl::PointCloud<PointType>::Ptr scene_kpts,
												 pcl::CorrespondencesPtr model_scene_corrs,
												 std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > &rototranslations,												 
												 std::vector<pcl::Correspondences> &clusters,
												 float cg_size,
												 float cg_thresh)												 
{
	clusters.clear();

  m_gc_clusterer.setGCSize (cg_size);
  m_gc_clusterer.setGCThreshold (cg_thresh);

  m_gc_clusterer.setInputCloud (model_kpts);
  m_gc_clusterer.setSceneCloud (scene_kpts);
  m_gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

  //m_gc_clusterer.cluster (clusters);
  m_gc_clusterer.recognize (rototranslations, clusters);
  std::cout << "Clusters formed: " << clusters.size () << std::endl;
}
////////////////////////////////////////////////////////////////////////////////
// 
// Registration : RANSAC
//
////////////////////////////////////////////////////////////////////////////////
void Pipeline::registration (pcl::PointCloud<PointType>::Ptr model_kpts,
														 pcl::PointCloud<PointType>::Ptr scene_kpts,
														 pcl::Correspondences cluster_in,
														 std::vector<int> &inliers,
														 Eigen::Matrix4f &T)
{
	inliers.clear();

	std::vector<int> model_idx;
	std::vector<int> scene_idx;
	for(size_t n=0; n<cluster_in.size(); ++n)
	{
		model_idx.push_back(cluster_in[n].index_query);
		scene_idx.push_back(cluster_in[n].index_match);
	}

	pcl::SampleConsensusModelRegistration< PointType >::Ptr sac (new 	pcl::SampleConsensusModelRegistration< PointType > (model_kpts, model_idx) );
	sac->setInputTarget (scene_kpts, scene_idx);

	pcl::RandomSampleConsensus<PointType> ransac (sac);
	ransac.setDistanceThreshold (0.1);
	ransac.computeModel (1);
	// Get4x4 transform matrix
	Eigen::VectorXf coeffs;
  ransac.getModelCoefficients (coeffs);
  assert(coeffs.size() == 16);
	T = Eigen::Map<Eigen::Matrix4f> (coeffs.data(),4,4);
  // Save inliers
	ransac.getInliers (inliers);
}
////////////////////////////////////////////////////////////////////////////////
// 
// Pose Refinement : ICP
//
////////////////////////////////////////////////////////////////////////////////
void Pipeline::reffinement (pcl::PointCloud<PointType>::Ptr model_kpts,
														pcl::PointCloud<PointType>::Ptr scene_kpts,
														Eigen::Matrix4f &T)
{
  pcl::PointCloud<PointType> Final;

  m_icp.setInputSource(model_kpts);
  m_icp.setInputTarget(scene_kpts);
  m_icp.align(Final);
  std::cout << "has converged:" << m_icp.hasConverged() << " score: " << m_icp.getFitnessScore() << std::endl;
  T = m_icp.getFinalTransformation();
}
////////////////////////////////////////////////////////////////////////////////
// 
// Hypothesis verification
//
////////////////////////////////////////////////////////////////////////////////

/*
    if (ICP_iterations_ > 0)
    {
      pcl::ScopeTime ticp ("ICP ");

      //Prepare scene and model clouds for the pose refinement step
      PointInTPtr cloud_voxelized_icp (new pcl::PointCloud<PointInT> ());
      pcl::VoxelGrid<PointInT> voxel_grid_icp;
      voxel_grid_icp.setInputCloud (processed);
      voxel_grid_icp.setLeafSize (VOXEL_SIZE_ICP_, VOXEL_SIZE_ICP_, VOXEL_SIZE_ICP_);
      voxel_grid_icp.filter (*cloud_voxelized_icp);
      source_->voxelizeAllModels (VOXEL_SIZE_ICP_);

#pragma omp parallel for schedule(dynamic,1) num_threads(omp_get_num_procs())
      for (int i = 0; i < static_cast<int>(models_->size ()); i++)
      {

        ConstPointInTPtr model_cloud;
        PointInTPtr model_aligned (new pcl::PointCloud<PointInT>);
        model_cloud = models_->at (i).getAssembled (VOXEL_SIZE_ICP_);
        pcl::transformPointCloud (*model_cloud, *model_aligned, transforms_->at (i));

        typename pcl::registration::CorrespondenceRejectorSampleConsensus<PointInT>::Ptr rej (
            new pcl::registration::CorrespondenceRejectorSampleConsensus<PointInT> ());

        rej->setInputTarget (cloud_voxelized_icp);
        rej->setMaximumIterations (1000);
        rej->setInlierThreshold (0.005f);
        rej->setInputSource (model_aligned);

        pcl::IterativeClosestPoint<PointInT, PointInT> reg;
        reg.addCorrespondenceRejector (rej);
        reg.setInputTarget (cloud_voxelized_icp); //scene
        reg.setInputSource (model_aligned); //model
        reg.setMaximumIterations (ICP_iterations_);
        reg.setMaxCorrespondenceDistance (VOXEL_SIZE_ICP_ * 4.f);

        typename pcl::PointCloud<PointInT>::Ptr output_ (new pcl::PointCloud<PointInT> ());
        reg.align (*output_);

        Eigen::Matrix4f icp_trans = reg.getFinalTransformation ();
        transforms_->at (i) = icp_trans * transforms_->at (i);
      }
    }
*/

/*
    if (hv_algorithm_)
    {

      pcl::ScopeTime thv ("HV verification");

      std::vector<typename pcl::PointCloud<PointInT>::ConstPtr> aligned_models;
      aligned_models.resize (models_->size ());
      for (size_t i = 0; i < models_->size (); i++)
      {
        ConstPointInTPtr model_cloud = models_->at (i).getAssembled (0.0025f);
        //ConstPointInTPtr model_cloud = models_->at (i).getAssembled (VOXEL_SIZE_ICP_);
        PointInTPtr model_aligned (new pcl::PointCloud<PointInT>);
        pcl::transformPointCloud (*model_cloud, *model_aligned, transforms_->at (i));
        aligned_models[i] = model_aligned;
      }

      std::vector<bool> mask_hv;
      hv_algorithm_->setSceneCloud (input_);
      hv_algorithm_->addModels (aligned_models, true);
      hv_algorithm_->verify ();
      hv_algorithm_->getMask (mask_hv);

      boost::shared_ptr < std::vector<ModelT> > models_temp;
      boost::shared_ptr < std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > transforms_temp;

      models_temp.reset (new std::vector<ModelT>);
      transforms_temp.reset (new std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >);

      for (size_t i = 0; i < models_->size (); i++)
      {
        if (!mask_hv[i])
          continue;

        models_temp->push_back (models_->at (i));
        transforms_temp->push_back (transforms_->at (i));
      }

      models_ = models_temp;
      transforms_ = transforms_temp;

*/
