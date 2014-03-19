#include "Reco.hpp"

Reco::Reco()
{
}

Reco::~Reco()
{
}

void Reco::load_models (std::string dirname)
{
	cout << "Loading models from " << dirname << endl;
	Model tmp_model;
	pcl::PointCloud<PointType>::Ptr tmp_kpts (new pcl::PointCloud<PointType>);
	pcl::PointCloud<DescriptorType>::Ptr tmp_descs (new pcl::PointCloud<DescriptorType>);
	pcl::PointCloud<PointType>::Ptr tmp_agreg_kpts (new pcl::PointCloud<PointType>);
	pcl::PointCloud<DescriptorType>::Ptr tmp_agreg_descs (new pcl::PointCloud<DescriptorType>);	
	
	std::vector < std::string > files;
  std::string start = "";
  bf::path dir = dirname;	
	getDirRec (dir, start, files);

	std::vector<std::string> kpts_files;	
	std::vector<std::string> descs_files;
	bf::path descs_path;
	bf::path kpts_path;
	
	//
	// For each model load keypoints and descriptors
	//
	for (size_t n=0; n< files.size () - 1; n+=2)
	{
		cout << "Loading model " << n << endl;
		descs_path = dirname + files[n] + "/";
		getFileByNameAndExt ( descs_path, descs_files, "descriptors", "pcd");
	
		kpts_path = dirname + files[n+1] + "/";
		getFileByNameAndExt ( kpts_path, kpts_files, "keypoints", "pcd");

		//
		// Load keypoints
		//	
		for (size_t k=0; k< kpts_files.size (); ++k)
		{
			std::string kpt_full_path = dirname + files[n+1] + "/" + kpts_files[k];
			//cout << "Loading keypoint from " << kpt_full_path << "." << endl;
			pcl::io::loadPCDFile<PointType> (kpt_full_path, *tmp_kpts);
			*tmp_agreg_kpts = *tmp_agreg_kpts + *tmp_kpts;
		}
		//
		// Load descriptors
		//
		for (size_t k=0; k<descs_files.size(); ++k)
		{
			std::string desc_full_path = dirname + files[n] + "/" + descs_files[k];		
			//cout << "Loading descriptor from " << desc_full_path << "." << endl;
			pcl::io::loadPCDFile<DescriptorType> (desc_full_path, *tmp_descs);
			*tmp_agreg_descs = *tmp_agreg_descs + *tmp_descs;
		}
		
		tmp_model.agreg_kpts = tmp_agreg_kpts;
		tmp_model.agreg_descs = tmp_agreg_descs;
		tmp_model.kpts.push_back(tmp_kpts);
		tmp_model.descs.push_back(tmp_descs);
		m_models.push_back(tmp_model);
	}
	
	cout << "Loaded "<< files.size ()/2 << " model(s) for " << tmp_model.agreg_kpts->size() << " points" << endl;
}

void Reco::run(pcl::PointCloud<PointType>::Ptr scene)
{
	//
	// Process clouds
	//
	Pipeline p;
	//
	// Normal extraction
	//
  pcl::PointCloud<NormalType>::Ptr scene_normals (new pcl::PointCloud<NormalType> ());
	p.getNormals(scene, scene_normals);
	cout << "Extracted " << scene_normals->size() << " normals." << endl;
	//
	// Keypoints extraction
	//
  pcl::PointCloud<PointType>::Ptr scene_kpts (new pcl::PointCloud<PointType> ());
	p.getKpts(scene, scene_kpts, 0.01f);
	cout << "Extracted " << scene_kpts->size() << " keypoints." << endl;	
	//
	// Descriptors extraction
	//
  pcl::PointCloud<DescriptorType>::Ptr scene_descs (new pcl::PointCloud<DescriptorType> ());
	p.getDescs(scene, scene_normals, scene_kpts, scene_descs, 0.03f);
	cout << "Extracted " << scene_descs->size() << " descriptors." << endl;

	//
	// Processing for each model
	//
	for (size_t n = 0; n < m_models.size(); ++n)
	{
		cout << "kikou" << endl;
		//
		// Keypoints matching : match scene descriptors with model's descriptors
		//
		pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());
		p.match (scene_descs, m_models[n].agreg_descs, model_scene_corrs);
		cout << "Found " << model_scene_corrs->size() << " matchs for model " << n << "." << endl;
		//
		// Correspondance grouping
		//
	  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;	
		std::vector<pcl::Correspondences> clustered_corrs;
		p.grouping(scene_kpts, m_models[n].agreg_kpts, model_scene_corrs, rototranslations, clustered_corrs);
	}

	/*
	//
	// Registration
	//
	std::vector<int> inliers;
	Eigen::Matrix4f T;
	p.registration (model_kpts, scene_kpts, clustered_corrs[0], inliers, T);

  pcl::PointCloud<PointType>::Ptr scene_inliers (new pcl::PointCloud<PointType> ());
  for (size_t n=0; n<inliers.size(); ++n)
  	scene_inliers->push_back (scene_kpts->at (inliers[n]));
	p.reffinement(model_kpts, scene_inliers, T);
	*/
}
