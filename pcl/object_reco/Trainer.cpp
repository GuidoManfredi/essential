#include "Trainer.hpp"

//using namespace pcl:

Trainer::Trainer ()
{
	m_kpts_radius = 0.01;
	m_descs_radius = 0.01;
}

Trainer::~Trainer ()
{
}

void
Trainer::load_model (const std::string &filename)
{ 
	cout << "Loading " << filename << "." << endl;
	std::vector< std::string > dirs;
	std::vector< std::string > file;	
	boost::split (dirs, filename, boost::is_any_of ("/"));
	boost::split (file, dirs[dirs.size () - 1], boost::is_any_of ("."));	
	m_modelname = file[0];
	//
  // Load the target mesh PLY file
  //  
  vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New ();
  reader->SetFileName (filename.c_str ());

  //
  // Extract views from faces of icosahedron
  //
	vtkSmartPointer < vtkPolyDataMapper > mapper =
	vtkSmartPointer<vtkPolyDataMapper>::New ();
	mapper->SetInputConnection (reader->GetOutputPort ());
	mapper->Update ();
  
	pcl::apps::RenderViewsTesselatedSphere renderer;
	renderer.addModelFromPolyData (mapper->GetInput ());
	renderer.generateViews ();
	renderer.getViews (m_views);
  renderer.getPoses (m_poses);
  renderer.getEntropies (m_entropies);  

	//assemble_views ();

	cout << "Extracted and assembled " << m_views.size () << " views." << endl;
}

void
Trainer::assemble_views ()
{
  for (size_t i=0; i < m_views.size (); ++i)
  {
	  pcl::PointCloud<PointType>::Ptr global (new pcl::PointCloud<PointType>);
    Eigen::Matrix4f inv_trans = m_poses[i];
    pcl::transformPointCloud (*(m_views[i]),*global, inv_trans);
    *(m_assembled) += *global;
  }
}

void
Trainer::computeCloudResolution (const pcl::PointCloud<PointType>::ConstPtr &cloud,
																	double &kpts_rad,
																	double &descs_rad)
{
  std::vector<double> vals;
  double res = 0.0;
  int n_points = 0;
  int nres;
  std::vector<int> indices (2);
  std::vector<float> sqr_distances (2);
  pcl::search::KdTree<PointType> tree;
  tree.setInputCloud (cloud);

  for (size_t i = 0; i < cloud->size (); ++i)
  {
    if (! pcl_isfinite ((*cloud)[i].x))
    {
      continue;
    }
    //Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
    if (nres == 2)
    {
      vals.push_back(sqrt (sqr_distances[1]));
      ++n_points;
    }
  }
  if (n_points != 0)
  {
		sort (vals.begin (), vals.end ());
		res = vals [floor (n_points/2)];
  }
  
  cout << "Kikou res : " << res << endl;
  
  kpts_rad *= res;
  descs_rad *= res;
}


void 
Trainer::train_model ()
{
	cout << "Starting features extraction ..." << endl;
	m_kpts.clear ();
	m_descs.clear ();

	pcl::PointCloud<NormalType>::Ptr model_normals (new pcl::PointCloud<NormalType> ());
	pcl::PointCloud<PointType>::Ptr model_kpts (new pcl::PointCloud<PointType> ());
	pcl::PointCloud<DescriptorType>::Ptr model_descs (new pcl::PointCloud<DescriptorType> ());
	//
	// For each view of the object
	//
	for (size_t n=0; n<m_views.size(); ++n)
	{
		double kpts_rad = m_kpts_radius;
		double descs_rad = m_descs_radius;
	
		computeCloudResolution (m_views[n], kpts_rad, descs_rad);
	  cout << "Resolution  used : " << kpts_rad << " " << descs_rad << endl;		
		//
		// Normal extraction, only to compute descriptors
		//		
		p.getNormals (m_views[n], model_normals);
		//
		// Keypoints extraction
		//		
		p.getKpts (m_views[n], model_kpts, kpts_rad);
		//
		// Descriptors extraction
		//  
		p.getDescs (m_views[n], model_normals, model_kpts, model_descs, descs_rad);
		//
		// Store data
		//
		m_kpts.push_back (*model_kpts);
		m_descs.push_back (*model_descs);
	}
	
	cout << "... features extraction done." << endl;
}

pcl::PointCloud<PointType>::Ptr
Trainer::getAssembled (float resolution)
{
  if (resolution <= 0)
    return m_assembled;

  std::map<float, pcl::PointCloud<PointType>::Ptr >::iterator it = m_voxelized_assembled.find (resolution);
  if (it == m_voxelized_assembled.end ())
  {
    pcl::PointCloud<PointType>::Ptr voxelized (new pcl::PointCloud<PointType>);
    pcl::VoxelGrid<PointType> grid_;
    grid_.setInputCloud (m_assembled);
    grid_.setLeafSize (resolution, resolution, resolution);
    grid_.setDownsampleAllData(true);
    grid_.filter (*voxelized);

    pcl::PointCloud<PointType>::Ptr voxelized_const (new pcl::PointCloud<PointType> (*voxelized));
    m_voxelized_assembled[resolution] = voxelized_const;
    return voxelized_const;
  }

  return it->second;
}

void Trainer::save_model (std::string dirname)
{
	std::ostringstream filename;
	std::ofstream infofile;
	//
	//  Creating model directory
	//
	std::string model_path;	
	model_path = dirname + m_modelname;
	if (!bf::exists (model_path))
		createDir (model_path);
	cout << "Saving features to " << model_path << endl;
	//
	//  Writing views
	//
	std::string views_path;
	views_path = model_path + "/views";
	if (!bf::exists (views_path))
		createDir (views_path);
	for (size_t n=0; n<m_views.size (); ++n)
	{
		filename << views_path << "/view_" << n << ".pcd";
		pcl::io::savePCDFileBinary (filename.str (), *m_views[n]);
		filename.str ("");
	}
	//
	//	Writing keypoints
	//
	std::string kpts_path;
	kpts_path = model_path + "/keypoints";
	if (!bf::exists (kpts_path))
		createDir (kpts_path);
	for (size_t n=0; n<m_kpts.size (); ++n)
	{
		filename << kpts_path << "/keypoints_" << n << ".pcd";
		pcl::io::savePCDFileBinary (filename.str (), m_kpts[n]);
		filename.str ("");
	}
	//
	//	Writing descriptors
	//
	std::string descs_path;
	descs_path = model_path + "/descriptors";
	if (!bf::exists (descs_path))
		createDir (descs_path);
	for (size_t n=0; n<m_descs.size (); ++n)
	{
		filename << descs_path << "/descriptors_" << n << ".pcd";
		pcl::io::savePCDFileBinary (filename.str (), m_descs[n]);
		filename.str ("");
	}
}
