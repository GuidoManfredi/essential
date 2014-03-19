#include "PlanDetector.h"

using namespace std;

int PlanDetector::detect_plans (pcl::PointCloud<PointType>::ConstPtr cloud)
{
	// clear saved data from previous iteration
	m_vec_coefs.clear ();
	m_vec_plans.clear ();
  
  // quantisation with a voxelgrid
  m_vox.setInputCloud (cloud);
  m_vox.filter (*m_cloud_filtered);
  
  int n_points = (int) m_cloud_filtered->points.size ();
  
  // Plan segmentation
	while (m_cloud_filtered->points.size () > 0.3 * n_points)
	{
    Cloud::Ptr plan (new Cloud);
		pcl::ModelCoefficients::Ptr coefs (new pcl::ModelCoefficients);
	
		m_seg.setInputCloud (m_cloud_filtered);
	  m_seg.segment (*m_inliers, *coefs);
		if (m_inliers->indices.size () == 0)
  	{
  	  PCL_ERROR ("Could not estimate a planar model for the given dataset.");
  	  return (-1);
  	}
    // Extract the inliers
    m_extract.setInputCloud (m_cloud_filtered);
    m_extract.setIndices (m_inliers);
    m_extract.setNegative (false);
    m_extract.filter (*plan);
    std::cerr << "PointCloud representing the planar component: " << plan->width * plan->height << " data points." << std::endl;
    
    m_extract.setNegative (true);
    m_extract.filter (*m_tmp);
    m_cloud_filtered.swap (m_tmp);
    
    // saving data
		m_vec_coefs.push_back (coefs);
		m_vec_plans.push_back (plan);
	}
	
	std::cout << "Detected " << m_vec_coefs.size () << " plans." << std::endl;
	
	return m_vec_coefs.size ();
}

int PlanDetector::segment ()
{
	if (m_cloud_filtered->width && m_cloud_filtered->height)
		return segment (m_cloud_filtered);
	else
	{
		std::cout << "Error: PlanDetector::segment: No filtered cloud present." << std::endl;
		return (0);
	}
}

int PlanDetector::segment (pcl::PointCloud<PointType>::ConstPtr cloud)
{
	m_cloud_cluster.clear ();

  m_tree->setInputCloud (cloud);
  m_ec.setInputCloud (cloud);
  m_ec.extract (m_cluster_indices);

  for (std::vector<pcl::PointIndices>::const_iterator it = m_cluster_indices.begin (); it != m_cluster_indices.end (); ++it)
  {
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
      cloud_cluster->points.push_back (m_cloud_filtered->points[*pit]); //
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    m_cloud_cluster.push_back (cloud_cluster);
  }
  
	std::cout << "Segmented " << m_cloud_cluster.size () << " objects." << std::endl;
	
	return m_cloud_cluster.size ();
}

void PlanDetector::bbox (std::vector<Eigen::Quaternionf> &quaternion, std::vector<Eigen::Vector3f> &translation,
																				 std::vector<double> &width, std::vector<double> &height, std::vector<double> &depth)
{
	int n = m_cloud_cluster.size ();
	quaternion.resize (n);
	translation.resize (n);
	width.resize (n);
	height.resize (n);
	depth.resize (n);
	for (size_t i=0; i<n; ++i)
		bbox_mvbb (m_cloud_cluster[i], quaternion[i], translation[i], width[i], height[i], depth[i]);
}

void PlanDetector::bbox (int id, Eigen::Quaternionf &quaternion, Eigen::Vector3f &translation,
																					double &width, double &height, double &depth)
{
	if (id <m_cloud_cluster.size () -1)
		bbox_mvbb (m_cloud_cluster[id], quaternion, translation, width, height, depth);
	else
		std::cout << "Error: PlanDetector::compute_bounding_box : index out of range" << std::endl;
}

void PlanDetector::bbox_pca (Cloud::ConstPtr cloud,
															Eigen::Quaternionf &quaternion, Eigen::Vector3f &translation,
															double &width, double &height, double &depth)
{
	pcl::PCA< PointType > pca;
	pca.setInputCloud (cloud);
	
	// Project cloud in object frame
	Cloud::Ptr proj (new Cloud);
	pca.project (*cloud, *proj);

	// Get bounding box corners in object frame
	PointType proj_min;
	PointType proj_max;
	pcl::getMinMax3D (*proj, proj_min, proj_max);

	// Compute bounding box dimensions
	width = fabs(proj_max.x-proj_min.x);
	height = fabs(proj_max.y-proj_min.y);
	depth = fabs(proj_max.z-proj_min.z);
	
	// Reconstruct bounding box corners in global frame
	PointType min;
	PointType max;
	pca.reconstruct (proj_min, min);
	pca.reconstruct (proj_max, max);
	
	translation = Eigen::Vector3f (min.x+(max.x-min.x)/2, min.y+(max.y-min.y)/2, min.z+(max.z-min.z)/2);
	Eigen::Matrix3f eig_vectors = pca.getEigenVectors ();
	quaternion = Eigen::Quaternionf (eig_vectors);
	
	cout << "W: " << width << " H: " << height << " D: " << depth << endl;
	cout << "Translation: " << translation.x() << " " << translation.y() << " " << translation.z() << endl;		
	cout << "Rotation: " << quaternion.w() << " " << quaternion.x() << " " << quaternion.y() << " " << quaternion.z() << endl;
}

void PlanDetector::bbox_mvbb (Cloud::ConstPtr cloud,
																		Eigen::Quaternionf &quaternion, Eigen::Vector3f &translation,
																		double &width, double &height, double &depth)
{
	PointType min, max;
	bbox_mvbb (cloud, quaternion, translation, width, height, depth, min, max);
}																		

void PlanDetector::bbox_mvbb (Cloud::ConstPtr cloud,
																		Eigen::Quaternionf &quaternion, Eigen::Vector3f &translation,
																		double &width, double &height, double &depth,
																		PointType &out_min, PointType &out_max)
{																					
	int n = cloud->size();
	gdiam_real* points = (gdiam_real*) malloc (3*n*sizeof(gdiam_real));
	pcl2gdiam (cloud, points);

	GPointPair pair;	
  pair = gdiam_approx_diam_pair( points, n, 0.0 );

  gdiam_point  * pnt_arr;
  pnt_arr = gdiam_convert( (gdiam_real *)points, n );

  gdiam_bbox bb;
  //bb = gdiam_approx_mvbb_grid_sample( pnt_arr, n, 1, 400 );
  //bb = gdiam_approx_mvbb_grid_sample( pnt_arr, n, 5, 400);
  //bb = gdiam_approx_mvbb_grid (pnt_arr, n, 5);
  bb = gdiam_approx_mvbb (pnt_arr, n, 1);
  if (bb.f_init)
  {
		PointType min (bb.low_1, bb.low_2, bb.low_3);
		PointType max (bb.high_1, bb.high_2, bb.high_3);
		out_min = min;
		out_max = max;		
	
		width = fabs(max.x-min.x);
		height = fabs(max.y-min.y);
		depth = fabs(max.z-min.z);

		gdiam_point_t centre;
		bb.get_centre (centre);
		translation = Eigen::Vector3f (centre[0], centre[1], centre[2]);
	
	/*
		Eigen::Vector4f centre;
		pcl::compute3DCentroid (*cloud, centre);
		translation = Eigen::Vector3f (centre.x(), centre.y(), centre.z());
	*/

		Eigen::Matrix3f rot;
		rot(0, 0) = bb.dir_1[0];		rot(0, 1) = bb.dir_2[0];		rot(0, 2) = bb.dir_3[0];
		rot(1, 0) = bb.dir_1[1];		rot(1, 1) = bb.dir_2[1];		rot(1, 2) = bb.dir_3[1];
		rot(2, 0) = bb.dir_1[2];		rot(2, 1) = bb.dir_2[2];		rot(2, 2) = bb.dir_3[2];
		quaternion = Eigen::Quaternionf (rot);

		//cout << "W: " << width << " H: " << height << " D: " << depth << endl;
		//cout << "Translation: " << translation.x() << " " << translation.y() << " " << translation.z() << endl;
		//cout << "Rotation: " << quaternion.w() << " " << quaternion.x() << " " << quaternion.y() << " " << quaternion.z() << endl;
	}
	else
		cout << "Couldn't compute bounding box for some reason" << endl;
}

void PlanDetector::project_on_plan (Cloud::ConstPtr in, Cloud::Ptr &out)
{
	int cloud_id = get_closest_plan_id (in);
	cout << "Projecting on plan " << cloud_id << endl;
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (in);
  proj.setModelCoefficients (m_vec_coefs[cloud_id]);
  proj.filter (*out);
}

int PlanDetector::get_closest_plan_id (Cloud::ConstPtr cloud)
{
	PointType min;
	PointType max;
	pcl::getMinMax3D (*cloud, min, max);
	
	int min_plan_id = -1;
	double min_dist = 1000.0;
	double tmp = 0.0;
	for (size_t i=0; i<m_vec_plans.size (); ++i)
	{
		tmp = distance_to_plan (min, m_vec_coefs[i]);
		cout << "Distance to plan " << i << " = " << tmp << endl;
		if ( tmp < min_dist )
		{
			min_dist = tmp;
			min_plan_id = i;
		}
	}
	
	return min_plan_id;
}

double PlanDetector::distance_to_plan(PointType p, pcl::ModelCoefficients::ConstPtr c)
{
	return fabs(c->values[0]*p.x + c->values[1]*p.y + c->values[2]*p.z + c->values[3])/sqrt (c->values[0]*c->values[0]+c->values[1]*c->values[1]+c->values[2]*c->values[2]);
}

void PlanDetector::pcl2gdiam (Cloud::ConstPtr cloud, gdiam_real* points)
{
	for (size_t i=0; i<cloud->size (); ++i)
	{
		points[ 3 * i ] = cloud->points[i].x;
		points[ 3 * i + 1 ] = cloud->points[i].y;
		points[ 3 * i + 2 ] = cloud->points[i].z;		
	}
}

int PlanDetector::num_plans ()
{	return m_vec_plans.size (); }

Cloud::ConstPtr PlanDetector::get_plan (int i)
{	return m_vec_plans[i]; }

std::vector< Cloud::ConstPtr > PlanDetector::get_plans ()
{	return m_vec_plans; }

Cloud::ConstPtr PlanDetector::get_filtered_cloud ()
{	return m_cloud_filtered; }

Cloud::ConstPtr PlanDetector::get_cluster(int i)
{	return m_cloud_cluster[i]; }

std::vector< Cloud::ConstPtr > PlanDetector::get_clusters()
{	return m_cloud_cluster; }

PlanDetector::PlanDetector () :		m_inliers (new pcl::PointIndices),
																	m_tmp (new pcl::PointCloud<PointType>),
																	m_cloud_filtered (new pcl::PointCloud<PointType>),
																	m_tree (new pcl::search::KdTree<PointType>)
{
	m_seg.setOptimizeCoefficients (true);
	m_seg.setModelType (pcl::SACMODEL_PLANE);
  m_seg.setMethodType (pcl::SAC_RANSAC);
  m_seg.setMaxIterations (1000);
  m_seg.setDistanceThreshold (0.01);
  
  m_vox.setLeafSize (0.01f, 0.01f, 0.01f);
  //m_vox.setLeafSize (0.05f, 0.05f, 0.05f);
  
  m_ec.setClusterTolerance (0.02); // 2cm
  m_ec.setMinClusterSize (100);
  m_ec.setMaxClusterSize (25000);
  m_ec.setSearchMethod (m_tree);
}

PlanDetector::~PlanDetector ()
{}

  

  
