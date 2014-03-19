#include "PlanDetector.h"

using namespace std;
using namespace cv;

PlanDetector::PlanDetector () :		m_tree (new pcl::search::KdTree<PointType>),
																	m_cloud_filtered (new pcl::PointCloud<PointType>)
{
	// RANSAC segmentation parameters
	m_seg.setOptimizeCoefficients (true);
	m_seg.setModelType (pcl::SACMODEL_PLANE);
  m_seg.setMethodType (pcl::SAC_RANSAC);
  m_seg.setMaxIterations (1000);
  // Max distance for a point belonging to a plan
  m_max_distance_to_plan = 0.01;
  m_seg.setDistanceThreshold (m_max_distance_to_plan);
	// Subsampling for faster overall processing
  m_vox.setLeafSize (0.01f, 0.01f, 0.01f);
  //m_vox.setLeafSize (0.05f, 0.05f, 0.05f);
  
  // Max euclidean distance between two objects.
  m_ec.setClusterTolerance (0.05); // 5cm
  m_ec.setMinClusterSize (250);
  m_ec.setMaxClusterSize (25000);
  m_ec.setSearchMethod (m_tree);
  
  m_plans_segmented = false;
}

PlanDetector::~PlanDetector ()
{}

int PlanDetector::segment_plans (Cloud::ConstPtr cloud)
{
	cout << "Segmenting plans..." << endl;
	// clear saved data from previous iteration
	m_vec_coefs.clear ();
	m_vec_plans.clear ();
  
  // downsampling with a voxelgrid (disabled for now)
  //*m_cloud_filtered = *cloud;
  m_vox.setInputCloud (cloud);
  m_vox.filter (*m_cloud_filtered);
  int initial_cloud_size = (int) m_cloud_filtered->points.size ();
  // Plan segmentation
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  Cloud::Ptr tmp(new Cloud);
  double distance = 0;
  double min_distance = 1000;
  int loop_counter = 0;
	while (m_cloud_filtered->points.size () > 0.3 * initial_cloud_size)
	{
		Cloud::Ptr plan (new Cloud);
		pcl::ModelCoefficients::Ptr coefs (new pcl::ModelCoefficients);
		
		m_seg.setInputCloud (m_cloud_filtered);
	  m_seg.segment (*inliers, *coefs);
		if (inliers->indices.size () == 0) {
  	  PCL_ERROR ("Could not estimate a planar model for the given dataset.");
  	  return (-1);
  	}
    // Extract the inliers
    m_extract.setInputCloud (m_cloud_filtered);
    m_extract.setIndices (inliers);
    m_extract.setNegative (false);
    m_extract.filter (*plan);
		//cout << plan->size() << endl;
    m_extract.setNegative (true);
    m_extract.filter (*tmp);
    m_cloud_filtered.swap (tmp);
    // saving data
		m_vec_coefs.push_back (coefs);
		m_vec_plans.push_back (plan);
		// test if this plan is the prefered plan
		distance = min_distance_to_z_axis(plan);
		if (distance < min_distance) {
			min_distance = distance;
			m_prefered_plan_idx = loop_counter;
		}
		++loop_counter;
	}
	std::cout << "segmented " << m_vec_coefs.size () << " plans (prefered: " << m_prefered_plan_idx << ")." << std::endl;
	
	m_plans_segmented = true;
	compute_prefered_plan_pose ();
	return m_vec_coefs.size ();
}

int PlanDetector::segment_clusters (Cloud::ConstPtr cloud) {

	cout << "Segmenting clusters..." << endl;
	if (m_plans_segmented) {
		// only keep points in a cropbox above current interest plan.
		Cloud::Ptr no_plans_cloud (new Cloud);
		Cloud::Ptr prefered_cloud (new Cloud);
		//cropbox_prefered_plan (m_cloud_filtered, 0.5, prefered_cloud);
		remove_plans (cloud, no_plans_cloud);
		cropbox_prefered_plan (no_plans_cloud, 0.5, prefered_cloud);
		// segment prefered_cloud
		//cout << "Keeping " << no_plans_cloud->size() << " points from non-plans." << endl;
		//cout << "Keeping " << prefered_cloud->size() << "/" << no_plans_cloud->size() << " points." << endl;
		if ( prefered_cloud->points.size() != 0 )
			segment_cloud (prefered_cloud);
		else
			cout << "no clusters on the prefered cloud" << endl;
	}
	cout << "segmented " << m_cloud_cluster.size() << " clusters." << endl;
	return m_cloud_cluster.size();
}

int PlanDetector::segment_clusters_all ()
{
	cout << "Segmenting clusters..." << endl;
	if (m_cloud_filtered->width && m_cloud_filtered->height)
		return segment_cloud (m_cloud_filtered);
	else
	{
		std::cout << "Error: PlanDetector::segment: No filtered cloud present." << std::endl;
		return (0);
	}
}

int PlanDetector::segment_cloud (Cloud::ConstPtr cloud)
{
	m_cloud_cluster.clear ();
	m_cluster_indices.clear ();

  m_tree->setInputCloud (cloud);
  m_ec.setInputCloud (cloud);
  m_ec.extract (m_cluster_indices);

  for (std::vector<pcl::PointIndices>::const_iterator it = m_cluster_indices.begin (); it != m_cluster_indices.end (); ++it)
  {
	  Cloud::Ptr cluster (new Cloud);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
      cluster->points.push_back (cloud->points[*pit]);
    cluster->width = cluster->points.size ();
    cluster->height = 1;
    cluster->is_dense = true;
    m_cloud_cluster.push_back (cluster);
  }
	//std::cout << "Segmented " << m_cloud_cluster.size () << " clusters." << std::endl;
	return m_cloud_cluster.size ();
}

int PlanDetector::segment_rgb (cv::Mat in_img)
{
	cout << "Segmenting rgb clusters..." << endl;
	m_rgb_images.clear();
	for (size_t k = 0; k < m_cloud_cluster.size(); ++k) {
		Mat* cluster_img = get_cluster_picture (m_cloud_cluster[k], in_img);
	  m_rgb_images.push_back (cluster_img);
  }
  int num_img = m_rgb_images.size ();
  cout << "done." << endl;
  return num_img;
}

cv::Mat* PlanDetector::get_cluster_picture (Cloud::ConstPtr cloud, cv::Mat img) {
	Rect roi = get_roi (cloud);
	//cout << roi << endl;
	Mat* picture = new Mat(roi.width, roi.height, CV_8UC3);
	*picture = img(roi);
	return picture;
}

cv::Rect PlanDetector::get_roi (Cloud::ConstPtr cloud) {
	int i, j;
	int minI = 1000, minJ = 1000;
	int maxI = 0, maxJ = 0;
	for (size_t n=0; n<cloud->size(); ++n) {
		reproj(cloud->points[n], i, j);
		minI = min(i, minI);  minJ = min(j, minJ);
		maxI = max(i, maxI);  maxJ = max(j, maxJ);
	}
	//Rect roi (minJ, minI, maxJ-minJ, maxI-minI);
	Rect roi (minI, minJ, maxI-minI, maxJ-minJ);
	return roi;
}

void PlanDetector::reproj(PointType p, int &i, int &j) {
	i = 525.0*p.x/p.z + 319.5;
	j = 525.0*p.y/p.z + 239.5;
	if (i <0 || j<0) {
		cout << i << " " << j << endl;
		cout << p.x << " " << p.y << " " << p.z << endl;
	}
}

double PlanDetector::min_distance_to_z_axis (Cloud::ConstPtr plan) {
	double distance = 0;
	double min_distance = 1000;
	for (size_t i=0; i<plan->points.size(); ++i) {
		distance = plan->points[i].x*plan->points[i].x 
						 + plan->points[i].y*plan->points[i].y;
		if (distance < min_distance)
			min_distance = distance;
	}
	return min_distance;
}

void PlanDetector::attention_cylinder (Cloud::ConstPtr in, float thresh,
																			 Cloud::Ptr out) {
	float a, b, c, d;
	float x, y, z;
	a = m_vec_coefs[m_prefered_plan_idx]->values[0];
	b = m_vec_coefs[m_prefered_plan_idx]->values[1];
	c = m_vec_coefs[m_prefered_plan_idx]->values[2];
	d = m_vec_coefs[m_prefered_plan_idx]->values[3];
	cout << a << " " << b << " " << c << " " << d << endl;
	for (size_t i=0; i<in->points.size (); ++i) {
		x = in->points[i].x;  y = in->points[i].y;  z = in->points[i].z;
		if ( pcl_isfinite(x) && pcl_isfinite(y) && pcl_isfinite(z) ) {
			// d signe is position of camera/plan (above or below)
			if (d < 0) {
				if (a*x+b*y+c*z+d < 0 && sqrt(x*x + y*y) < thresh)
					out->points.push_back (PointType(x, y, z));
			}
			else if (d > 0) {
				if (a*x+b*y+c*z+d > 0 && sqrt(x*x + y*y) < thresh)
					out->points.push_back (PointType(x, y, z));
			}
		}
	}
}

void PlanDetector::remove_plans (Cloud::ConstPtr in, Cloud::Ptr out) {
	size_t count = 0;
	for (size_t i=0; i<in->size(); ++i) {
		for (size_t k=0; k<m_vec_coefs.size(); ++k) {
			if (fabs(in->points[i].x * m_vec_coefs[k]->values[0]
							+ in->points[i].y * m_vec_coefs[k]->values[1]
							+ in->points[i].z * m_vec_coefs[k]->values[2]
							+ m_vec_coefs[k]->values[3]) >= m_max_distance_to_plan)
				++count;
		}
		if (count == m_vec_coefs.size())
			out->points.push_back (PointType(in->points[i].x, in->points[i].y, in->points[i].z));
		count = 0;
	}
}

void PlanDetector::compute_prefered_plan_pose () {
	compute_plan_pose (m_vec_plans[m_prefered_plan_idx],
										 m_vec_coefs[m_prefered_plan_idx],
										 m_prefered_plan_translation, m_prefered_plan_orientation,
										 m_prefered_plan_min, m_prefered_plan_max,
										 m_prefered_plan_rotation_matrix);
}

void PlanDetector::compute_plan_pose (Cloud::ConstPtr cloud, pcl::ModelCoefficients::ConstPtr coefs,
																			Eigen::Vector3f &translation, Eigen::Vector3f &orientation,
																			Eigen::Vector4f &min, Eigen::Vector4f &max, Eigen::Matrix3f &rot_mat) {
	double a, b, c, d;
	a = coefs->values[0];
	b = coefs->values[1];
	c = coefs->values[2];
	d = coefs->values[3];
	//cout << a << " " << b << " " << c << " " << d << endl;
	// Compute rotation
	Eigen::Vector3f n;
	Eigen::Vector3f u;
	if (d > 0) {
		n << a, b, c;		u << -b, a, 0;
	} else {
		n << -a, -b, -c;		u << b, -a, 0;
	}
	n.normalize();
	u.normalize();
	Eigen::Vector3f v = n.cross(u);	v.normalize();
	rot_mat << u(0), v(0), n(0), u(1), v(1), n(1), u(2), v(2), n(2);
	//cout << R << endl;
	orientation = rot_mat.eulerAngles(0, 1, 2);
	//cout << "Orientation :" << endl << orientation << endl;
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid (*cloud, centroid);
	translation = Eigen::Vector3f(centroid(0), centroid(1), centroid(2));
	//cout << "Translation :" << endl << translation << endl;
	Eigen::Affine3f t;
	pcl::getTransformation (translation(0), translation(1), translation(2),
													orientation(0),	orientation(1), orientation(2), t);
	// Look for min and max in plane frame
	Cloud::Ptr tmp (new Cloud);
	pcl::transformPointCloud (*cloud, *tmp, t.inverse());
	pcl::getMinMax3D	(*tmp, min, max);
}

void PlanDetector::cropbox_prefered_plan (Cloud::ConstPtr in, float offset, Cloud::Ptr out) {
	Eigen::Vector4f max, min;
	min = m_prefered_plan_min;
	max = m_prefered_plan_max;
	min(2) = 0.01;  // start cropbox at z=mid of plan (centroid) + 1cm
	max(2) += offset; // end cropbox at z=top of plan + offset
	//cout << min << endl << max << endl;
	// Crop with cropbox
	pcl::CropBox< PointType > cropbox;
	cropbox.setInputCloud(in);
	cropbox.setMin(min);
	cropbox.setMax(max);
	cropbox.setRotation(m_prefered_plan_orientation);
	cropbox.setTranslation(m_prefered_plan_translation);
	cropbox.filter(*out);
}

Cloud::ConstPtr PlanDetector::get_prefered_plan () {
		return m_vec_plans[m_prefered_plan_idx];
}

Cloud::ConstPtr PlanDetector::get_plan (size_t i) {
	if (i < m_vec_plans.size() ) {
		return m_vec_plans[i];
	}	else {
		cout << "Plan " << i << " doesn't exists." << endl;
		return m_vec_plans[0];
	}
}

std::vector< Cloud::ConstPtr > PlanDetector::get_plans () {
	return m_vec_plans; 
}

Cloud::ConstPtr PlanDetector::get_cluster(size_t i) {
	if (i < m_cloud_cluster.size() ) {
		return m_cloud_cluster[i]; 
	}	else {
		cout << "Cluster " << i << " doesn't exists." << endl;
		return m_cloud_cluster[0];
	}
}

std::vector<Cloud::ConstPtr> PlanDetector::get_clusters() {
	return m_cloud_cluster;
}

Mat* PlanDetector::get_rgb_cluster(size_t i) {
	if (i < m_rgb_images.size() ) {
		return m_rgb_images[i];
	}	else {
		cout << "Image " << i << " doesn't exists." << endl;
		return m_rgb_images[0];
	}
}

vector<Mat*> PlanDetector::get_rgb_clusters() {
	return m_rgb_images;
}

Cloud::ConstPtr PlanDetector::get_filtered_cloud () {
	return m_cloud_filtered;
}

size_t PlanDetector::get_num_clusters() {
	return m_cloud_cluster.size ();
}

size_t PlanDetector::get_num_plans()	{
	return m_vec_plans.size ();
}

size_t PlanDetector::get_num_rgbs()	{
	return m_rgb_images.size ();
}

Eigen::Vector3f PlanDetector::get_prefered_plan_translation () {
	return m_prefered_plan_translation;
}

Eigen::Quaternionf PlanDetector::get_prefered_plan_orientation () {
	return Eigen::Quaternionf (m_prefered_plan_rotation_matrix);
}

Eigen::Vector4f PlanDetector::get_prefered_plan_min () {
	return m_prefered_plan_min;
}

Eigen::Vector4f PlanDetector::get_prefered_plan_max () {
	return m_prefered_plan_max;
}
