#include "OctreeProcessor.h"

using namespace std;
using namespace octomap;
using namespace cv;

OctreeProcessor::OctreeProcessor (): cloud(new pcl::PointCloud<pcl::PointXYZ>) {
	tree = new OcTree(1.0); // dummy resolution. Will be overwritten by load_tree.
}

void OctreeProcessor::load_tree (const char* path) {
	tree->readBinary(path);
  maxDepth_ = tree->getTreeDepth ();
  resolution_ = tree->getResolution (); // octree resolution in meters
  cout << "Loaded tree with depth " << maxDepth_
  		 << " and resolution " << resolution_ << endl;
	// Give up the multiresolution, all cells will have 0.05 size
	tree->expand();
	// Compute min and max once and for all
	double x, y, z;
	tree->getMetricMin (x, y, z);
	min_ = point3d(x, y, z);
	tree->getMetricMax (x, y, z);
	max_ = point3d(x, y, z);
	cout << "Min : " << min_ << " "
			 << "Max : " << max_ << endl;
	voxel_grid_size_ = point_to_voxel_coord (max_, resolution_, min_);
	cout << "Voxel grid size : " << voxel_grid_size_ << endl;
}

void OctreeProcessor::update_size () {
	// Compute min and max once and for all
	double x, y, z;
	tree->getMetricMin (x, y, z);
	min_ = point3d(x, y, z);
	tree->getMetricMax (x, y, z);
	max_ = point3d(x, y, z);
	cout << "New min : " << min_ << " "
			 << "New max : " << max_ << endl;
	voxel_grid_size_ = point_to_voxel_coord (max_, resolution_, min_);
}

vector< vector<point3d> > OctreeProcessor::get_interest_clouds () {
	// CLEANING
	octomap::point3d min (0.0, 0.0, 0.0), max (10.0, 8.0, 2.2);
	std::vector<octomap::point3d> out_of_bounds = get_out_of_bounds (min, max);
	remove (out_of_bounds);

	std::vector<unsigned int> hist = compute_histogram_z ();
	std::vector<octomap::point3d> floor = get_floor (hist);
	remove (floor);
	
	std::vector<octomap::point3d> walls = get_walls (hist);
	remove (walls);
	
	std::vector<octomap::point3d> noise = get_noise ();
	remove (noise);
	// ACTUAL PROCESSING
	vector< vector<point3d> > interest_clusters;
	vector<point3d> cluster;
	for (unsigned int i=0; i<7; ++i) {
		hist = compute_histogram_z ();
		for (size_t i = 0; i < hist.size(); ++i)
			cout << hist[i] << endl;
		cluster = get_max_hist_cluster (hist);
		interest_clusters.push_back (cluster);
		remove (cluster);
	}
	return interest_clusters;
}

vector<point3d> OctreeProcessor::get_max_hist_cluster (vector<unsigned int> hist) {
	unsigned int max_bin = std::max_element(hist.begin(), hist.end()) - hist.begin ();
	std::vector<octomap::point3d> slice_z = get_slice_z (max_bin);
	vector<double> x, y;
	for (size_t i = 0; i < slice_z.size(); ++i) {
		x.push_back (slice_z[i].x());
		y.push_back (slice_z[i].y());
	}
	return get_slice_xy (x, y);
}

vector<Mat>	OctreeProcessor::get_interest_areas () {
	vector<Mat> slices;
	
	octomap::point3d min (0.0, 0.0, 0.0), max (10.0, 8.0, 2.2);
	std::vector<octomap::point3d> out_of_bounds = get_out_of_bounds (min, max);
	remove (out_of_bounds);

	std::vector<unsigned int> hist = compute_histogram_z ();
	std::vector<octomap::point3d> floor = get_floor (hist);
	remove (floor);
	
	std::vector<octomap::point3d> walls = get_walls (hist);
	remove (walls);
	
	std::vector<octomap::point3d> noise = get_noise ();
	remove (noise);
	
	// compute new histogram	
	hist = compute_histogram_z ();
	for (size_t i = 0; i < hist.size(); ++i) {
		cout << hist[i] << endl;
	}
	std::vector<unsigned int> maxs = find_max_bins (hist);
	for (size_t i = 0; i < maxs.size(); ++i) {
		std::vector<octomap::point3d> slice = get_slice_z (maxs[i]);
		cv::Mat proj = project_slices_2d (slice);
		slices.push_back(proj);
	}
	cout << "Found " << slices.size () << " slices." << endl;
	for (size_t i = 0; i < slices.size(); ++i) {
		cout << maxs[i]*resolution_ << " ";
	}
	cout << endl;
	
	return slices;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr OctreeProcessor::cloud_to_pcl (vector<point3d> cloud) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	for (size_t i = 0; i < cloud.size (); ++i) {
		pcl_cloud->points.push_back (pcl::PointXYZ (cloud[i].x(),
																								cloud[i].y(),
																								cloud[i].z()));
	}
	return pcl_cloud;
}

vector<unsigned int> OctreeProcessor::compute_histogram_z () {
	unsigned int num_bins = (max_.z() - min_.z())/resolution_;
  cout << "Histogram has " << num_bins << " bins." << endl;
	vector<unsigned int> hist (num_bins);
	for(OcTree::iterator it = tree->begin(maxDepth_), end=tree->end(); it!= end; ++it) {
		if (tree->isNodeOccupied(*it)) {
			double z = it.getCoordinate().z();
			unsigned int bin = floor(z/resolution_);
	 		hist[bin] += 1;
		}
  }
/*  
  for (size_t i = 0; i < hist.size(); ++i) {
  	cout << hist[i] << endl;
  }
*/
  return hist;
}

void OctreeProcessor::remove (vector<point3d> points) {
	cout << "Removing " << points.size () << " points." << endl;
	for (size_t i = 0; i < points.size(); ++i) {
		tree->updateNode (points[i], octomap::logodds(0.0f));
	}
	tree->updateInnerOccupancy ();
	
	//update_size ();
}

std::vector<octomap::point3d> OctreeProcessor::get_whole_cloud () {

	vector<point3d> cloud;
	for(OcTree::iterator it = tree->begin(maxDepth_), end=tree->end(); it!= end; ++it) {
		if (tree->isNodeOccupied(*it)) {
			cloud.push_back (it.getCoordinate());
		}
  }
  cout << "Whole cloud has " << cloud.size() << " points." << endl;
  return cloud;
}

std::vector<octomap::point3d> OctreeProcessor::get_floor (std::vector<unsigned int> hist) {
	// getting the bin number = index in the vector
	unsigned int max_bin = std::max_element(hist.begin(), hist.end()) - hist.begin ();
	vector<point3d> slice_z = get_slice_z (max_bin);
	cout << "Floor bin : " << max_bin << ", with " << hist[max_bin] <<" elements." << endl;
	
	return slice_z;
}

vector<point3d> OctreeProcessor::get_walls (vector<unsigned int> hist) {
	// getting the bin number = index in the vector
	min_bin_ = std::min_element(hist.begin(), hist.end()) - hist.begin ();
	//unsigned int min_bin = hist.size()-1; // just before the ceiling
	cout << "Walls bin : " << min_bin_ << ", with " << hist[min_bin_] <<" elements." << endl;
	
	vector<point3d> slice_z = get_slice_z (min_bin_);
	//double min_z = 1.9, max_z = 2.2;
	//vector<point3d> slice_z = get_slice_z (min_z, max_z);
	vector<double> x, y;
	for (size_t i = 0; i < slice_z.size(); ++i) {
		x.push_back (slice_z[i].x());
		y.push_back (slice_z[i].y());
	}
	return get_slice_xy (x, y);
}

vector<point3d> OctreeProcessor::get_noise () {
	vector<point3d> noise;
	for(OcTree::iterator it = tree->begin(maxDepth_), end=tree->end(); it!= end; ++it) {
		if (tree->isNodeOccupied(*it)) {
			point3d pt = it.getCoordinate ();
			if (!star_test(pt))
				noise.push_back (pt);
		}
  }
  return noise;
}

vector<point3d> OctreeProcessor::get_noise (std::vector<octomap::point3d> points) {
	vector<point3d> noise;
	for (size_t i = 0; i < points.size(); ++i) {
		if (!star_test(points[i]))
			noise.push_back (points[i]);
	}
  return noise;
}

vector<point3d> OctreeProcessor::get_out_of_bounds (point3d min, point3d max) {
	vector<point3d> out;
	for(OcTree::iterator it = tree->begin(maxDepth_), end=tree->end(); it!= end; ++it) {
		if (tree->isNodeOccupied(*it)) {
			point3d pt = it.getCoordinate ();
			if (pt.x() < min.x()
				|| pt.y() < min.y()
				|| pt.z() < min.z()
				|| pt.x() > max.x()
				|| pt.y() > max.y()
				|| pt.z() > max.z())
				out.push_back (pt);
		}
  }
  return out;
}

vector<point3d> OctreeProcessor::get_slice_xy (vector<double> x, vector<double> y) {
	vector<point3d> slice;
  for (size_t i=0; i < x.size (); ++i) {
  	for (double z=min_.z()+resolution_/2; z < max_.z(); z+=resolution_) {
  		//cout << x[i] << " " << y[i] << " " << z << endl;
  		OcTreeNode* node = tree->search(x[i], y[i], z);
  		if (node != NULL) {
	  		if (tree->isNodeOccupied(node)) {
	  			/*
	  			for (unsigned int k = 0; k < 5; ++k) {
	  				for (unsigned int l = 0; l < 5; ++l) {
	  					slice.push_back (point3d(x[i]+k*resolution_, y[i]+l*resolution_, z));
	  				}
	  			}
	  			*/
	  			slice.push_back (point3d(x[i], y[i], z));
  			}
  		}
  	}
  }  
  return slice;
}

vector<point3d> OctreeProcessor::get_slice_z (unsigned int bin) {
	double min_z, max_z;
	bin_to_z (bin, resolution_, min_z, max_z);
	return get_slice_z (min_z, max_z);
}

vector<point3d> OctreeProcessor::get_slice_z (unsigned int min_bin, unsigned int max_bin) {
	double min_z, max_z, tmp;
	bin_to_z (min_bin, resolution_, min_z, tmp);
	bin_to_z (max_bin, resolution_, tmp, max_z);
	return get_slice_z (min_z, max_z);
}

vector<point3d> OctreeProcessor::get_slice_z (double min_z, double max_z) {
	vector<point3d> slice;
	for(OcTree::iterator it = tree->begin(maxDepth_), end=tree->end(); it!= end; ++it) {
		if (tree->isNodeOccupied(*it)) {
			point3d pt = it.getCoordinate ();
			if (min_z <= pt.z() && pt.z() <= max_z)
				slice.push_back (pt);
		}
  }
  return slice;
}

unsigned int OctreeProcessor::num_occupied_above (vector<point3d> slice) {
	unsigned int counter = 0;
	for (size_t i=0; i < slice.size(); ++i) {
		OcTreeNode* node = tree->search(slice[i].x(),
																		slice[i].y(),
																		slice[i].z() + resolution_);
		if (node == NULL || tree->isNodeOccupied(node)) {
			++counter;
		}
	}
	return counter;
}

unsigned int OctreeProcessor::num_free_above (vector<point3d> slice) {
	unsigned int counter = 0;
	for (size_t i=0; i < slice.size(); ++i) {
		OcTreeNode* node = tree->search(slice[i].x(),
																		slice[i].y(),
																		slice[i].z() + resolution_);
		if (node != NULL && !tree->isNodeOccupied(node)) {
			++counter;
		}
	}
	return counter;
}

bool OctreeProcessor::is_free_above (point3d pt, unsigned int num_free_voxels) {
	for (unsigned int i=1; i < num_free_voxels+1; ++i) {
		//cout << pt.x() << " " << pt.y() << " " << z << endl;
		OcTreeNode* node = tree->search(pt.x(), pt.y(), pt.z() + i*resolution_);
		if (node == NULL || tree->isNodeOccupied(node)) {
			return false;
		}
	}
	return true;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr OctreeProcessor::tree_to_pointcloud () {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	for(OcTree::iterator it = tree->begin(maxDepth_), end=tree->end(); it!= end; ++it) {
		if (tree->isNodeOccupied(*it)) {
			point3d pt = it.getCoordinate();
			cloud->points.push_back (pcl::PointXYZ (pt.x(), pt.y(), pt.z()));
		}
  }
  return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr OctreeProcessor::slices_to_pointcloud (vector<Mat> slices) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	for (size_t z = 0; z < slices.size(); ++z) {
		for (int x = 0; x < slices[z].cols; ++x) {
			for (int y = 0; y < slices[z].rows; ++y) {
				if (slices[z].at<unsigned char>(x, y) == 255) {
					//point3d pt = voxel_coord_to_point (point3d(x, y, z), resolution_, min_);
					//cloud->points.push_back (pcl::PointXYZ (pt.x(), pt.y(), pt.z()));
					cloud->points.push_back (pcl::PointXYZ (x*resolution_, y*resolution_, z*resolution_));
				}
			}
		}
	}
	return cloud;
}

void OctreeProcessor::pointcloud_to_tree (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

}

vector<unsigned int> OctreeProcessor::find_max_bins (vector<unsigned int> hist) {
	vector<unsigned int> maxs;
	for (size_t i = 1; i < hist.size()-1; ++i) {
		if ( (hist[i-1] < hist[i]) && hist[i] > hist[i+1])
			maxs.push_back (i);
	}
	
	cout << "Maxs size : " << maxs.size() << endl;
	for (size_t i = 0; i < maxs.size (); ++i) {
		cout << maxs[i] << " ";
	}
	cout << endl;
	return maxs;
}

vector<point3d> OctreeProcessor::get_open_areas (unsigned int num_free_voxels) {
	vector<point3d> points;
	for(OcTree::iterator it = tree->begin(maxDepth_), end=tree->end(); it!= end; ++it) {
		if (tree->isNodeOccupied(*it)) {
			point3d pt = it.getCoordinate ();
			if (is_free_above(pt, num_free_voxels)) {
				points.push_back (pt);
			}
		}
  }
  cout << "Found " << points.size () << " points out there in the open." << endl;
  return points;
}

vector<point3d> OctreeProcessor::get_non_open_areas (unsigned int num_free_voxels) {
	vector<point3d> points;
	for(OcTree::iterator it = tree->begin(maxDepth_), end=tree->end(); it!= end; ++it) {
		if (tree->isNodeOccupied(*it)) {
			point3d pt = it.getCoordinate ();
			if (!is_free_above(pt, num_free_voxels)) {
				points.push_back (pt);
			}
		}
  }
  cout << "Found " << points.size () << " points out there non open." << endl;
  return points;
}

Mat OctreeProcessor::project_slices_2d (vector<point3d> slice) {
	Mat proj = Mat::zeros (voxel_grid_size_.y(), voxel_grid_size_.x(), CV_8UC1);
	for (size_t i = 0; i < slice.size(); ++i) {
		point3d coord = point_to_voxel_coord (slice[i], resolution_, min_);
		proj.at<unsigned char>(coord.y(), coord.x()) = 255;
	}
	return proj;
}

Mat OctreeProcessor::get_slice_2d (double z) {
	Mat slice = Mat::zeros (voxel_grid_size_.y(), voxel_grid_size_.x(), CV_8UC1);
	for(OcTree::iterator it = tree->begin(maxDepth_), end=tree->end(); it!= end; ++it) {
		if (tree->isNodeOccupied(*it)) {
			point3d pt = it.getCoordinate ();
			//cout << z << " " << pt.z() << endl;
			if (z == pt.z()) {
				//cout << z << " " << pt.z() << endl;
				point3d coord = point_to_voxel_coord (pt, resolution_, min_);
				slice.at<unsigned char>(coord.y(), coord.x()) = 255;
			}
		}
  }
  return slice;
}

vector<Mat> OctreeProcessor::get_slice_2d_all () {
	vector<Mat> slices;
	for (unsigned int i = 0; i < voxel_grid_size_.z(); ++i) {
		//cout << 0.025+i*resolution_ << endl;
		slices.push_back (get_slice_2d (0.025+i*resolution_));
	}
	return slices;
}

bool OctreeProcessor::is_occupied_or_null (double x, double y, double z) {
	OcTreeNode* node = tree->search(x, y, z);
	if (node == NULL || tree->isNodeOccupied(node))
		return true;
	return false;
}

bool OctreeProcessor::is_occupied (point3d pt) {
	return is_occupied (pt.x(), pt.y(), pt.z());
}

bool OctreeProcessor::is_occupied (double x, double y, double z) {
	//cout << x << " " << y << " " << z << endl;
	OcTreeNode* node = tree->search(x, y, z);
	if (node != NULL && tree->isNodeOccupied(node))
		return true;
	return false;
}

bool OctreeProcessor::star_test (point3d pt) {
	double x = pt.x(), y = pt.y(), z = pt.z(), r=resolution_;
	if (//&& is_occupied_or_null (x, y, z+r)
			//&& is_occupied_or_null (x, y, z-r)
			 is_occupied_or_null (x, y+r, z)
			&& is_occupied_or_null (x, y-r, z)
			&& is_occupied_or_null (x+r, y, z)
			&& is_occupied_or_null (x-r, y, z))
		return true;

	return false;
}

void OctreeProcessor::grow_cloud (vector<point3d> &in) {
	double r=resolution_;
	for (size_t i = 0; i < in.size(); ++i) {
		double x = in[i].x(), y = in[i].y(), z = in[i].z();
		if (is_occupied (x, y, z+r)) {
			in.push_back (point3d(x, y, z+r));
			grow_cloud (in);
		}
		if (is_occupied (x, y, z-r)) {
			in.push_back (point3d(x, y, z-r));
			grow_cloud (in);
		}
		if (is_occupied (x, y+r, z)) {
			in.push_back (point3d(x, y+r, z));
			grow_cloud (in);
		}
		if (is_occupied (x, y-r, z)) {
			in.push_back (point3d(x, y-r, z));
			grow_cloud (in);
		}
		if (is_occupied (x+r, y, z)) {
			in.push_back (point3d(x+r, y, z));
			grow_cloud (in);
		}
		if (is_occupied (x-r, y, z)) {
			in.push_back (point3d(x-r, y, z));
			grow_cloud (in);
		}
	}
}

// Takes as input a set of segments supposed to be in the walls
Mat OctreeProcessor::find_clusters (vector<Point> walls_inliers) {
	vector<Point> under_over_points;
	Mat res = Mat::zeros (voxel_grid_size_.x(), voxel_grid_size_.x(), CV_8UC1);
	cout << walls_inliers.size() << endl;
	for (size_t i = 0; i < walls_inliers.size(); ++i) {
		Point voxel = walls_inliers[i];
		unsigned int under = 0, over = 0;
		get_num_occupied_cells (voxel.x, voxel.y, under, over);
		under_over_points.push_back (Point(under, over));
		//res.at<unsigned char>(under, over) = 255;
		//cout << "Coords: " << voxel.x << " " << voxel.y << endl;
		//cout << "Nums: " << under << " " << over << endl;
	}
	
	Mat data (under_over_points.size(), 2, CV_32F);
	for (int i = 0; i < data.rows; ++i) {
		data.at<float>(i, 0) = under_over_points[i].x;
		data.at<float>(i, 1) = under_over_points[i].y;
	}
	int num_classes = 5;
	Mat labels;
	TermCriteria crit (CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 1000, 0.0001);
	int attempts = 5;
	Mat centers;
	kmeans(data, num_classes, labels, crit, attempts, KMEANS_PP_CENTERS, centers);

	return centers;
}

void OctreeProcessor::classify (vector<Point> points, Mat centers, Mat &labels) {
	labels = Mat::zeros (points.size(), 1, CV_8UC1);
	//vector<Point> under_over_vec;
	for (size_t i = 0; i < points.size(); ++i) {
		Point under_over;
		under_over = get_num_occupied_cells (points[i]);
		//cout << under_over << endl;
		//under_over_vec.push_back (under_over);
		labels.at<unsigned char>(i) = get_label (under_over, centers);
	}
	//get_labels (under_over_vec, centers, labels);
}
/*
void OctreeProcessor::get_labels (vector<Point> under_over, Mat centers,
																Mat &labels) {
	labels = Mat::zeros (under_over.size(), 1, CV_8UC1);
	for (size_t i = 0; i < under_over.size(); ++i) {
		labels.at<unsigned char>(i) = get_label (under_over[i], centers);
	}
}
*/
int OctreeProcessor::get_label (Point pt, Mat centers) {
	int idx = -1;
	vector<double> dist;
	for (unsigned int i = 0; i < centers.rows; ++i) {
		Point center (centers.at<float>(i,0), centers.at<float>(i,1));
		//cout << center << " " << pt << endl;
		//cout << norm (pt-center) << endl;
		dist.push_back ( norm (pt-center) );
		//cout << dist[i] << " ";
	}
	//cout << endl;
	idx = *std::min_element(dist.begin(), dist.end());
	return idx;
}

Point OctreeProcessor::get_num_occupied_cells (Point pt) {
	unsigned int under, over;
	get_num_occupied_cells (pt.x, pt.y, under, over);
	//cout << under << " " << over << endl;
	return Point(under, over);
}

void OctreeProcessor::get_num_occupied_cells (double x, double y,
																							 unsigned int &under,
																							 unsigned int &over) {
	under = 0, over = 0;																							 
	for (unsigned int z = 0; z < voxel_grid_size_.z(); ++z) {
		//cout << "Bin: " << z << endl;
		point3d pt = voxel_coord_to_point (point3d(x, y, z), resolution_, min_);
		//cout << "Z " << pt.z() << endl;
		if (is_occupied (pt)) {
			if ( z <= min_bin_)
				++under;
			else
				++over;
		}
	}
}

//double kmeans(InputArray data, int K, InputOutputArray bestLabels, TermCriteria criteria, int attempts, int flags, OutputArray centers=noArray() )
