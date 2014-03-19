#ifndef DEF_ROBBOT_OBBCOMPUTER
#define DEF_ROBBOT_OBBCOMPUTER

#include <pcl/io/openni_grabber.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>

#include "string_tools.h"
#include "PlanDetector.h"

class OBBComputer
{
public:
	void process_root (std::string path);
	int process_class (std::string path);
	int process_instance (std::string path);
	void process_pose (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, PointType &pt_xyz, PointType &lab_min, PointType &lab_max);
	
	void split_XYZ_RGB (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Cloud::Ptr xyz, Cloud::Ptr rgb);
	void getXYZBox (Cloud::Ptr xyz, PointType &pt);
	void getLABBox (Cloud::Ptr lab, PointType &min, PointType &max);

	void save_results (std::string dir, std::string filename, PointType pt_xyz);
	void save_results (std::string dir, std::string filename, PointType lab_min, PointType lab_max);
	void save_processed_filename (std::string file_path);
	int already_processed (std::string dir, std::string input);

	void rgb2lab (uint r, uint g, uint b, float &L, float &A, float &B);
	void rgb2xyz(uint R, uint G, uint B, float &X, float &Y, float &Z);
	void xyz2lab(float X, float Y, float Z,	float &l, float &a, float &b);

	std::string get_lab_name ();
	std::string get_xyz_name ();

	OBBComputer (std::string output);

protected:
	PlanDetector _det;
	
	std::string _processed_name;
	std::string _xyz_results_name;
	std::string _lab_results_name;	
};

#endif

