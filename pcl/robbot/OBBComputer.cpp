#include "OBBComputer.h"

using namespace std;
namespace bf = boost::filesystem;

OBBComputer::OBBComputer (string output)
{
	_processed_name = output + "_proc.txt";
	_xyz_results_name = output + "_xyz_res.txt";
	_lab_results_name = output + "_lab_res.txt";	
}

void OBBComputer::process_root (std::string dir)
{
	if (bf::is_directory (dir))
  {
  	bf::directory_iterator end_itr;
    for (bf::directory_iterator i (dir); i != end_itr; ++i)
    {
	    if ( !already_processed (dir, i->path().string()) )
    	{
  	  	if (process_class (i->path().string()) == 0)
  	  	{
  	  		// Save this class as processed
					save_processed_filename (i->path().string());
				}
			}
			else
				std::cout << i->path().string() << " already processed." << std::endl;
    }
  }
}

int OBBComputer::process_class (std::string dir)
{
	if (bf::is_directory (dir))
  {
  	bf::directory_iterator end_itr;
    for (bf::directory_iterator i (dir); i != end_itr; ++i)
    {
    	if ( !already_processed (dir, i->path().string()) )
    	{
    		if (bf::is_directory (i->path().string()))
			  {
	    		if (process_instance (i->path().string()) == 0)
	    		{
	    			// Save this instance as processed
						save_processed_filename (i->path().string());
					}
				}
			}
			else
				std::cout << i->path().string() << " already processed." << std::endl;
    }
  }
  
  return 0;
}

int OBBComputer::process_instance (std::string dir)
{ 
	if (bf::is_directory (dir))
  {
  	bf::directory_iterator end_itr;
    for (bf::directory_iterator i (dir); i != end_itr; ++i)
    {
    	if ( !already_processed (dir, i->path().string()) 
    				&& get_extension(i->path().string()).compare("pcd") == 0 )
    	{
				pcl::PCDReader reader;
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
				PointType pt_xyz;
				PointType lab_min, lab_max;    	
    	
		  	std::cout << "Processing " << i->path().string() << std::endl;
				// Save processed pose filename
				save_processed_filename (i->path().string());
		  	// Read cloud
				reader.read (i->path().string(), *cloud);
				// Compute obbs
				process_pose (cloud, pt_xyz, lab_min, lab_max);
				// Save results
				save_results(i->path().string(), _xyz_results_name, pt_xyz);
				save_results(i->path().string(), _lab_results_name, lab_min, lab_max);
			}
			else
				std::cout << i->path().string() << " already processed." << std::endl;
		}
	}
	
	return 0;
}

void OBBComputer::process_pose (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, PointType &pt_xyz, PointType &lab_min, PointType &lab_max)
{
	Cloud::Ptr xyz (new Cloud);
	Cloud::Ptr lab (new Cloud);
	// get XYZ and RGB clouds from input cloud
	split_XYZ_RGB (cloud, xyz, lab);
	// Get dimensions of XYZ obb
	getXYZBox (xyz, pt_xyz);
	// Get dimensions of RGB obb
	getLABBox (lab, lab_min, lab_max);
}

void OBBComputer::split_XYZ_RGB (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Cloud::Ptr xyz, Cloud::Ptr lab)
{
	for (size_t i=0; i<cloud->points.size (); ++i)
	{
	  PointType pt;
		pt.x = cloud->points[i].x;
		pt.y = cloud->points[i].y;
		pt.z = cloud->points[i].z;		
		xyz->points.push_back (pt);
		pt.x = cloud->points[i].r;
		pt.y = cloud->points[i].g;
		pt.z = cloud->points[i].b;
		lab->points.push_back (pt); // not lab yet, still rgb
		
		rgb2lab (cloud->points[i].r, cloud->points[i].g, cloud->points[i].b,
						pt.x, pt.y, pt.z);
		lab->points.push_back (pt);
	}
}

void OBBComputer::getXYZBox (Cloud::Ptr xyz, PointType &pt)
{
	Eigen::Quaternionf quaternion;
	Eigen::Vector3f translation;
	double w, h, d;
	_det.bbox_mvbb(xyz, quaternion, translation, w, h, d);
	pt.x = w;
	pt.y = h;
	pt.z = d;
}

void OBBComputer::getLABBox (Cloud::Ptr lab, PointType &min, PointType &max)
{
	Eigen::Quaternionf quaternion;
	Eigen::Vector3f translation;
	double w, h, d;
	_det.bbox_mvbb(lab, quaternion, translation, w, h, d, min, max);
}

void OBBComputer::save_results (std::string dir, std::string filename, PointType pt_xyz)
{
  std::ofstream myfile;
  std::string path = path2updir(dir) + filename;
  myfile.open (path.c_str(), std::ios_base::app);
  myfile << path2pose(dir) << " " << pt_xyz.x << " " << pt_xyz.y << " " << pt_xyz.z << "\n";
  myfile.close();
}

void OBBComputer::save_results (std::string dir, std::string filename, PointType lab_min, PointType lab_max)
{
  std::ofstream myfile;
  std::string path = path2updir(dir) + filename;
  myfile.open (path.c_str(), std::ios_base::app);
  myfile << path2pose(dir) << " " << lab_min.x << " " << lab_min.y << " " << lab_min.z
	  											 << " " << lab_max.x << " " << lab_max.y << " " << lab_max.z <<"\n";
  myfile.close();
}

// Save path, string to write in file
void OBBComputer::save_processed_filename (std::string file_path)
{
  std::ofstream myfile;
  std::string path = path2updir(file_path) + _processed_name;
  //cout << "Path:" << path << endl;
  myfile.open (path.c_str(), std::ios_base::app);
  myfile << file_path << "\n";
  myfile.close();
}
// Save path, string to check in file
int OBBComputer::already_processed (std::string dir, std::string input)
{
  std::ifstream myfile;
  std::string line;
  std::string path = dir + "/" + _processed_name;
  //cout << "Already : " << path << endl;
  myfile.open (path.c_str());
  if (myfile.is_open())
  {
    while ( myfile.good() )
    {
			getline (myfile,line);
			if (input.compare(line) == 0)
				return 1;
		}
 	  myfile.close();
  }
 
  return 0;
}

string OBBComputer::get_xyz_name ()
{
	return _xyz_results_name;
}

string OBBComputer::get_lab_name ()
{
	return _lab_results_name;
}

void OBBComputer::rgb2lab (uint r, uint g, uint b, float &L, float &A, float &B)
{
	float X, Y, Z;
	
	rgb2xyz (r, g, b, X, Y, Z);
	xyz2lab (X, Y, Z, L, A, B);
	
	//cout << L << " " << A << " " << B << endl;
}

void OBBComputer::rgb2xyz(uint R, uint G, uint B,
													float &X, float &Y, float &Z)
{
	// normalize between 0 and 1
	float r = static_cast<float>(R)/255;
	float g = static_cast<float>(G)/255;
	float b = static_cast<float>(B)/255;
 
	if (r > 0.04045){ r = pow((r + 0.055) / 1.055, 2.4); }
	else { r = r / 12.92; }
	if ( g > 0.04045){ g = pow((g + 0.055) / 1.055, 2.4); }
	else { g = g / 12.92; }
	if (b > 0.04045){ b = pow((b + 0.055) / 1.055, 2.4); }
	else {	b = b / 12.92; }
	
	r *= 100;
	g *= 100;
	b *= 100;
 
	X = r * 0.4124 + g * 0.3576 + b * 0.1805;
	Y = r * 0.2126 + g * 0.7152 + b * 0.0722;
	Z = r * 0.0193 + g * 0.1192 + b * 0.9505;
	
	//cout << X << " " << Y << " " << Z << endl;
}

void OBBComputer::xyz2lab(float X, float Y, float Z,
													float &l, float &a, float &b)
{
	float REF_X = 95.047;
	float REF_Y = 100.000;
	float REF_Z = 108.883;
	float x = X / REF_X; 
	float y = Y / REF_Y;
	float z = Z / REF_Z;  
 
	if ( x > 0.008856 ) { x = pow( x , 1.0/3.0 ); }
	else { x = ( 7.787 * x ) + ( 16.0/116.0 ); }
	if ( y > 0.008856 ) { y = pow( y , 1.0/3.0 ); }
	else { y = ( 7.787 * y ) + ( 16.0/116.0 ); }
	if ( z > 0.008856 ) { z = pow( z , 1.0/3.0 ); }
	else { z = ( 7.787 * z ) + ( 16.0/116.0 ); }
 
	l = ( 116 * y ) - 16;
	a = 500 * ( x - y );
	b = 200 * ( y - z );
	
	//cout << l << " " << a << " " << b << endl;
}
