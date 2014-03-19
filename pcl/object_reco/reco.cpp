#include <iostream>
#include <pcl/io/openni_grabber.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
 
#include "Reco.hpp"

using namespace std;

int
main (int argc, char** argv)
{
	if (argc != 3)
	{
		cout << "Usage : reco trained scene" << endl;
		return (-1);		
	}
	
	Reco r;
	r.load_models (argv[1]);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud (new pcl::PointCloud<pcl::PointXYZ>);	
	pcl::io::loadPCDFile<pcl::PointXYZ> (argv[2], *scene_cloud);
	r.run (scene_cloud);
	

	return (0);
}
