#include <pcl/visualization/pcl_visualizer.h>

#include </home/gmanfred/devel/libraries/pcl/apps/3d_rec_framework/include/pcl/apps/3d_rec_framework/pc_source/mesh_source.h>

using namespace std;
using namespace pcl;
using namespace rec_3d_framework;

int main(int argc, char** argv)
{
	string views_dir = "../views/";
	string models_dir = "../cad";

	boost::shared_ptr<pcl::rec_3d_framework::MeshSource<pcl::PointXYZ> > mesh_source (new pcl::rec_3d_framework::MeshSource<pcl::PointXYZ>);
  mesh_source->setPath (models_dir);
  mesh_source->setResolution (250);
	mesh_source->setTesselationLevel (1);
  mesh_source->setViewAngle (57.f);
  mesh_source->setRadiusSphere (1.5f);
  //mesh_source->setModelScale (0.001f);
  mesh_source->setModelScale (1.0f);
	mesh_source->generate (views_dir);

	boost::shared_ptr<vector<Model<PointXYZ> > > models = mesh_source->getModels();
	for (size_t i=0; i<models->size(); ++i) {
		Model<PointXYZ> m = models->at(i);
		
		std::stringstream pathmodel;
    pathmodel << "../models/" << m.id_ << ".pcd";
 		std::cout << "Saving " << m.id_ << ", " << m.assembled_->points.size() << " points." << std::endl;
		pcl::io::savePCDFileASCII (pathmodel.str(), *m.assembled_);
		
		/*
		pcl::visualization::PCLVisualizer vis("results");
		pcl::visualization::PointCloudColorHandlerCustom<PointXYZ> random_handler (m.assembled_, 255, 0, 0);
		vis.addPointCloud<PointXYZ> (m.assembled_, random_handler, "points");
		vis.spin();
		*/
	}

	return 0;
}
