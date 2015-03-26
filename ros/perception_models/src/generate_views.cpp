#include <pcl/visualization/pcl_visualizer.h>

#include </home/gmanfred/devel/libraries/pcl/apps/3d_rec_framework/include/pcl/apps/3d_rec_framework/pc_source/mesh_source.h>

using namespace std;
using namespace pcl;
using namespace rec_3d_framework;

typedef pcl::PointXYZ PointType;

int main(int argc, char** argv)
{
	string views_dir = argv[1];//"../views/";
	string models_dir = argv[2];//"../cad";

	boost::shared_ptr<pcl::rec_3d_framework::MeshSource<PointType> > mesh_source (new pcl::rec_3d_framework::MeshSource<PointType>);
    mesh_source->setPath (models_dir);
    mesh_source->setResolution (250);
	mesh_source->setTesselationLevel (1);
    mesh_source->setViewAngle (57.f);
    mesh_source->setRadiusSphere (1.5f);
    //mesh_source->setModelScale (0.001f);
    mesh_source->setModelScale (1.0f);
	mesh_source->generate (views_dir);

	boost::shared_ptr<vector<Model<PointType> > > models = mesh_source->getModels();
	for (size_t i=0; i<models->size(); ++i) {
		Model<PointType> m = models->at(i);
		
		std::stringstream pathmodel;
        pathmodel << argv[3] << m.id_ << ".pcd"; //"../models/" 
 		std::cout << "Saving " << m.id_ << ", " << m.assembled_->points.size() << " points." << std::endl;
		pcl::io::savePCDFileASCII (pathmodel.str(), *m.assembled_);
		
		/*
		pcl::visualization::PCLVisualizer vis("results");
		pcl::visualization::PointCloudColorHandlerCustom<PointType> random_handler (m.assembled_, 255, 0, 0);
		vis.addPointCloud<PointType> (m.assembled_, random_handler, "points");
		vis.spin();
		*/
	}

	return 0;
}
