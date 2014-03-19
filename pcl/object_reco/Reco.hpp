#ifndef DEF_RECO
#define DEF_RECO

#include <pcl/io/pcd_io.h>

#include "file_tools.h"
#include "Pipeline.hpp"

typedef struct
{
	std::vector<pcl::PointCloud<PointType>::Ptr> kpts;
	std::vector<pcl::PointCloud< DescriptorType>::Ptr> descs;
	pcl::PointCloud<PointType>::Ptr agreg_kpts;
	pcl::PointCloud<DescriptorType>::Ptr agreg_descs;
} Model;

class Reco
{
public:
	Reco();
	~Reco();
	void load_models (std::string dirname);	
	void run(pcl::PointCloud<PointType>::Ptr scene);

protected:
	std::vector<Model> m_models;
	
	float m_kpts_rad;
	float m_descr_rad;
};

#endif
