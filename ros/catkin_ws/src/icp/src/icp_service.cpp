#include <ros/ros.h>
//#include <sensor_msgs/PoseStamped.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include "icp/ICP.h"

using namespace pcl;

void transform2pose(Eigen::Matrix4f transform, geometry_msgs::PoseStamped &pose);

IterativeClosestPoint<PointXYZ, PointXYZ> icp_engine;

bool perform_icp(icp::ICP::Request  &req,
                 icp::ICP::Response &res) {
    PointCloud<PointXYZ>::Ptr input (new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr target (new PointCloud<PointXYZ>);
	fromROSMsg (req.input, *input);
	fromROSMsg (req.target, *target);
	icp_engine.setInputSource(input);
	icp_engine.setInputTarget (target);
    // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    icp_engine.setMaxCorrespondenceDistance (0.02);
    // Set the maximum number of iterations (criterion 1)
    icp_engine.setMaximumIterations (200);
    // Set the transformation epsilon (criterion 2)
    icp_engine.setTransformationEpsilon (1e-8);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp_engine.setEuclideanFitnessEpsilon (0.1);
	
    PointCloud<PointXYZ>::Ptr final (new PointCloud<PointXYZ>);
	icp_engine.align(*final);
	transform2pose(icp_engine.getFinalTransformation (), res.pose);
	ROS_INFO("ICP done with fitness: %lf", icp_engine.getFitnessScore());
	
	return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "icp_service");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("icp", perform_icp);
    ROS_INFO("Ready for ICP.");
    ros::spin();

    return 0;
}

void transform2pose(Eigen::Matrix4f transform, geometry_msgs::PoseStamped &pose) {
	Eigen::Matrix3f R;
	R << transform(0,0), transform(0,1), transform(0,2),
		 transform(1,0), transform(1,1), transform(1,2),
		 transform(2,0), transform(2,1), transform(2,2);
	Eigen::Quaternionf q(R);
	
    pose.pose.position.x = transform (0, 3);
	pose.pose.position.y = transform (1, 3);
	pose.pose.position.z = transform (2, 3);
	pose.pose.orientation.x = q.x();
	pose.pose.orientation.y = q.y();
	pose.pose.orientation.z = q.z();
	pose.pose.orientation.w = q.w();
}
