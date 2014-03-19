#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;
using namespace pcl;

pcl::PointCloud<pcl::PointXYZ>::Ptr scene (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr model (new pcl::PointCloud<pcl::PointXYZ>);

float precision = 0.005;

pcl::VoxelGrid<pcl::PointXYZ> grid;
const float leaf = 0.005f;

// ICP takes as initial guess from source (inputCloud) to target. However the result
// is the transform trom target to source.
// The final transform getted is the whole transform, not just the refine
void align (const pcl::PointCloud<pcl::PointXYZ>::Ptr model, pcl::PointCloud<pcl::PointXYZ>::Ptr scene, 
						Eigen::Matrix4f initial_guess,
						Eigen::Matrix4f &final_transform) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_model (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scene (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud (*model, *filtered_model, initial_guess);
	
    //grid.setInputCloud (scene);
    //grid.filter (*filtered_scene);

	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setMaxCorrespondenceDistance (precision);
	icp.setMaximumIterations (100);
	icp.setTransformationEpsilon (1e-8);
	//icp.setEuclideanFitnessEpsilon (0.005);
	
	//icp.setUseReciprocalCorrespondences (true);

    icp.setInputCloud(filtered_model);
    icp.setInputTarget(scene);  
    //icp.setInputCloud(model);
    //icp.setInputTarget(transformed_scene);
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
	//icp.align(Final, initial_guess);

	final_transform = icp.getFinalTransformation();
}

bool getTransforms (char* org, char* dest, char* root,
										Eigen::Matrix4f &base_arm,
										Eigen::Matrix4f &base_xtion,
										Eigen::Matrix4f &org_dest)
{
	tf::TransformListener listener;
	tf::StampedTransform base_arm_t, base_xtion_t, org_dest_t;
	try {
  	listener.lookupTransform(root, dest, ros::Time(0), base_arm_t);
		pcl_ros::transformAsMatrix (base_arm_t, base_arm);

  	listener.lookupTransform(root, org, ros::Time(0), base_xtion_t);
		pcl_ros::transformAsMatrix (base_xtion_t, base_xtion);

		listener.lookupTransform(org, dest, ros::Time(0), org_dest_t);
		pcl_ros::transformAsMatrix (org_dest_t, org_dest);
		return true;
	}
	catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
		return false;
  }
}

void pcd_callback (sensor_msgs::PointCloud2 msg) {
	pcl::fromROSMsg(msg, *scene);
}

int main(int argc, char** argv){
	assert (argc == 7 && "Usage : calibrate_xtion_position /pointcloud_topic /root_link /sensor_link /optical_link /target_link precision");
	char * root = argv[2];
	char * sensor = argv[3];
	char * optical = argv[4];
	char * target = argv[5];
	precision = atof(argv[6]);

    ros::init(argc, argv, "calibrate_xtion_position");
    ros::NodeHandle node;

	ros::Subscriber pcd_sub = node.subscribe(argv[1], 1, pcd_callback);
	ros::Publisher calibrated_pcd_pub = node.advertise<sensor_msgs::PointCloud2>("calibrated_pointcloud", 10);

    //pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/gmanfred/devel/pcl/calibration_recalage/models/arm_segment_last.pcd", *model);
	//pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/gmanfred/devel/pcl/calibration_recalage/models/bout_bras_kuka.pcd", *model);
	pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/gmanfred/devel/pcl/calibration_recalage/models/bras_kuka_2_pieces.pcd", *model);
	Eigen::Matrix4f z_rotation;
	// The model coordinate system is rotated 180 degrees around Z compared to the frame in tf
	
	z_rotation <<  0, 1, 0, 0,
				   -1, 0, 0, 0,
                   0, 0, -1, 0,
                   0, 0, 0, 1;
    /*
	z_rotation <<  -1, 0, 0, 0,
				   0, -1, 0, 0,
                   0, 0, 1, 0,
                   0, 0, 0, 1;                   
    */
	pcl::transformPointCloud (*model, *model, z_rotation);
    /*
	grid.setLeafSize (leaf, leaf, leaf);
	grid.setInputCloud (model);
    grid.filter (*model);
    */
    
    std::cout << "Loaded model " << model->points.size () << "  points." << std::endl;

	tf::TransformListener listener;
	static tf::TransformBroadcaster br;
	tf::StampedTransform root_sensor_t, root_target_t, optical_target_t, result_t, sensor_optical_t;

  ros::Rate rate(1.0);
  while (node.ok()){
		Eigen::Matrix4f root_sensor, root_target, optical_target, refine, result, sensor_optical;

		try {
			listener.lookupTransform(root, sensor, ros::Time(0), root_sensor_t);
			pcl_ros::transformAsMatrix (root_sensor_t, root_sensor);

			listener.lookupTransform(root, target, ros::Time(0), root_target_t);
			pcl_ros::transformAsMatrix (root_target_t, root_target);

			listener.lookupTransform(optical, target, ros::Time(0), optical_target_t);
			pcl_ros::transformAsMatrix (optical_target_t, optical_target);

			//align_ndt (model, scene, optical_target, refine); //optical_target is the initial guess
			align (model, scene, optical_target, refine); //optical_target is the initial guess

			Eigen::Matrix4f refined_optical_target = refine*optical_target; // Transform from target to optical
			
			pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
			pcl::transformPointCloud (*model, transformed_cloud, refined_optical_target);
			// Publish transformed cloud
			sensor_msgs::PointCloud2 msg;
			pcl::toROSMsg(transformed_cloud, msg);
			msg.header.frame_id = optical;
			calibrated_pcd_pub.publish (msg);

			//Eigen::Matrix4f C = refine;
			//Eigen::Matrix4f C = refined_optical_target.inverse();
			//Eigen::Matrix4f C = root_sensor.inverse();
			//Eigen::Matrix4f C = root_sensor.inverse()*root_target;
			//Eigen::Matrix4f C = refined_optical_target*root_target.inverse();
			//Eigen::Matrix4f C = root_sensor.inverse()*root_target*optical_target.inverse();
			//Eigen::Matrix4f C = root_sensor.inverse()*root_target*refined_optical_target.inverse();
			
			Eigen::Matrix4f C = root_sensor.inverse()*root_target*refined_optical_target.inverse();
			//Eigen::Matrix4f C = refined_optical_target;
			tf::Matrix3x3 R (C(0,0), C(0,1), C(0,2),
							 C(1,0), C(1,1), C(1,2),
							 C(2,0), C(2,1), C(2,2));
			float x, y, z;
			x = C(0, 3);
			y = C(1, 3);
			z = C(2, 3);
			// broadcast transform
			tf::Transform transform;
      		transform.setOrigin(tf::Vector3(x,y,z));
			transform.setBasis(R);
			
			double yaw, pitch, roll;
			R.getEulerYPR (yaw, pitch, roll);
			cout << x << " " << y << " " << z << " " << yaw << " " << pitch << " " << roll << endl;
			//br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), optical, "/optical_prime"));
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), optical, "/sensor"));
		}
		catch (tf::TransformException ex){
		  ROS_ERROR("%s",ex.what());
		}

		ros::spinOnce();
    rate.sleep();
  }
  return 0;
}



