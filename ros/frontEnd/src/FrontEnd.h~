#pragma once

#include <opencv2/highgui/highgui.hpp>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include "g2o/examples/interactive_slam/g2o_incremental/hardwareParameters.hpp"
#include "g2o/stuff/command_args.h"
#include "g2o/examples/interactive_slam/g2o_interactive/g2o_slam_interface.h"
#include "g2o/examples/interactive_slam/g2o_incremental/graph_optimizer_sparse_incremental.h"
#include "g2o/types/slam3d/isometry3d_mappings.h"
#include "g2o/types/slam3d/parameter_se3_offset.h"

#include "Frame.h"
#include "Pipeline2D.h"
#include "Pipeline2DGPU.h"
#include "PipelineGeom.h"

class FrontEnd
{
	public:
		FrontEnd (std::string calibration_file);
		void process (cv::Mat img, cv::Mat depth);
		void initialise (Frame &F1, cv::Mat image, cv::Mat depth);
		bool match (Frame &F1, Frame &F2);
		bool computePose (Frame &F1, Frame &F2);
		cv::Mat getLastPose ();
		std::vector<cv::Mat> getKeyframePoses ();
        std::vector<pcl::PointCloud<pcl::PointXYZ> > getPoints ();
		Frame last_;

	private:
    	void init_optimizer ();
	
		void show_matches (Frame F1, Frame F2);
    	void printToFile (std::vector<cv::Point3f> p3d, std::vector<cv::Point2f> p2d,
    	                  std::vector<int> inliers, cv::Mat pose);
        Eigen::Matrix4d MatToEigen (cv::Mat pose);
    	Eigen::Isometry3d MatToIsometry (cv::Mat pose);
    	pcl::PointCloud<pcl::PointXYZ> vectorToCloud (std::vector<cv::Point3f> vector);
		void addKeyframe (Frame F);
    		
		std::vector<Frame> KF_;
	    // Features part
		std::vector<cv::DMatch> matches_;
		Pipeline2DGPU pipe2d_;
		//Pipeline2D pipe2d_;
		PipelineGeom * pipeGeom_;
		
		unsigned int required_matches_;
		float required_inliers_;
		// Optimization part
		g2o::SparseOptimizerIncremental optimizer_;
		g2o::G2oSlamInterface *slam_interface_;
		
		unsigned int optimize_every_N_;
		double robust_kernel_;
		std::vector<double> information_matrix_;
};
