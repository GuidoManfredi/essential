#include "FrontEnd.h"

using namespace std;
using namespace cv;
using namespace pcl;

FrontEnd::FrontEnd (std::string calibration_file) {
    required_matches_ = 0; // min number of matches to compute the pose
    required_inliers_ = 0.2; // min percent of inliers to continue without taking a keyframe
	pipeGeom_ = new PipelineGeom (calibration_file, required_inliers_);
	
	init_optimizer ();
}

void FrontEnd::init_optimizer () {
	//optimizer.setVerbose(verbose);
	//optimizer.vizWithGnuplot = vis;
	optimize_every_N_ = 4;
	slam_interface_ = new g2o::G2oSlamInterface(&optimizer_);
	slam_interface_->setBatchEveryN(optimize_every_N_);
	
	robust_kernel_ = -1.0;
	if ( robust_kernel_ > 0.0 ) {
		slam_interface_->_robustKernel = new g2o::RobustKernelHuber;
		slam_interface_->_robustKernel->setDelta(robust_kernel_);
	}
	
    information_matrix_.resize(6);
	information_matrix_[0]=information_matrix_[3]=information_matrix_[5]=1;
	information_matrix_[1]=information_matrix_[2]=information_matrix_[4]=0;
	
	std::vector<HardwareParameters> known_hardware_parameters;
	int hardwareParameterId=0;
	HardwareParameters hp;
	hp.tag="PARAMS_SE3OFFSET";
	hp.id=hardwareParameterId++;
	Eigen::Isometry3d sensorOffset;
	Eigen::Matrix3d R;
	R  << 1,  0,  0,
			0,  1,  0,
			0, 0,  1;
	sensorOffset = R;
	sensorOffset.translation() = Eigen::Vector3d(0,0,0);
	vector<double> v(7);
	for(int j=0;j<7;j++)
		v[j]=(g2o::internal::toVectorQT(sensorOffset))(j,0);
	hp.paramVec=v;
	known_hardware_parameters.push_back(hp);
	cout<< "1 landmarks 3D sensor added (stereocam, kinect)"<<endl;
	
    for(vector<HardwareParameters>::iterator it = known_hardware_parameters.begin(); it != known_hardware_parameters.end(); it++){
		if(it->tag=="PARAMS_SE3OFFSET"){
			g2o::ParameterSE3Offset* p=new g2o::ParameterSE3Offset;
			p->setId(it->id);
			Eigen::Map<g2o::Vector7d> pv(&(it->paramVec)[0]);
			p->setOffset(g2o::internal::fromVectorQT(pv));
			optimizer_.addParameter(p);
			//cerr << "Graph: parameters block "<<it->tag<<" added"<<endl;
		}
		else{
			cout << "Trouble with parameter block "<< it->tag <<endl;
		}
	}
}

void FrontEnd::process (Mat img, Mat depth) {
	if ( !img.data || !depth.data) // wait to have data in both images
		return;
	
	Frame current;
	initialise (current, img, depth);
	
	if (current.kpts_.size () == 0 || current.p3d_.size () == 0) // wait to have image with keypoints and 3d points
		return;

	if (KF_.size() != 0) { // General case
		if (match (KF_.back(), current)) { // if enough matches
		    if (!computePose (KF_.back(), current)) { // if not enough inliers
			    cout << "New keyframe" << endl;
			    addKeyframe(last_);
    		}
    	}
	}
	else {// (KF_.size() == 0), first keyframe
		addKeyframe(current);
	}
	
	last_ = current;
}

void FrontEnd::initialise (Frame &F, Mat image, Mat depth) {
    pipe2d_.getGray (image, F.gray_);
    pipe2d_.detectFeatures (F.gray_, F.raw_kpts_);
    //filterNaNKeyPoints (depth, raw_kpts_, kpts_, p3d_); // remove NaN of depth from keypoints
    //pipe2d_.describeFeatures (gray_, kpts_, descs_);
    
    vector<int> kpts_index;
    F.filterNaNKeyPoints (depth, F.raw_kpts_, kpts_index, F.p3d_); // remove NaN of depth from keypoints
    for ( size_t i = 0; i < kpts_index.size(); ++i ) {
        F.kpts_.push_back (F.raw_kpts_[kpts_index[i]]);
    }
    pipe2d_.describeFeatures (F.gray_, kpts_index, F.descs_);

    F.pose_ = Mat::eye (4, 4, CV_64F);
    F.id_ = KF_.size();
}

bool FrontEnd::match (Frame &F1, Frame &F2) {
    /*
    vector<DMatch> raw_matches;
    pipe2d_.match (F1.descs_, F2.descs_, matches_);
    cout << "Matches (KF" << KF_.size()-1 << "): " << matches_.size () << endl;
    pipe2d_.filterMatches (raw_matches, matches_);
    cout << "Matches after filter (KF" << KF_.size()-1 << "): " << matches_.size () << endl;
    */
	pipe2d_.match (F1.descs_, F2.descs_, matches_);
	//cout << "Matches (KF" << KF_.size()-1 << "): " << matches_.size () << endl;
	F1.setMatchesRight (matches_);
	F2.setMatchesLeft (matches_);
	//show_matches (F1, F2);
	return (matches_.size() > required_matches_);
}

bool FrontEnd::computePose (Frame &F1, Frame &F2) {
	vector<int> inliers;
	Mat	pose = pipeGeom_->computePoseOpenCV (F1.p3d_right_, F2.p2d_left_, inliers);
	/*
	vector<Point2f> p2d;
	for (size_t i = 0; i < F2.kpts_.size(); ++i) {
	    p2d.push_back (F2.kpts_[i].pt);
	}
	Mat	pose = pipeGeom_->computePoseOpenCV (F2.p3d_, p2d, inliers);
	
	Mat R = Mat::eye(3,3, CV_64F);
	Mat t = Mat::zeros(3,1, CV_64F);
	double reprojection_error = pipeGeom_->meanReprojectionError3 (F2.p3d_, p2d, R, t);
	cout << "Mean reprojection error = " << reprojection_error << endl;
	*/
	// In any case give a pose
	F2.pose_ = Mat(pose) * F1.pose_;
	//F2.pose_ = Mat(pose).inv() * F1.pose_;
	
	if ( inliers.size() > (F1.number_matches_first_match_ * required_inliers_) ) {
	    //cout << "Inliers : " << inliers.size () << endl;
	    /*
	    cout << "First matches : " << F1.number_matches_first_match_ << endl;
	    cout << "Percent needed : " << required_inliers_ << endl;
	    cout << "Inliers needed : " << F1.number_matches_first_match_ * required_inliers_ << endl;
	    */
		return true;
	}

	return false;
}

cv::Mat FrontEnd::getLastPose () {
	return last_.pose_;
}

std::vector<cv::Mat> FrontEnd::getKeyframePoses () {
    vector<Mat> keyframePoses;
    for (size_t i = 0; i < KF_.size(); ++i) {
        keyframePoses.push_back (KF_[i].pose_);
    }
    return keyframePoses;
}

std::vector<pcl::PointCloud<pcl::PointXYZ> > FrontEnd::getPoints () {
    vector<pcl::PointCloud<pcl::PointXYZ> > clouds;
    for (size_t i = 0; i < KF_.size(); ++i) {
        pcl::PointCloud<pcl::PointXYZ> cloud = vectorToCloud (KF_[i].p3d_);
        //Eigen::Map<Eigen::Matrix4f> pose ( &KF_[i].pose_.data[0] );
        //pcl::transformPointCloud (cloud, cloud, MatToEigen(KF_[i].pose_));
        pcl::transformPointCloud (cloud, cloud, MatToEigen(KF_[i].pose_.inv()));
        clouds.push_back (cloud);
    }
    return clouds;    
}

////////////////////////////////////////////////////////////////////////////////
// PRIVATE METHODS
////////////////////////////////////////////////////////////////////////////////
void FrontEnd::show_matches (Frame F1, Frame F2) {
  namedWindow("matches", 1);
  Mat img_matches;
  //cout << "Kpts : " << F1.kpts_.size() << " " << F2.kpts_.size() << endl;
  drawMatches(F1.gray_, F1.kpts_, F2.gray_, F2.kpts_, matches_, img_matches);
  imshow("matches", img_matches);
  waitKey(1);
}

void FrontEnd::printToFile (vector<Point3f> p3d, vector<Point2f> p2d, vector<int> inliers, Mat pose) {
    ofstream file("debug.txt");

    for (size_t i = 0; i < inliers.size(); ++i) {
        file << p3d[inliers[i]].x << " " << p3d[inliers[i]].y << " " << p3d[inliers[i]].x << " " << p2d[inliers[i]].x << " " << p2d[inliers[i]].y << endl;
    }
    file << pose << endl;
    file.close ();
}

Eigen::Matrix4d FrontEnd::MatToEigen (cv::Mat pose) {
    Eigen::Matrix4d R;
    R (0, 0) = pose.at<double>(0, 0); R (0, 1) = pose.at<double>(0, 1); R (0, 2) = pose.at<double>(0, 2); R (0, 3) = pose.at<double>(0, 3);
    R (1, 0) = pose.at<double>(1, 0); R (1, 1) = pose.at<double>(1, 1); R (1, 2) = pose.at<double>(1, 2); R (1, 3) = pose.at<double>(1, 3);
    R (2, 0) = pose.at<double>(2, 0); R (2, 1) = pose.at<double>(2, 1); R (2, 2) = pose.at<double>(2, 2); R (2, 3) = pose.at<double>(2, 3);
    R (3, 0) = pose.at<double>(3, 0); R (3, 1) = pose.at<double>(3, 1); R (3, 2) = pose.at<double>(3, 2); R (3, 3) = pose.at<double>(3, 3);
    return R;
}

Eigen::Isometry3d FrontEnd::MatToIsometry (Mat pose) {
    Eigen::Matrix3d R;
    R (0, 0) = pose.at<double>(0, 0); R (0, 1) = pose.at<double>(0, 1); R (0, 2) = pose.at<double>(0, 2);
    R (1, 0) = pose.at<double>(1, 0); R (1, 1) = pose.at<double>(1, 1); R (1, 2) = pose.at<double>(1, 2);
    R (2, 0) = pose.at<double>(2, 0); R (2, 1) = pose.at<double>(2, 1); R (2, 2) = pose.at<double>(2, 2);
    Eigen::Vector3d t;
    t (0) = pose.at<double>(0, 3);
    t (1) = pose.at<double>(1, 3);
    t (2) = pose.at<double>(2, 3);
    Eigen::Isometry3d iso;
    iso = R;
    iso.translation() = t;
    //cout << R << endl << t << endl;
    return iso;
}

pcl::PointCloud<pcl::PointXYZ> FrontEnd::vectorToCloud (std::vector<cv::Point3f> vector) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.height = 1;
    cloud.width = vector.size();
    for (size_t i = 0; i < vector.size(); ++i) {
        cloud.push_back (PointXYZ(vector[i].x, vector[i].y, vector[i].z));
    }
    return cloud;
}
// At each step add only a frame and the keypoints (p3d) it sees.
// The keypoints matching through 2 or more frames must have the same ID.
void FrontEnd::addKeyframe (Frame frame) {
    // Add to optimizer
    Eigen::Isometry3d iso = MatToIsometry(frame.pose_);
    if (KF_.size () == 0)
		slam_interface_->initializeVertexNoEdge(-1, frame.id_, iso);
	else {
	    Frame last_keyframe = KF_.back();
    	//slam_interface_->initializeVertexNoEdge(last_keyframe.id_, frame.id_, iso);
        slam_interface_->initializeVertexNoEdge(KF_[0].id_, frame.id_, iso); // replace later by upper line, save one mult per cycle
        for (size_t i = 0; i < last_keyframe.p3d_right_.size(); ++i) {
            vector<double> p3d(3);
            p3d[0] = last_keyframe.p3d_right_[i].x;
            p3d[1] = last_keyframe.p3d_right_[i].y;
            p3d[2] = last_keyframe.p3d_right_[i].z;
            int point_id = (last_keyframe.id_+1)*1000+i; // Be carefull this will work while KF_.size() < 1000
            slam_interface_->initializeEdge("ONLINE_EDGE_SE3:QUAT_POINT_XYZ", 0,
                                         last_keyframe.id_, point_id,
                                         p3d, information_matrix_);
        }
    }   
    slam_interface_->solveState();
    
    // Add to keyframe list
    KF_.push_back (frame);
}
/*
        vector<double> p2d(2);                             
        p2d[0] = frame.p2d_left_[i].x;
        p2d[1] = frame.p2d_left_[i].y;
        slam_interface.initializeEdge("ONLINE_EDGE_SE3:QUAT_POINT_XYZ", 0,
                                     last_keyframe.id_, frame.id_,
                                     p2d, info);
*/



