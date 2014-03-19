#include "SliceProcessor.h"

using namespace cv;
using namespace std;

SliceProcessor::SliceProcessor () {

}

vector< vector<Point> > SliceProcessor::compute_interest_area_contours (vector<Mat> slices,
																																				unsigned int thresh) {
	std::vector< std::vector<cv::Point> > all_contours;
	for (size_t i = 0; i < slices.size(); ++i) {
		//save_slice (slices[i], i);
		Mat denoised = denoise (slices[i]);
		//save_slice (denoised, i);
		std::vector< std::vector<cv::Point> > contours = compute_contours (denoised);
		for (size_t i = 0; i < contours.size(); ++i) {
			if (contours[i].size() > thresh)
				all_contours.push_back (contours[i]);
		}
	}
	cout << "Found " << all_contours.size() << " contours." << endl;
	return all_contours;
}

Mat SliceProcessor::denoise (Mat slice) {
	width_  = slice.rows;
	height_ = slice.cols;
	cv::Mat res(slice.size(), CV_8UC1);
	int erosion_elem = MORPH_RECT;//MORPH_ELLIPSE; //MORPH_CROSS;//MORPH_RECT;
	int erosion_size = 1;
	Mat element = getStructuringElement( erosion_elem,
                                       Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                       Point( erosion_size, erosion_size ) );
	cv::morphologyEx(slice, res, cv::MORPH_OPEN, element);
	return res;
}

vector< vector<Point> > SliceProcessor::compute_contours (Mat slice) {
	vector< vector<Point> > contours;
 	vector<Vec4i> hierarchy;
	findContours( slice, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point(0, 0) );
	return contours;
}

Mat SliceProcessor::contours_to_mat (vector< vector<Point> > contours) {
	Mat res = Mat::zeros (width_, height_, CV_8UC1);
	for (size_t i = 0; i < contours.size(); ++i) {
		for (size_t j = 0; j < contours[i].size(); ++j) {
			res.at<unsigned char>(contours[i][j].y, contours[i][j].x) = 255;
		}
	}
	return res;
}

Mat SliceProcessor::contours_to_mat (vector<Point> contours) {
	Mat res = Mat::zeros (width_, height_, CV_8UC1);
	for (size_t i = 0; i < contours.size(); ++i)
			res.at<unsigned char>(contours[i].y, contours[i].x) = 255;
	return res;
}

//void SliceProcessor::save_slices (vector< vector<Point> > contours, string path) {
void SliceProcessor::save_slice (Mat contours, unsigned int i) {
	std::stringstream ss;
	ss << "slice" << i << ".png";
	cv::imwrite (ss.str(), contours);
}



vector<Vec4i> SliceProcessor::segment_lines (cv::Mat img, vector<Point> &all_inliers) {
	vector<Vec4i> segments;
	Mat working_copy = img.clone();
	vector<Point> inliers;
	do {
		Vec4i line = segment_line_ransac (working_copy, 4, 5000, inliers);
		Vec4i segment = min_max_along_line (inliers, line);
		remove (working_copy, inliers);
		all_inliers.insert (all_inliers.end(), inliers.begin(), inliers.end());
		segments.push_back (segment);
	//} while (inliers.size() > 250);
	//} while (inliers.size() > 150);
	//} while (inliers.size() > 40);
	} while (inliers.size() > 20); // 20*0.05 = 1m
	return segments;
}

Vec4i SliceProcessor::segment_line_ransac (cv::Mat img,
																						unsigned int tresh,
																						unsigned int num_iterations,
																						vector<Point> &max_inliers) {
	max_inliers.clear();
	Vec4i best_line;
	for (unsigned int i = 0; i < num_iterations; ++i) {
		// Take two points on the border of the image
		Vec4i line = get_two_points_random (img.cols, img.rows);
		//Vec4i line = get_two_points_random ();
		vector<Point> inliers = get_inliers (img, line, tresh);
		//cout << "Num inliers : " << inliers.size() << endl;
		if (inliers.size() > max_inliers.size()) {
			max_inliers = inliers;
			best_line = line;
		}
	}

	//for (size_t i = 0; i < max_inliers.size(); ++i)
		//cout << int(img.at<unsigned char>(max_inliers[i])) << endl;
	
	return best_line;
}
/*
Vec4i SliceProcessor::get_two_points_random () {
	unsigned int i1 = rand()%occupied_points_.size();
	unsigned int i2 = rand()%occupied_points_.size();

	return Vec4i (occupied_points_[i1].x, 
								occupied_points_[i1].y,
								occupied_points_[i2].x,
								occupied_points_[i2].y);
}
*/
Vec4i SliceProcessor::get_two_points_random (unsigned int width,
																							unsigned int height) {
	unsigned int x1, y1, x2, y2;
	if (rand()%2) {
		x1 = rand()%width;
		y1 = 0;
	}	else {
		x1 = 0;
		y1 = rand()%height;
	}
	
	if (rand()%2) {
		x2 = width;
		y2 = rand()%height;
	} else {
		x2 = rand()%width;
		y2 = height;
	}
	
	return Vec4i (x1, y1, x2, y2);
}

vector<Point> SliceProcessor::get_inliers (Mat img, Vec4i line,
																					 unsigned int tresh) {
	vector<Point> inliers;
	Point L1 (line(0),line(1));
	Point L2 (line(2),line(3));
	LineIterator it(img, L1, L2, 8);
	for(int i = 0; i < it.count; ++i, ++it) {
		Point pt = it.pos();
		if ( img.at<unsigned char>(pt) == 255 )
			inliers.push_back (pt);
	}
	
	//for (size_t i = 0; i < inliers.size(); ++i)
		//cout << int(img.at<unsigned char>(inliers[i])) << endl;
	
	return inliers;
}
/*
vector<Point> SliceProcessor::get_inliers (Mat img, Vec4i line,
																					 unsigned int tresh) {
	vector<Point> inliers;
	Vec4i line_down, line_up;
	get_bounding_lines (line, tresh, img.cols, img.rows, line_down, line_up);
	for (int x=0; x<img.cols; ++x) {
		for (int y=0; y<img.rows; ++y) {
			Point p (y, x);
			if (is_up(line_down, p) && !is_up(line_up, p) ) {
				if ( img.at<unsigned char>(p) == 255 )
					inliers.push_back (p);
			}
		}
	}
	//for (size_t i = 0; i < inliers.size(); ++i)
		//cout << int(img.at<unsigned char>(inliers[i])) << endl;
	
	return inliers;
}
*/
void SliceProcessor::remove (cv::Mat &img, vector<Point> pts) {
	cout << "Removing " << pts.size() << " points." << endl;
	for (size_t i = 0; i < pts.size(); ++i) {
		//cout << static_cast<int>(img.at<unsigned char>(pts[i])) << endl;
		img.at<unsigned char>(pts[i]) = 0;
	}
}

Vec4i SliceProcessor::min_max_along_line (vector<Point> vec, Vec4i line) {
	Vec4i new_line;																		
	Point min, max;
	min_max_along_line (vec, line, min, max);
	new_line(0) = min.x;
	new_line(1) = min.y;
	new_line(2) = max.x;
	new_line(3) = max.y;
	return new_line;
}

void SliceProcessor::min_max_along_line (vector<Point> vec, Vec4i line,
																					Point &min, Point &max) {
	float min_dist_to_min = 1e4;
	float min_dist_to_max = 1e4;
	Point L1(line(0), line(1)), L2(line(2), line(3));
	for (size_t i = 0; i < vec.size(); ++i) {
		float dist_to_min = norm (vec[i] - L1);
		float dist_to_max = norm (vec[i] - L2);
		if (dist_to_min < min_dist_to_min) {
			min = vec[i];
			min_dist_to_min = dist_to_min;
		}
		if (dist_to_max < min_dist_to_max) {
			max = vec[i];
			min_dist_to_max = dist_to_max;
		}
	}
}

vector<Point> SliceProcessor::get_occupied_points (Mat img) {
	vector<Point> occupied_points;
	for (int y = 0; y < img.rows; ++y) {
		for (int x = 0; x < img.cols; ++x) {
			if (img.at<unsigned char>(y, x) == 255)
				occupied_points.push_back (Point(x,y));
		}
	}
	cout << "Counter " << occupied_points.size() << " occupied points" << endl;
	return occupied_points;
}

void SliceProcessor::get_bounding_lines (Vec4i segment, double delta,
																				 int width, int height,
																				 Vec4i &line1, Vec4i &line2) {
	if ( segment(0) == 0 ) {
		line1(0) = 0;
		line2(0) = 0;
		line1(1) = segment(1) - delta;
		line2(1) = segment(1) + delta;
	}
	else { // segment(1) == 0
		line1(0) = segment(0) - delta;
		line2(0) = segment(0) + delta;
		line1(1) = 0;
		line2(1) = 0;
	}
	
	if ( segment(2) == width ) {
		line1(2) = width;
		line2(2) = width;
		line1(3) = segment(3) - delta;
		line2(3) = segment(3) + delta;
	}
	else { // segment(3) == height
		line1(2) = segment(2) - delta;
		line2(2) = segment(2) + delta;
		line1(3) = height;
		line2(3) = height;
	}
}

bool SliceProcessor::is_up (Vec4i line, Point p){
	return ((line(2) - line(0))*(p.y - line(1)) - (line(3) - line(1))*(p.x - line(0))) > 0;
}

vector<Vec4i> SliceProcessor::merge_close_segments (vector<Vec4i> segments) {
	vector<Vec4i> groups;
	vector<unsigned int> nums;
	double thresh = 80;
	for (size_t i = 0; i < segments.size(); ++i) {
		bool assigned = false;
		for (size_t k = 0; k < groups.size(); ++k) {
			double dist = segment_distance (segments[i], groups[k]);
			if (dist < thresh && !assigned) {
				add_to_group (segments[i], groups[k], nums[k]);
				assigned = true;
			}
		}
		if (!assigned) {
			groups.push_back (segments[i]);
			nums.push_back(1);
		}
	}
	cout << groups.size () << endl;
	return groups;
}

double SliceProcessor::segment_distance (Vec4i seg1, Vec4i seg2) {
	return (norm(seg1 - seg2));
}

void SliceProcessor::add_to_group (Vec4i in, Vec4i &group, unsigned int &size) {
	group(0) *= size;
	group(1) *= size;
	group(2) *= size;
	group(3) *= size;
	++size;
	group = (group + in);
	group(0) /= size;
	group(1) /= size;
	group(2) /= size;
	group(3) /= size;
}

