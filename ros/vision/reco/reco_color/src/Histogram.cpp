#include "Histogram.h"

using namespace std;
using namespace cv;

////////////////////////////////////////////////////////////////////////////////
//  PUBLIC METHODS
////////////////////////////////////////////////////////////////////////////////
Mat Histogram::selective_spatial_histogram (Mat img, int nbins) {
	Mat classes = compute_classes_image (img, nbins);
	vector<Mat> II = integral_images (classes, nbins);
	Mat desc = compute_selective_descriptor (classes, II);
	return desc;
}

Mat Histogram::spatial_histogram (Mat img, int nbins) {
	Mat classes = compute_classes_image (img, nbins);
	vector<Mat> II = integral_images (classes, nbins);
	Mat desc = compute_descriptor (II);
	return desc;
}

Mat Histogram::classic_histogram (Mat img, Mat mask, int nbins) {
	Mat hsv;
	cvtColor (img, hsv, CV_BGR2HSV);
  /// Using 10 bins for hue and 10 for saturation
  int h_bins = 10; int s_bins = 10;
  int histSize[] = { h_bins, s_bins };
  // hue varies from 0 to 256, saturation from 0 to 180
  float h_ranges[] = { 0, 256 };
  float s_ranges[] = { 0, 180 };
  const float* ranges[] = { h_ranges, s_ranges };
  // Use the 0-th and 1-st channels
  int channels[] = { 0, 1 };
  
  Mat hist;
  //calcHist( &hsv, 1, channels, Mat(), hist, 2, histSize, ranges, true, false );
  calcHist( &hsv, 1, channels, mask, hist, 2, histSize, ranges, true, false );
  normalize( hist, hist, 0, 1, NORM_MINMAX, -1, Mat() );
  return hist;
}

////////////////////////////////////////////////////////////////////////////////
//  PRIVATE METHODS
////////////////////////////////////////////////////////////////////////////////
Mat Histogram::compute_classes_image (Mat img, int nbins) {
	Mat classes (img.rows, img.cols, img.type());

	//float* ranges = get_ranges(nbins, -128, 128);
	float* ranges = get_ranges(nbins, 0, 255);
	for (int i=0; i<img.rows; ++i) {
		for (int j=0; j<img.cols; ++j)
			classes.at<uint8_t>(i,j) = get_bin (img.at<uint8_t>(i,j), ranges, nbins);
	}
	return classes;
}

// Assume II[0] existe and all II[i] have same size, which should be the case.
Mat Histogram::compute_selective_descriptor (Mat img, vector<Mat> II) {
	// Nbins * 4 for each bin = Nbins*(Nbins*4)
	Mat desc = Mat::zeros(II.size(), 4*II.size(), CV_32FC1);
	int nbins = II.size();
	int* quad;
	for (int i=0; i<II[0].rows; ++i) {
		for (int j=0; j<II[0].cols; ++j) {
			for (int n=0; n<nbins; ++n) {
				int k = img.at<uint8_t>(i,j);
				quad = compute_spatial_quadrants (II[n], i, j);
				desc.at<float>(n, 4*k + 0) += quad[0];
				desc.at<float>(n, 4*k + 1) += quad[1];
				desc.at<float>(n, 4*k + 2) += quad[2];
				desc.at<float>(n, 4*k + 3) += quad[3];
			}
		}
	}
	// normalize : divide by number of pixels in this class
	for (int n=0; n<nbins; ++n) {
		for (int k=0; k<nbins; ++k) {
			float sum = desc.at<float>(n, 4*k + 0) + desc.at<float>(n, 4*k + 1)
							+ desc.at<float>(n, 4*k + 2) + desc.at<float>(n, 4*k + 3);
			desc.at<float>(n, 4*k + 0) /= sum;
			desc.at<float>(n, 4*k + 1) /= sum;
			desc.at<float>(n, 4*k + 2) /= sum;
			desc.at<float>(n, 4*k + 3) /= sum;
		}
	}
	
	return desc;
}
// Assume II[0] existe and all II[i] have same size, which should be the case.
Mat Histogram::compute_descriptor (vector<Mat> II) {
	Mat desc = Mat::zeros(II.size(), 4, CV_32FC1);
	int nbins = II.size();
	int* quad;
	for (int i=0; i<II[0].rows; ++i) {
		for (int j=0; j<II[0].cols; ++j) {
			for (int n=0; n<nbins; ++n) {
				quad = compute_spatial_quadrants (II[n], i, j);
				desc.at<float>(n, 0) += quad[0];
				desc.at<float>(n, 1) += quad[1];
				desc.at<float>(n, 2) += quad[2];
				desc.at<float>(n, 3) += quad[3];
			}
		}
	}
	// normalize : divide by number of pixels in this class
	for (int n=0; n<nbins; ++n) {
		float sum = desc.at<float>(n, 0) + desc.at<float>(n, 1)
							+ desc.at<float>(n, 2) + desc.at<float>(n, 3);
		if (sum != 0) {
			desc.at<float>(n, 0) /= sum;
			desc.at<float>(n, 1) /= sum;
			desc.at<float>(n, 2) /= sum;
			desc.at<float>(n, 3) /= sum;
		}
	}
	
	return desc;
}

float* Histogram::get_ranges(int nbins, int min, int max) {
	float* ranges = new float[nbins+1];
	for (int i=0; i<nbins; ++i)
		ranges[i] = min + i*(max-min)/nbins;
	return ranges;
}

int Histogram::get_bin (int val, float* ranges, int nbins) {
	for (int i=0; i<nbins-1; ++i) {
		if (val >= ranges[i] && val < ranges[i+1])
			return i;
	}
	if(val >= ranges[nbins-1])
		return nbins-1;
	
	cout << "Warning: get_ranges: unexpected value, this shouldn't happen." << endl;
	return -1;
}

int* Histogram::compute_spatial_quadrants (Mat II, int i, int j) {
	int* quad = new int[4];
	if (i < II.rows && j < II.cols) {
		quad[0] = II.at<int>(i, j);
		quad[1] = II.at<int>(i, II.cols-1) - II.at<int>(i, j);
		quad[2] = II.at<int>(II.rows-1, j) - II.at<int>(i, j);
		quad[3] = II.at<int>(II.rows-1, II.cols-1) + II.at<int>(i, j)
						- II.at<int>(i, II.cols-1) - II.at<int>(II.rows-1, j);
	}
	return quad;
}

vector<Mat> Histogram::integral_images (Mat classes, int nbins) {
	vector<Mat> II(nbins);
	II = selective_cumsum_cols (selective_cumsum_rows (classes, nbins),
															nbins);
	return II;
}

vector<Mat> Histogram::selective_cumsum_rows (Mat img, int nbins) {
	vector<Mat> I(nbins);
	for (int n=0; n<nbins; ++n)
		I[n] = Mat(img.rows, img.cols, CV_32SC1);
	int s[nbins];
	
	for(int i=0; i<img.rows; ++i) {
		for (int n =0; n<nbins; ++n)
			s[n] = 0;
		for(int j=0; j<img.cols; ++j) {
			int k = img.at<uint8_t>(i, j);
			s[k] += 1;
			for (int n=0; n<nbins; ++n)
				I[n].at<int>(i, j) = s[n];
		}
	}
	return I;
}

vector<Mat> Histogram::selective_cumsum_cols (vector<Mat> I, int nbins) {
	vector<Mat> II(nbins);
	for (int n=0; n<nbins; ++n)
		II[n] = Mat(I[0].rows, I[0].cols, CV_32SC1);

	for(int j=0; j<I[0].cols; ++j) {
		// first value of each column
		for (int n=0; n<nbins; ++n)
			II[n].at<int>(0, j) = I[n].at<int>(0, j);
		for(int i=1; i<I[0].rows; ++i) {
			for (int n=0; n<nbins; ++n)
				II[n].at<int>(i, j) = II[n].at<int>(i-1, j) + I[n].at<int>(i, j);
		}
	}
	return II;
}
