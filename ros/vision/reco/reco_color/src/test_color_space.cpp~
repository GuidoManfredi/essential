#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

int* compute_spatial_quadrants (Mat II, int i, int j) {
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
// Assume II[0] existe and all II[i] have same size, which should be the case.
Mat compute_selective_descriptor (Mat img, vector<Mat> II) {
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
Mat compute_descriptor (vector<Mat> II) {
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


vector<Mat> selective_cumsum_rows (Mat img, int nbins) {
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

vector<Mat> selective_cumsum_cols (vector<Mat> I, int nbins) {
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

vector<Mat> integral_images (Mat classes, int nbins) {
	vector<Mat> II(nbins);
	II = selective_cumsum_cols (selective_cumsum_rows (classes, nbins),
															nbins);
	return II;
}

float* get_ranges(int nbins, int min, int max) {
	float* ranges = new float[nbins+1];
	for (int i=0; i<nbins; ++i)
		ranges[i] = min + i*(max-min)/nbins;
	return ranges;
}

int get_bin (int val, float* ranges, int nbins) {
	for (int i=0; i<nbins-1; ++i) {
		if (val >= ranges[i] && val < ranges[i+1])
			return i;
	}
	if(val >= ranges[nbins-1])
		return nbins-1;
	
	cout << "Warning: get_ranges: unexpected value, this shouldn't happen." << endl;
	return -1;
}

Mat compute_classes_image (Mat img, int nbins) {
	Mat classes (img.rows, img.cols, img.type());

	//float* ranges = get_ranges(nbins, -128, 128);
	float* ranges = get_ranges(nbins, 0, 255);
	for (int i=0; i<img.rows; ++i) {
		for (int j=0; j<img.cols; ++j)
			classes.at<uint8_t>(i,j) = get_bin (img.at<uint8_t>(i,j), ranges, nbins);
	}
	return classes;
}

int main (int argc, char** argv) {

	if (argc != 2) {
		cout << "Usage : test_color_space imagepath" << endl;
		return 1;
	}

	int nbins = 3;
	Mat img, gray, classes;
	img = imread (argv[1], CV_LOAD_IMAGE_GRAYSCALE);
	
	clock_t start = clock();
	classes = compute_classes_image (img, nbins);
	vector<Mat> II = integral_images (classes, nbins);
	Mat desc = compute_descriptor (II);
	clock_t ends = clock();
	
  cout << "Running Time : " << (double) (ends - start)*1000 / CLOCKS_PER_SEC << endl;
  cout << desc << endl;
	/*
	cout << gray << endl;
	cout << endl;
	cout << classes << endl;

	float *ranges = get_ranges (nbins, -128, 128);
	int val = get_bin(atoi(argv[1]), ranges, nbins);
	cout << "Ranges: ";
	for (int i=0; i<nbins; ++i)
		cout << ranges[i] << " ";
	cout << endl;
	cout << "Bin number for " << argv[1] << ":" << val << endl;
	vector<Mat> I = selective_cumsum_rows (classes, nbins);
	vector<Mat> II = selective_cumsum_cols (I, nbins);
	cout << classes << endl;
	cout << endl;
	cout << II[2] << endl;
	*/
	return 0;
}

// TODO tester selective descriptor. Test descriptor normal.

/*
vector<Mat> compute_integral_images (Mat classes, int nbins) {
	vector<Mat> II(nbins);
	int s[nbins];
	
	// case x = 0
	for (int y=0; y<classes.cols; ++y) {
		s[classes.at<char>(0, y)] += 1;
		for (int n=0; n<nbins; ++n)
			II[n].at<char>(0, y) = s[n];
	}
	
	for (int x=1; x<classes.rows; ++x) {
		for (int n=0; n<nbins; ++n)
			s[n] = 0;
			
		for (int y=0; y<classes.cols; ++y) {
			s[classes.at<char>(x, y)] += 1;
			for (int n=0; n<nbins; ++n)
				II[n].at<char>(x, y) = II[n].at<char>(x-1, y) + s[n];
		}
	}
	
	return II;
}
*/
