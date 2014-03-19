#include <opencv2/highgui/highgui.hpp>
#include <sstream>
#include <iostream>

using namespace std;
using namespace cv;

int main (int argc, char** argv) {
	int flag_w = 120;
	int flag_h = 60;
	
	Mat I = imread ("/home/gmanfred/Desktop/flags.png");
	Mat tmp = Mat(flag_w, flag_h, I.type());
	string base = "/home/gmanfred/Desktop/flags/";
	
	int counter = 0;
	for (int i=0; i<I.rows; i+=flag_h) {
		for (int j=0; j<I.cols; j+=flag_w) {
			Rect roi (j, i, flag_w, flag_h);
			//cout << i << " " << j << " " << flag_w << " " << flag_h << endl;
			tmp = I(roi);

			std::ostringstream outpath;
			outpath << base << "flag" << counter << ".png" << endl;
			//cout << outpath.str() << endl;
			imwrite (outpath.str(), tmp);
			
			++counter;
		}
	}

	return 0;
}
