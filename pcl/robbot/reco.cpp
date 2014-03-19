#include "Reco.h"

using namespace std;

int main (int argc, char** argv)
{
	if (argc != 2)
		cout << "Usage: reco pose_path" << endl;
	
	Reco rec (3);
	vector<string> candidates;

	// Keyboard_2
	//pcl::PointXYZ pt (0.4434, 0.1914, 0.02726);
	// glue stick
	//pcl::PointXYZ pt (0.0779771, 0.0254996, 0.0130192);
	// apple
	//pcl::PointXYZ pt (0.0805002, 0.0636096, 0.0888361);
	// plate
	/*
	pcl::PointXYZ pt (0.014304, 0.1571, 0.15392);	
	rec.load_stats ("xyz_instance_stats.txt");
	//rec.load_stats ("xyz_class_stats.txt");
	candidates = rec.reco (pt, 0.99);
	*/
	
	/* Apple_1 */
	pcl::PointXYZ min (-85.4073, -28.3931, -105.924);
	pcl::PointXYZ max (107.003, 39.8678, 65.3585);
	/* Toothpaste_1
	pcl::PointXYZ min (-112.945, -64.394, -127.214);
	pcl::PointXYZ max (125.424, 109.437, 172.571);	
	*/
	cout << "Loading stats" << endl;
	rec.load_stats ("lab_instance_stats.txt");
	cout << "Recognising " << endl;
	candidates = rec.reco (min, max, 0.90);
	
	
	for (size_t i=0; i<candidates.size (); ++i)
	{
		cout << candidates[i] << " ";
	}
	cout << endl << candidates.size () << endl;
	
	return 0;
}
