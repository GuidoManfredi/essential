#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "Histogram.h"
/**
 * This class trains a directory containing : various instances of the same class
 * OR various poses of the same instance.
 * It takes as input :
 *					- the folder to train
 *					- a list of separators
 *					- a list of end names for each different file to load
 *					- the interval of files to use (from number ... to number ...) or
 *						 a percentage interval (from ...% to ...%).
 * The class first looks for a already present training file.
 * If training file available for a part of the poses/instances,
 * it asks if want to override them, if not it loads them and continue processing.
 * After processing a file, the file name and descriptor are saved to a
 *	unique training file.
 * After extracting features from all poses/instances and saving them,
 * it merges them to obtain a unique instance/class descriptor.
 * The results are gathered in a unique file containing in each line:
 * 		- the name of the pose(instance folder)/instance(class folder)/class(whole dataset).
 *		- the descriptor
 * If the processing is stopped, whatever the reason, the training restarts at the last 
 * processed file, or the next one (two modes available).
**/
class TrainHist
{
 public:
 	TrainHist ();
 	/**
 	 * Input : a directory containing a directory for each instance of the class.
 	 		OR an instance directory containing various training files.
 	 * Output : a boolean telling if everything went right.
 	**/ 	
 	bool train (std::string dir, 
 							std::vector<std::string> separators,
 							std::vector<std::string> names,
 							float* interval);
 	/**
 	 * Input : a path to a xml file containing objects names and descriptors.
 	 *	       The new object name and descriptor will be added to the list.
 	 *				 a boolean telling if the whole file must be overriden or if
 	 *				 data must be haded to the end of file.
 	 * Output : a boolean telling if everything went right.
 	 **/
 	bool save_training (std::string path, bool override);
 	/**
 	 * Input : a path to a xml file containing ONE object name and descriptors.
 	 *	       They will be added to the current list.
 	 * Output : a boolean telling if everything went right.
 	 **/
 	bool load_training (std::string path);
 	/**
 	 * Erase the name and all descriptors of the current object;
 	 **/
 	void clear_training ();
 private:
 	/**
 	 * Check if dir is a directory. If no name has been asined to the object, use
 	 * the dir name as name. Check if the directory has already been trained. If not
 	 * train it, mark it as trained and save the results in the directory.
 	 **/
 	bool train_directory (std::string dir,
					 							std::vector<std::string> separators,
 												std::vector<std::string> names,
					 							float* interval);
 	bool train_file (std::string path, std::vector<std::string> separators,
						 												std::vector<std::string> names);
	bool save_descriptor (std::string path, cv::Mat descriptor, bool override);
	bool already_trained (std::string path);
	std::string path2name (std::string path);
	std::string path2rest (std::string path);
	std::string path2rest (std::string path, std::string separator);
	bool is_right (std::string file, std::string separator, std::string name);
 	/**
 	 * Input : a filename containing the required type of data, depending on
 	 *				 which type of information to extract.
 	 * Output : depends on information to extract. Some data to be processed by
 	 *					the recognition nodes so they can send back a descriptor.
 	 **/
	cv::Mat extract (std::string path, std::vector<std::string> separators,
										 												std::vector<std::string> names);
	cv::Mat merge (cv::Mat descriptor1, cv::Mat descriptor2);
	
	Histogram hist;
	cv::Mat _descriptor;
	std::string _training_results;
};
