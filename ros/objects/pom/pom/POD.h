#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "Object.h"
#include "Pipeline2D.h"
#include "PipelineGeom.h"

class POD
{
  public:
    POD ();
    /**
    *   \brief Add an object to the detector object list.
    *   \param object An Object class instance describing the model of an object.
    */
    void setObject (Object object);
    /**
    *   \brief Load a model file and turns it into a Object instance, 
    *           then add the object to the detector object list.
    *   \param path The path to a model file describing an object.
    */    
    void loadObject (std::string path);
    /**
    *   \brief Same as loadObject but process a file containing paths and load
    *   .       all objects.
    *   \param list_path A file containing a list of paths to load.
    *   \return Number of objects loaded
    */
    int loadObjectsFromList (std::string list_path);
    /**
    *   \brief Provide the intrinsic parameters of the camera to the detector.
    *   \param K The intrinsic parameters matrix.
    */
    void setIntrinsic (cv::Mat K);
    /**
    *   \brief Provide the intrinsic parameters of the camera to the detector.
    *   \param path A path to a .yaml file containing the intrinsic parameters
    *           in the OpenCV format.
    */
    void loadIntrinsic (std::string path);
    /**
    *   \brief Returns true if the intrinsic parameters are set. False otherwise.
    */
    bool isIntrinsicSet();
    /**
    *   \brief Returns the object name from its number.
    *   \param object_number The object number in the vector of objects.
    */
    std::string objectName (int object_number);
    /**
    *   \brief Takes an image, extract features and match them to the provided
    *           object models. Returns the name and poses of found objects.
    *   \param image An opencv image of a scene.
    *   \param poses A vector of recognised objects poses.
    *   \param names A vector containing the recognised objects names.
    */
    void process (const cv::Mat image, std::vector<cv::Mat> &poses, std::vector<std::string> &names);
    /**
    *   \brief Proceed as be previous one, except the image and extracted features
    *           are save in a View instance.
    */
    void process (View current, std::vector<cv::Mat> &poses, std::vector<std::string> &names);

  private:
    /**
    *   \brief From an image, extract the features and save them in a View instance.
    *   \param image An OpenCV image of a scene.
    */
    View createView (cv::Mat image);
    void match (View current, Object object, std::vector<cv::DMatch> &matches);
    cv::Mat computePose (View current, Object object, std::vector<cv::DMatch> matches);
    void match2points (std::vector<cv::KeyPoint> keypoints, std::vector<cv::Point3f> points, std::vector<cv::DMatch> matches,
                        std::vector<cv::Point2f> &points2d, std::vector<cv::Point3f> &points3d);

    cv::Mat K_;
    bool is_intrinsic_set_;
    std::vector<Object> objects_;
    Pipeline2D pipeline2d_;
    PipelineGeom pipelineGeom_;
};
