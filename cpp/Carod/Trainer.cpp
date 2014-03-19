#include "Trainer.h"

using namespace std;
using namespace cv;

void Trainer::train (string dir)
{
  try
  {
    if (bf::exists (dir))
    {
      if (bf::is_directory (dir))
      {
        bf::directory_iterator end_itr;
        for (bf::directory_iterator i (dir); i != end_itr; ++i)
        {
          if ( read_info (i->path().string()) )
          {
            load_images (i->path().string());
            train_object();
          }
          else
            cout << "Unable to find info.txt" << endl;
        }
        save_objects(dir);
      }
    }
  }
  catch (const bf::filesystem_error& ex)
  {
    cout << ex.what() << endl;
  }
}

bool Trainer::read_info (string dir)
{
  std::ostringstream info_path;
  info_path << dir << "/" << "info.txt";

  string line;
  std::string already_trained;
  vector<string> dims;
  ifstream file (info_path.str().c_str());
  if (file.is_open())
  {
    getline (file, _name);
    getline (file, line);
    boost::split (dims, line, boost::is_any_of (" "));
    cout << "Loading info: " << _name << endl;
    if (dims.size() == 3)
    {
      _w = atof (dims[0].c_str ());
      _h = atof (dims[1].c_str ());
      _d = atof (dims[2].c_str ());
    }
    else
    {
      cout << "not enough/too much dimensions, aborting" << endl;
      return false;
    }
    file.close();
  }
  else
  {
    cout << "Unable to open file" << endl;
    return false;
  }

  return true;
}

bool Trainer::load_images (string dir)
{
  Mat img, warped;

  namedWindow ("Select object corners");
  setMouseCallback ("Select object corners", mouse_cb);

  std::ostringstream img_path;
  std::ostringstream out_path;
  for(size_t i=0; i<NUM_TRAIN_IMG; ++i)
  {
    img_path << dir << "/" << "view_" << i << ".jpg" ;
    out_path << dir << "/" << "warp_" << i << ".jpg" ;

    if (!bf::exists (out_path.str ()))
    {
      cout << "No warped training images." << endl;
      img = imread (img_path.str());

      imshow ("Select object corners", img);
      cout << "Select object corners in that order: top left, top right, bottom left, bottom right. Then press any key to continue." << endl;
      waitKey(0);
      // rectify perspective for object
      if (i == 0 || i == 2)
        rect_img (img , warped, corners, _w, _h);
      else if (i==1 || i==3)
        rect_img (img , warped, corners, _d, _h);
      imwrite (out_path.str(), warped);
    }
    else
    {
      //cout << "Found warped training image." << endl;
      warped = imread (out_path.str());
    }

    _img[i] = warped;

    img_path.str ("");
    out_path.str ("");
  }

  destroyWindow ("Select object corners");

  return true;
}

void Trainer::train_object ()
{
  Object obj (_name, _w, _h, _d);
  vector<KeyPoint> kpts;
  Mat descs;

  for (size_t i=0; i<_img.size (); ++i) {
    //cout << _img[i].cols << " " << _img[i].rows << endl;
    M.extract (_img[i], kpts, descs);
    obj.add_img (_img[i]);
    obj.add_p3d_from_kpts (i, kpts, _img[i].cols, _img[i].rows);
    obj.add_descs (descs);
  }
  _objects.push_back (obj);
}

void Trainer::rect_img (Mat in, Mat &out, vector<Point2f> corners, float width, float height)
{
  //width *= 2;
  //height *= 2;

  // Corners in rectified image
  vector<Point2f> dest;
  dest.push_back (Point2f (0.0f, 0.0f));
  dest.push_back (Point2f (width, 0.0f));
  dest.push_back (Point2f (0.0f, height));
  dest.push_back (Point2f (width, height));

  Mat T = getPerspectiveTransform (corners, dest);
  warpPerspective (in, out, T, Size (width, height));
}

// Path is the path to the main dir containing the training sets
void Trainer::save_objects (string main_training_folder)
{
  // create cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
  cloud.reset (new pcl::PointCloud<pcl::PointXYZRGB>);

  std::ostringstream object_folder;
  for (size_t i=0; i<_objects.size (); ++i) {
    object_folder << main_training_folder << "/" << _objects[i]._name;
    _objects[i].save_to_pcd (object_folder.str());
    _objects[i].save_descriptors (object_folder.str());
    _objects[i].save_descriptors_with_key_points (object_folder.str());
    object_folder.str ("");
  }
}

Object Trainer::get_object_by_name (string name)
{
  int res = -1;

  for (size_t i=0; i<_objects.size (); ++i)
  {
    if (_objects[i]._name.compare (name) == 0 )
      res = i;
  }

  return _objects[res];
}

Trainer::Trainer ()
{
  _img.resize (NUM_TRAIN_IMG);
}

Trainer::~Trainer ()
{}
