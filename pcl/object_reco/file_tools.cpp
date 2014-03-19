#include "file_tools.h"

void
createDir (std::string & dir)
{
  bf::path dir_path = dir;
  if (!bf::exists (dir_path))
  	bf::create_directory (dir_path);
}

void
removeDir (std::string & dir)
{
  bf::path dir_path = dir;
  if (bf::exists (dir_path))
  	bf::remove_all (dir_path);
}

void
getFileByNameAndExt (bf::path & dir, std::vector<std::string> & filenames, std::string basename, std::string ext)
{
	filenames.clear();

  bf::directory_iterator end_itr;
  for (bf::directory_iterator itr (dir); itr != end_itr; ++itr)
  {
    if (!(bf::is_directory (*itr)))
    {
      std::vector < std::string > strs;
      std::vector < std::string > strs_;

#if BOOST_FILESYSTEM_VERSION == 3
      std::string file = (itr->path ().filename ()).string();
#else
      std::string file = (itr->path ()).filename ();
#endif
      boost::split (strs, file, boost::is_any_of ("."));
      boost::split (strs_, file, boost::is_any_of ("_"));

      std::string extension = strs[strs.size () - 1];

      if (extension == ext && (strs_[0].compare (basename) == 0))
      {
#if BOOST_FILESYSTEM_VERSION == 3
        filenames.push_back ((itr->path ().filename ()).string());
#else
        filenames.push_back ((itr->path ()).filename ());
#endif
      }
    }
  }
}

void
getDirRec (bf::path & dir, std::string & rel_path_so_far, std::vector<std::string> & relative_paths)
{
  bf::directory_iterator end_itr;
  for (bf::directory_iterator itr (dir); itr != end_itr; ++itr)
  {
    //check if its a directory, then get models in it
    if (bf::is_directory (*itr))
    {
#if BOOST_FILESYSTEM_VERSION == 3
      std::string so_far = rel_path_so_far + (itr->path ().filename ()).string() + "/";
#else
      std::string so_far = rel_path_so_far + (itr->path ()).filename () + "/";
#endif

      bf::path curr_path = itr->path ();

      if (isleafDirectory (curr_path))
      {
#if BOOST_FILESYSTEM_VERSION == 3
        std::string path = rel_path_so_far + (itr->path ().filename ()).string();
#else
        std::string path = rel_path_so_far + (itr->path ()).filename ();
#endif
        relative_paths.push_back (path);
      }
      else
      {
        getDirRec (curr_path, so_far, relative_paths);
      }
    }
  }
}

bool
isleafDirectory (bf::path & path)
{
  bf::directory_iterator end_itr;
  bool no_dirs_inside = true;
  for (bf::directory_iterator itr (path); itr != end_itr; ++itr)
  {
    if (bf::is_directory (*itr))
    {
      no_dirs_inside = false;
    }
  }

  return no_dirs_inside;
}
