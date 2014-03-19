#include "file_tools.h"

using namespace std;

void
createDir (string & dir)
{
  bf::path dir_path = dir;
  if (!bf::exists (dir_path))
  	bf::create_directory (dir_path);
}

void
removeDir (string & dir)
{
  bf::path dir_path = dir;
  if (bf::exists (dir_path))
  	bf::remove_all (dir_path);
}

void
getFileByNameAndExt (bf::path dir, vector<string> & filenames, string basename, string ext)
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

bool
isleafDirectory (bf::path & path)
{
  bf::directory_iterator end_itr;
  bool no_dirs_inside = true;
  for (bf::directory_iterator itr (path); itr != end_itr; ++itr)
    if (bf::is_directory (*itr))
      no_dirs_inside = false;

  return no_dirs_inside;
}

bool get_subdirectories (string dir, vector<string> &subdirs)
{
  subdirs.clear();
  bf::path p;
  vector<bf::path> v;

  try
  {
    if (bf::exists (dir))
    {
      if (bf::is_directory (dir))
      {
        copy(bf::directory_iterator(p), bf::directory_iterator(), back_inserter(v));
      }
    }
  }

  catch (const bf::filesystem_error& ex)
  {
    cout << ex.what() << '\n';
  }

  bf::directory_iterator end_itr;
  for (bf::directory_iterator i (dir); i != end_itr; ++i)
  {

  }
}
