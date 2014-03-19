FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/opencv_display/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/opencv_display/msg/__init__.py"
  "../src/opencv_display/msg/_LocaPose.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
