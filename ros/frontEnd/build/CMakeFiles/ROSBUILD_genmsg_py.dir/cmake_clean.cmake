FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/frontEnd/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/frontEnd/msg/__init__.py"
  "../src/frontEnd/msg/_CloudArray.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
