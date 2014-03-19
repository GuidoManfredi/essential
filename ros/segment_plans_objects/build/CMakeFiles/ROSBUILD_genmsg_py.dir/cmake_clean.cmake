FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/segment_plans_objects/msg"
  "../src/segment_plans_objects/srv"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/segment_plans_objects/msg/__init__.py"
  "../src/segment_plans_objects/msg/_PointCloudArray.py"
  "../src/segment_plans_objects/msg/_ImageArray.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
