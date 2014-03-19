FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/seg_plans_objs/msg"
  "../src/seg_plans_objs/srv"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/seg_plans_objs/msg/__init__.py"
  "../src/seg_plans_objs/msg/_PointCloudArray.py"
  "../src/seg_plans_objs/msg/_ImageArray.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
