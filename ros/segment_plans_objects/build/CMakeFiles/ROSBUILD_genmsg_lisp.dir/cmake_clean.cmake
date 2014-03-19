FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/segment_plans_objects/msg"
  "../src/segment_plans_objects/srv"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/PointCloudArray.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_PointCloudArray.lisp"
  "../msg_gen/lisp/ImageArray.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_ImageArray.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
