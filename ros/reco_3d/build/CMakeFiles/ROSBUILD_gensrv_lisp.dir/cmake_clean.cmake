FILE(REMOVE_RECURSE
  "../srv_gen"
  "../srv_gen"
  "../src/reco_3d/srv"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/OrientedBoundingBoxRecognition.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_OrientedBoundingBoxRecognition.lisp"
  "../srv_gen/lisp/IterativeClosestPointRecognition.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_IterativeClosestPointRecognition.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
