FILE(REMOVE_RECURSE
  "../srv_gen"
  "../srv_gen"
  "../src/reco_3d/srv"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/reco_3d/OrientedBoundingBoxRecognition.h"
  "../srv_gen/cpp/include/reco_3d/IterativeClosestPointRecognition.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
