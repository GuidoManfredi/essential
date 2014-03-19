FILE(REMOVE_RECURSE
  "../srv_gen"
  "../srv_gen"
  "../src/reco_3d/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/reco_3d/srv/__init__.py"
  "../src/reco_3d/srv/_OrientedBoundingBoxRecognition.py"
  "../src/reco_3d/srv/_IterativeClosestPointRecognition.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
