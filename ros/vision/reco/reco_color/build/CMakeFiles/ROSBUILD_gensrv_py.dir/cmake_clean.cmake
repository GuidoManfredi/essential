FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/reco_color/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/reco_color/srv/__init__.py"
  "../src/reco_color/srv/_HistogramRecognition.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
