FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/hand_detect/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/hand_detect/srv/__init__.py"
  "../src/hand_detect/srv/_HandDetection.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
