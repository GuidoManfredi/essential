FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/reco_texture/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/reco_texture/srv/__init__.py"
  "../src/reco_texture/srv/_TexturedRecognition.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
