FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/hand_detect/srv"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
