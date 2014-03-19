FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/seg_plans_objs/msg"
  "../src/seg_plans_objs/srv"
  "CMakeFiles/test"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/test.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
