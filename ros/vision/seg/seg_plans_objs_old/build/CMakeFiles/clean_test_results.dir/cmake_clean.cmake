FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/seg_plans_objs/msg"
  "../src/seg_plans_objs/srv"
  "CMakeFiles/clean_test_results"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/clean_test_results.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
