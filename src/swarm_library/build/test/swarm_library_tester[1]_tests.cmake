add_test( dummy_test.this_should_pass /home/fabrizzio/ros2_ws/src/swarm_bots/src/swarm_library/build/test/swarm_library_tester [==[--gtest_filter=dummy_test.this_should_pass]==] --gtest_also_run_disabled_tests)
set_tests_properties( dummy_test.this_should_pass PROPERTIES WORKING_DIRECTORY /home/fabrizzio/ros2_ws/src/swarm_bots/src/swarm_library/build/test SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==])
set( swarm_library_tester_TESTS dummy_test.this_should_pass)
