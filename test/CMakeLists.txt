catkin_add_gtest(test_chunked_kdtree src/test_chunked_kdtree.cpp)
target_link_libraries(test_chunked_kdtree ${catkin_LIBRARIES} ${PCL_LIBRARIES})
catkin_add_gtest(test_filter src/test_filter.cpp)
target_link_libraries(test_filter ${catkin_LIBRARIES})
catkin_add_gtest(test_nd src/test_nd.cpp)
target_link_libraries(test_nd ${catkin_LIBRARIES})
catkin_add_gtest(test_pf src/test_pf.cpp)
target_link_libraries(test_pf ${catkin_LIBRARIES})
catkin_add_gtest(test_point_types src/test_point_types.cpp)
target_link_libraries(test_point_types ${catkin_LIBRARIES} ${PCL_LIBRARIES})
catkin_add_gtest(test_point_cloud_random_sampler src/test_point_cloud_random_sampler.cpp)
target_link_libraries(test_point_cloud_random_sampler ${catkin_LIBRARIES} ${PCL_LIBRARIES})
catkin_add_gtest(test_quat src/test_quat.cpp)
target_link_libraries(test_quat ${catkin_LIBRARIES})
catkin_add_gtest(test_raycast src/test_raycast.cpp)
target_link_libraries(test_raycast ${catkin_LIBRARIES} ${PCL_LIBRARIES})
catkin_add_gtest(test_state_6dof src/test_state_6dof.cpp)
target_link_libraries(test_state_6dof ${catkin_LIBRARIES})
catkin_add_gtest(test_vec3 src/test_vec3.cpp)
target_link_libraries(test_vec3 ${catkin_LIBRARIES})
catkin_add_gtest(test_imu_measurement_model_gravity src/test_imu_measurement_model_gravity.cpp)
target_link_libraries(test_imu_measurement_model_gravity ${catkin_LIBRARIES})
catkin_add_gtest(test_motion_prediction_model_differential_drive src/test_motion_prediction_model_differential_drive.cpp)
target_link_libraries(test_motion_prediction_model_differential_drive ${catkin_LIBRARIES})

add_executable(performance_raycast src/performance_raycast.cpp)
target_link_libraries(performance_raycast ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})

add_rostest_gtest(test_debug_output tests/debug_output_rostest.test
    src/test_debug_output.cpp)
target_link_libraries(test_debug_output ${catkin_LIBRARIES})
add_dependencies(test_debug_output mcl_3dl)

add_rostest_gtest(test_expansion_resetting tests/expansion_resetting_rostest.test
    src/test_expansion_resetting.cpp)
target_link_libraries(test_expansion_resetting ${catkin_LIBRARIES})
add_dependencies(test_expansion_resetting mcl_3dl)

add_rostest_gtest(test_global_localization tests/global_localization_rostest.test
    src/test_global_localization.cpp)
target_link_libraries(test_global_localization ${catkin_LIBRARIES})
add_dependencies(test_global_localization mcl_3dl)

add_rostest_gtest(test_landmark tests/landmark_rostest.test
    src/test_landmark.cpp)
target_link_libraries(test_landmark ${catkin_LIBRARIES})
add_dependencies(test_landmark mcl_3dl)

add_rostest_gtest(test_mcl_3dl_compat tests/mcl_3dl_compat_rostest.test
    src/test_mcl_3dl_compat.cpp)
target_link_libraries(test_mcl_3dl_compat ${catkin_LIBRARIES})

add_rostest_gtest(test_mcl_3dl_compat_incompatible tests/mcl_3dl_compat_incompatible_rostest.test
    src/test_mcl_3dl_compat_incompatible.cpp)
target_link_libraries(test_mcl_3dl_compat_incompatible ${catkin_LIBRARIES})

add_rostest_gtest(test_transform_failure tests/transform_rostest.test
    src/test_transform_failure.cpp)
target_link_libraries(test_transform_failure ${catkin_LIBRARIES})
add_dependencies(test_transform_failure mcl_3dl)

if(MCL_3DL_EXTRA_TESTS)

  catkin_download_test_data(
      ${PROJECT_NAME}_short_test.bag
      https://openspur.org/~atsushi.w/dataset/mcl_3dl/short_test.bag
      MD5 664ecb2824d81247ce4b26c599d99230
      FILENAME test/short_test.bag)

  catkin_download_test_data(
      ${PROJECT_NAME}_short_test_ref.topic
      https://openspur.org/~atsushi.w/dataset/mcl_3dl/short_test_ref.topic
      MD5 6ad3b72f6c50952ad1120316d4702e71
      FILENAME test/short_test_ref.topic)

  configure_file(tests/localization_rostest.test.in
      ${PROJECT_BINARY_DIR}/test/tests/localization_rostest.test)
  configure_file(tests/tf_rostest.test.in
      ${PROJECT_BINARY_DIR}/test/tests/tf_rostest.test)

  add_executable(compare_pose
      src/compare_pose.cpp)
  target_link_libraries(compare_pose ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
  add_dependencies(compare_pose 
        ${PROJECT_NAME}_short_test.bag
        ${PROJECT_NAME}_short_test_ref.topic
        mcl_3dl)

  add_rostest(${PROJECT_BINARY_DIR}/test/tests/localization_rostest.test
    DEPENDENCIES
      compare_pose
      ${PROJECT_NAME}_short_test.bag
      ${PROJECT_NAME}_short_test_ref.topic
      mcl_3dl)
  add_rostest(${PROJECT_BINARY_DIR}/test/tests/localization_rostest.test
    ARGS without_imu:=true
    DEPENDENCIES
      compare_pose
      ${PROJECT_NAME}_short_test.bag
      ${PROJECT_NAME}_short_test_ref.topic
      mcl_3dl)

  add_rostest_gtest(compare_tf ${PROJECT_BINARY_DIR}/test/tests/tf_rostest.test
      src/compare_tf.cpp)
  target_link_libraries(compare_tf ${catkin_LIBRARIES})
  add_dependencies(compare_tf 
        ${PROJECT_NAME}_short_test.bag
        ${PROJECT_NAME}_short_test_ref.topic
        mcl_3dl
      )

else()

  message("To perform extra heavy tests for mcl_3dl, use -DMCL_3DL_EXTRA_TESTS:=ON")

endif()
