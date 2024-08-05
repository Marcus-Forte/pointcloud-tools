add_executable(demo_segmentation
segmentation.cpp)

add_executable(demo_point
labeled_point.cpp)

target_link_libraries(demo_segmentation ${PCL_LIBRARIES})
target_link_libraries(demo_point ${PCL_LIBRARIES})