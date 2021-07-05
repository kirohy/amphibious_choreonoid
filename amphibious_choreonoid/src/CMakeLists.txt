choreonoid_add_simple_controller(CameraController CameraController.cpp)
target_link_libraries(CameraController ${roscpp_LIBRARIES} ${image_transport_LIBRARIES})

choreonoid_add_simple_controller(JointController JointController.cpp)
add_dependencies(JointController ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(JointController ${roscpp_LIBRARIES})

choreonoid_add_simple_controller(RobotStateController RobotStateController.cpp)
target_link_libraries(RobotStateController ${roscpp_LIBRARIES} ${tf2_LIBRARIES} ${tf2_ros_LIBRARIES})

choreonoid_add_simple_controller(ThrusterController ThrusterController.cpp)
target_link_libraries(ThrusterController ${roscpp_LIBRARIES} CnoidFluidDynamicsPlugin)

choreonoid_add_simple_controller(TrackController TrackController.cpp)
target_link_libraries(TrackController ${roscpp_LIBRARIES})
