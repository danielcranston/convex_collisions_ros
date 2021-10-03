cmake_minimum_required(VERSION 2.8)
project(convex_collision_msgs)

find_package(
  catkin REQUIRED
  COMPONENTS
    geometry_msgs
    message_generation
)

add_message_files(FILES ConvexHull.msg)

add_service_files(FILES CheckCollision.srv)

generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
)