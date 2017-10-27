cmake_minimum_required(VERSION 2.8.3)
project(Testing_repo)

find_package(orocos_kdl)

include_directories(
  ${orocos_kdl_INCLUDE_DIRS}
  include
)

link_directories(
  "/home/djole/Downloads/Master/R_&_D/KDL_GIT/KDL_install_dir/lib/"
)

add_executable(main
  src/main.cpp
  src/solver_vereshchagin.cpp
)

target_link_libraries(main ${orocos_kdl_LIBRARIES})
