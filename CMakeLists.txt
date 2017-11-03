cmake_minimum_required(VERSION 2.8.3)
project(Testing_repo)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(orocos_kdl)

include_directories(
  ${orocos_kdl_INCLUDE_DIRS}
  include
)

link_directories(
  "/home/djole/Downloads/Master/R_&_D/KDL_GIT/KDL_install_dir/lib/"
)

#add_executable(old_main
 # src/old_main.cpp
  #src/solver_vereshchagin.cpp
#)

add_executable(main
  src/main.cpp
  src/solver_vereshchagin.cpp
)

target_link_libraries(main ${orocos_kdl_LIBRARIES})
#target_link_libraries(old_main ${orocos_kdl_LIBRARIES})
