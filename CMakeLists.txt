cmake_minimum_required(VERSION 2.8.3)
project(Testing_repo)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()


find_package(orocos_kdl)
find_package(kdl_parser)

include_directories(
  ${orocos_kdl_INCLUDE_DIRS}
  include
)

link_directories(
  "/home/djole/Downloads/Master/R_&_D/KDL_GIT/KDL_install_dir/lib/"
)

add_executable(main
  src/main.cpp
  src/vereshchagin_with_friction.cpp
  src/solver_vereshchagin.cpp
)

target_link_libraries(main
    ${orocos_kdl_LIBRARIES}
    ${kdl_parser_LIBRARIES}
)
