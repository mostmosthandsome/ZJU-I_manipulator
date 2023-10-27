cmake_minimum_required(VERSION 3.11)
#list(APPEND CMAKE_MODULE_PATH remoteAPI/build/jsoncons/cmake)
project(manipulator)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
include_directories(/usr/include/python3.10)
add_subdirectory(pybind11)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# add_library(HMath STATIC src/Hmath.cpp)
# target_include_directories(HMath PUBLIC include)
# set_property(TARGET HMath PROPERTY POSITION_INDEPENDENT_CODE ON)

# add_library(HIKsolver SHARED src/HIKsolver.cpp)
# target_include_directories(HIKsolver PRIVATE include)
# target_link_libraries(HIKsolver PRIVATE HMath)

# add_library(HTrajectory SHARED src/Htrajectory.cpp)
# target_include_directories(HTrajectory PUBLIC include)
# target_link_libraries(HTrajectory PRIVATE HMath)

# add_executable(main src/main.cpp)
# target_link_libraries(main PRIVATE HIKsolver PRIVATE HMath PRIVATE HTrajectory)
include_directories(include)
include_directories(/usr/include/eigen3)

pybind11_add_module(handsome src/main.cpp)