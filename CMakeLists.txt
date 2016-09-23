cmake_minimum_required(VERSION 2.8)

include(cmake/base.cmake)
include(cmake/boost.cmake)

set(PROJECT_NAME mc_nao)
set(PROJECT_DESCRIPTION "Controller for vision integration (SLAM) for mc_rtc")
set(PROJECT_URL https://gite.lirmm.fr/atanguy/mc_vision_controllers)

setup_project()

# Enable C++11
if(NOT WIN32)
  add_definitions(-std=c++0x)
endif()

# Use jrl-cmakemoduels to get mc_rtc depency
set(PKG_CONFIG_ADDITIONAL_VARIABLES "${PKG_CONFIG_ADDITIONAL_VARIABLES} libdir")
add_required_dependency(mc_rtc)
add_required_dependency(mc_rbdyn)
add_required_dependency(mc_rbdyn_urdf_cpp)

# for Sophus
add_definitions(-Wno-ignored-qualifiers)
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include/Sophus")


configure_file(${PROJECT_SOURCE_DIR}/include/mc_nao/config.h.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/include/mc_nao/config.h)

# XXX hardcoded
set(MC_ROBOTS_INSTALL_PREFIX /usr/local/lib/mc_robots)

set(robot_SRC src/nao.cpp)
set(robot_HDR include/mc_nao/nao.h)
set(robot_base nao)
add_library(${robot_base} SHARED ${robot_SRC} ${robot_HDR})
set_target_properties(${robot_base} PROPERTIES COMPILE_FLAGS "-DMC_ROBOTS_EXPORTS" PREFIX "")
target_link_libraries(${robot_base} mc_rbdyn ${Boost_LIBRARIES})
pkg_config_use_dependency(${robot_base} mc_rbdyn_urdf_cpp)
install(TARGETS ${robot_base} DESTINATION ${MC_ROBOTS_INSTALL_PREFIX})


setup_project_finalize()
