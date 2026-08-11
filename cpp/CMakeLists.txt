cmake_minimum_required(VERSION 3.22)
project(par6_shim VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(pinocchio REQUIRED)
find_package(toppra REQUIRED)

set(PAR6_SHIM_SOURCES src/par6_shim.cpp src/par6_traj.cpp)

# Shared library — what pinokin-sys links by default (Pinocchio itself is
# only distributed as shared libs on conda-forge).
add_library(par6_shim SHARED ${PAR6_SHIM_SOURCES})

# Static archive of the shim itself (Pinocchio still linked dynamically);
# used by packaging/cross builds that want a single relocatable object.
add_library(par6_shim_static STATIC ${PAR6_SHIM_SOURCES})
set_target_properties(par6_shim_static PROPERTIES
  OUTPUT_NAME par6_shim
  POSITION_INDEPENDENT_CODE ON)

foreach(tgt par6_shim par6_shim_static)
  target_include_directories(${tgt} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  # Minimal Pinocchio targets: instantiated algorithms + URDF parser.
  # (The pinocchio::pinocchio umbrella drags in collision/casadi/extra.)
  target_link_libraries(${tgt} PUBLIC
    pinocchio::pinocchio_default
    pinocchio::pinocchio_parsers
    toppra::toppra)
  target_compile_options(${tgt} PRIVATE -Wall -Wextra)
endforeach()

include(GNUInstallDirs)
install(TARGETS par6_shim par6_shim_static
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(FILES include/par6_shim.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
