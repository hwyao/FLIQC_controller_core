cmake_minimum_required(VERSION 3.12)
project(FLIQC_controller_core)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set the options
if(${CMAKE_CURRENT_SOURCE_DIR} STREQUAL ${CMAKE_SOURCE_DIR})
    set(FLIQC_BUILD_EXAMPLE_DEFAULT ON)
else()
    set(FLIQC_BUILD_EXAMPLE_DEFAULT OFF)
endif()

if("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
    set(FLIQC_DEBUG_MODE_DEFAULT ON)
else()
    set(FLIQC_DEBUG_MODE_DEFAULT OFF)
endif()

option(FLIQC_BUILD_EXAMPLE "Build the example" ${FLIQC_BUILD_EXAMPLE_DEFAULT})
option(FLIQC_DEBUG_MODE "Build with debug mode" ${FLIQC_DEBUG_MODE_DEFAULT})

# display the options
message(STATUS 
  "[FLIQC_controller_core] Options:
         FLIQC_BUILD_EXAMPLE:     ${FLIQC_BUILD_EXAMPLE}
            FLIQC_DEBUG_MODE:     ${FLIQC_DEBUG_MODE}
")

# Find the external libraries
find_package(Eigen3 REQUIRED)

# Add and build external projects in configure time
# You can use ExternalProject_Add here, but it will be then at compile time, which will case a bit problem by parallel build
execute_process(
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_SOURCE_DIR}/external/build
)
execute_process(
    COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .. 
            -DFLIQC_BASE_PATH=${CMAKE_CURRENT_SOURCE_DIR}
            -DFLIQC_BINARY_DIR=${CMAKE_BINARY_DIR}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/external/build
)
execute_process(
    COMMAND ${CMAKE_COMMAND} --build .
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/external/build
)

add_library(qpOASES_lib SHARED IMPORTED)
set_target_properties(qpOASES_lib PROPERTIES
  IMPORTED_LOCATION ${CMAKE_BINARY_DIR}/external/LCQPow/lib/${CMAKE_FIND_LIBRARY_PREFIXES}qpOASES${CMAKE_SHARED_LIBRARY_SUFFIX}
  INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_BINARY_DIR}/external/LCQPow/external/src/qpoases/include
  IMPORTED_NO_SONAME TRUE
)

add_library(osqp_lib SHARED IMPORTED)
set_target_properties(osqp_lib PROPERTIES
  IMPORTED_LOCATION ${CMAKE_BINARY_DIR}/external/LCQPow/lib/${CMAKE_FIND_LIBRARY_PREFIXES}osqp${CMAKE_SHARED_LIBRARY_SUFFIX}
  INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_BINARY_DIR}/external/LCQPow/external/src/osqp/include
)

add_library(LCQPow_lib SHARED IMPORTED)
set_target_properties(LCQPow_lib PROPERTIES
  IMPORTED_LOCATION ${CMAKE_BINARY_DIR}/external/LCQPow/lib/${CMAKE_FIND_LIBRARY_PREFIXES}lcqpow${CMAKE_SHARED_LIBRARY_SUFFIX}
  INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_SOURCE_DIR}/external/LCQPow/include
)

# add the debug flags
if(FLIQC_DEBUG_MODE)
  add_compile_options(-g)
endif()

add_library(FLIQC_controller_core SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/src/LCQPow_bridge.cpp
)
target_include_directories(FLIQC_controller_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(FLIQC_controller_core PUBLIC
  "$<LINK_GROUP:RESCAN,LCQPow_lib,osqp_lib,qpOASES_lib>"
  Eigen3::Eigen
)

# add the executable
if (FLIQC_BUILD_EXAMPLE)
    list(APPEND FLIQC_controller_core_EXAMPLE
        ${CMAKE_CURRENT_SOURCE_DIR}/example/Example_LCQPow.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/example/Example_LCQPow_bridge.cpp
    )

    foreach(_source IN ITEMS ${FLIQC_controller_core_EXAMPLE})
      get_filename_component(_name ${_source} NAME_WE)
      add_executable(${_name} ${_source})
      target_link_libraries(${_name} PRIVATE FLIQC_controller_core LCQPow_lib osqp_lib qpOASES_lib)
      target_include_directories(${_name} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
    endforeach()
endif()
    