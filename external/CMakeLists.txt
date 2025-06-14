cmake_minimum_required(VERSION 3.12)
project(FLIQC_controller_external)

include(ExternalProject)

set(FLIQC_BASE_PATH)
set(FLIQC_BINARY_DIR)
message("[FLIQC_controller_core_external] Options:
         FLIQC_BASE_PATH:     ${FLIQC_BASE_PATH}
         FLIQC_BINARY_DIR:    ${FLIQC_BINARY_DIR}
         FLIQC_DEBUG_MODE:    ${FLIQC_DEBUG_MODE}
")

if (FLIQC_DEBUG_MODE)
    set(EXTERNAL_BUILD_TYPE "Debug")
else()
    set(EXTERNAL_BUILD_TYPE "") 
endif()

ExternalProject_Add(
    LCQPow
    SOURCE_DIR ${FLIQC_BASE_PATH}/external/LCQPow
    BINARY_DIR ${FLIQC_BINARY_DIR}/LCQPow
    CMAKE_ARGS -DCMAKE_BUILD_TYPE=${EXTERNAL_BUILD_TYPE}
    BUILD_COMMAND ${CMAKE_COMMAND} --build .
    INSTALL_COMMAND ""
)