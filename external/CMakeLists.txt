cmake_minimum_required(VERSION 3.12)
project(FLIQC_controller_external)

include(ExternalProject)

set(FLIQC_BASE_PATH)
set(FLIQC_BINARY_DIR)
message("[FLIQC_controller_core_external] Options:
         FLIQC_BASE_PATH:     ${FLIQC_BASE_PATH}
         FLIQC_BINARY_DIR:    ${FLIQC_BINARY_DIR}
")

ExternalProject_Add(
    LCQPow
    SOURCE_DIR ${FLIQC_BASE_PATH}/external/LCQPow
    BINARY_DIR ${FLIQC_BINARY_DIR}/external/LCQPow
    BUILD_COMMAND ${CMAKE_COMMAND} --build .
    INSTALL_COMMAND ""
)