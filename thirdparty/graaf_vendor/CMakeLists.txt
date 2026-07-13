cmake_minimum_required(VERSION 3.16)
project(graaf_vendor)

find_package(ament_cmake REQUIRED)
include(FetchContent)

set(SKIP_TESTS       ON CACHE BOOL "" FORCE)
set(SKIP_BENCHMARKS  ON CACHE BOOL "" FORCE)
set(SKIP_EXAMPLES    ON CACHE BOOL "" FORCE)

FetchContent_Declare(graaf_src
  GIT_REPOSITORY https://github.com/bobluppes/graaf.git
  GIT_TAG        v1.1.1
)

FetchContent_MakeAvailable(graaf_src)

add_library(graaf INTERFACE)


target_include_directories(graaf INTERFACE
  "$<BUILD_INTERFACE:${graaf_src_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

install(DIRECTORY "${graaf_src_SOURCE_DIR}/include/"
        DESTINATION include)

install(TARGETS graaf
        EXPORT graaf_vendorTargets)

install(EXPORT graaf_vendorTargets
        NAMESPACE graaf_vendor::
        DESTINATION share/${PROJECT_NAME}/cmake)

ament_export_include_directories(include)
ament_export_targets(graaf_vendorTargets)

ament_package()
