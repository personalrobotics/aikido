cmake_minimum_required(VERSION 2.8)
project(r3)

set(R3_MODULE_NAME "${PROJECT_NAME}py")
add_definitions(-DR3_MODULE_NAME=${R3_MODULE_NAME})

find_package(Boost REQUIRED COMPONENTS python)
find_package(PythonLibs REQUIRED)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/Skeleton.cpp
)

add_library(${R3_MODULE_NAME} SHARED
  src/python/python.cpp
  src/python/python_Skeleton.cpp
)
target_link_libraries(${R3_MODULE_NAME}
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${PYTHON_LIBRARIES}
)
set_target_properties(${R3_MODULE_NAME} PROPERTIES
  PREFIX ""
  SUFFIX ".so"
)
