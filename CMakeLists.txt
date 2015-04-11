cmake_minimum_required(VERSION 2.8)
project(r3)

find_library(Boost REQUIRED COMPONENTS python)
find_library(PythonLibs REQUIRED)

message(STATUS ${PYTHONLIBS_FOUND})
message(STATUS ${PYTHON_INCLUDE_DIRS})
message(STATUS ${PYTHON_LIBRARIES})

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/Skeleton.cpp
)

add_library(${PROJECT_NAME}py
  src/python/python.cpp
  src/python/python_Skeleton.cpp
)
target_link_libraries(${PROJECT_NAME}py
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${PYTHON_LIBRARIES}
)
set_target_properties(${PROJECT_NAME}py PROPERTIES
  OUTPUT_PREFIX ""
  OUTPUT_SUFFIX ".so"
)
