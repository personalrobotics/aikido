# Make sure pytest is installed
execute_process(
  COMMAND ${PYTHON_EXECUTABLE} -c "import pytest; print(pytest.__version__)"
  RESULT_VARIABLE pytest_not_found
  OUTPUT_VARIABLE pytest_version
  ERROR_QUIET
)
if(pytest_not_found)
  message(STATUS "Running the tests requires pytest. Please install it manually"
    " (try: ${PYTHON_EXECUTABLE} -m pip install pytest)"
  )
  set(AIKIDOPY_PYTEST_FOUND FALSE)
elseif(pytest_version VERSION_LESS 3.0)
  message(STATUS "Running the tests requires pytest >= 3.0. Found: ${pytest_version}"
    "Please update it (try: ${PYTHON_EXECUTABLE} -m pip install -U pytest)"
  )
  set(AIKIDOPY_PYTEST_FOUND FALSE)
else()
  set(AIKIDOPY_PYTEST_FOUND TRUE)
endif()

set(aikidopy_test_utils
  util.py
)

file(GLOB_RECURSE aikidopy_test_files "test_*.py")

# Add custom target to run the tests
add_custom_target(pytest
  COMMAND ${CMAKE_COMMAND} -E echo "Running pytest by: PYTHONPATH=${AIKIDO_AIKIDOPY_BUILD_DIR} ${PYTHON_EXECUTABLE} -m pytest [sources]"
  COMMAND PYTHONPATH=${AIKIDOPY_BUILD_DIR} ${PYTHON_EXECUTABLE} -m pytest ${aikidopy_test_files} -v
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  SOURCES ${aikidopy_test_files} ${aikidopy_test_utils}
)

#===============================================================================
# Usage:
#   aikidopy_add_test(test_name) # assumed source is test_name.py
#   aikidopy_add_test(test_name main.py)
#===============================================================================
function(aikidopy_add_test test_name) # ARGN for source file
  if(${ARGC} GREATER 1)
    set(source ${ARGN})
  else()
    set(source "${test_name}.py")
  endif()
  add_custom_target(${test_name}
    COMMAND ${CMAKE_COMMAND} -E echo "Running pytest by: PYTHONPATH=${AIKIDO_AIKIDOPY_BUILD_DIR} ${PYTHON_EXECUTABLE} ${source}"
    COMMAND PYTHONPATH=${AIKIDO_AIKIDOPY_BUILD_DIR} ${PYTHON_EXECUTABLE} ${source}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    SOURCES ${source}
  )
endfunction()

foreach(aikidopy_test_file ${aikidopy_test_files})
  get_filename_component(aikidopy_test_name ${aikidopy_test_file} NAME_WE)
  aikidopy_add_test(py_${aikidopy_test_name} ${aikidopy_test_file})
endforeach()
