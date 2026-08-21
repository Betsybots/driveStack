# - Config file for the FooBar package
# It defines the following variables
#  CNPY_INCLUDE_DIRS - include directories for FooBar
#  CNPY_LIBRARIES    - libraries to link against


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was project-config.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

set_and_check(CNPY_INCLUDE_DIR "${PACKAGE_PREFIX_DIR}/include/cnpy")

if(NOT TARGET cnpy)
  include("${CMAKE_CURRENT_LIST_DIR}/cnpy-targets.cmake")
endif()

set(CNPY_INCLUDE_DIRS ${CNPY_INCLUDE_DIR})

set(CNPY_LIBRARIES "cnpy")
