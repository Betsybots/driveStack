# - Config file for the MPPI-Generic package
# It defines the following variables
#  MPPI_INCLUDE_DIRS - include directories for MPPI-Generic
#  MPPI_INCLUDE_DIR  - include directories for MPPI-Generic
#  MPPI_LIBRARIES    - libraries to link against


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was Config.cmake.in                            ########

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
include(CMakeFindDependencyMacro)

set(MPPI_LIBRARIES "MPPI::MPPI")
set_and_check(MPPI_INCLUDE_DIRS "${PACKAGE_PREFIX_DIR}/include/")
set_and_check(MPPI_TARGET_CMAKE_FILE "${PACKAGE_PREFIX_DIR}/lib/cmake/MPPI-Generic/MPPI-GenericTargets.cmake")

# Find dependencies of MPPI-Generic
find_dependency(cnpy REQUIRED HINTS ${PACKAGE_PREFIX_DIR})
find_dependency(Eigen3 REQUIRED)

# Set up cmake targets and autodetection for CUDA architectures
include(${MPPI_TARGET_CMAKE_FILE})
include("${CMAKE_CURRENT_LIST_DIR}/MPPIGenericToolsConfig.cmake")

set(MPPI_INCLUDE_DIR ${MPPI_INCLUDE_DIRS})
check_required_components(MPPI-Generic)
