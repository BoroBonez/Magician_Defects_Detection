# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_computing_point_cloud_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED computing_point_cloud_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(computing_point_cloud_FOUND FALSE)
  elseif(NOT computing_point_cloud_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(computing_point_cloud_FOUND FALSE)
  endif()
  return()
endif()
set(_computing_point_cloud_CONFIG_INCLUDED TRUE)

# output package information
if(NOT computing_point_cloud_FIND_QUIETLY)
  message(STATUS "Found computing_point_cloud: 0.0.0 (${computing_point_cloud_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'computing_point_cloud' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${computing_point_cloud_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(computing_point_cloud_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${computing_point_cloud_DIR}/${_extra}")
endforeach()
