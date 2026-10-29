# Enforce the workspace language contract after ROS generators create targets.
# Jazzy generators explicitly set CXX_STANDARD=17, after project() has returned.
if(CMAKE_VERSION VERSION_LESS 3.19)
  message(FATAL_ERROR "The workspace C++20 target policy requires CMake 3.19 or newer")
endif()
include_guard(DIRECTORY)

function(spr_require_cxx20_targets directory)
  get_property(spr_targets DIRECTORY "${directory}" PROPERTY BUILDSYSTEM_TARGETS)
  foreach(spr_target IN LISTS spr_targets)
    get_target_property(spr_target_type "${spr_target}" TYPE)
    if(spr_target_type MATCHES "^(EXECUTABLE|STATIC_LIBRARY|SHARED_LIBRARY|MODULE_LIBRARY|OBJECT_LIBRARY)$")
      set_target_properties("${spr_target}" PROPERTIES
        CXX_STANDARD 20
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS OFF)
    endif()
  endforeach()
  get_property(spr_subdirectories DIRECTORY "${directory}" PROPERTY SUBDIRECTORIES)
  foreach(spr_subdirectory IN LISTS spr_subdirectories)
    spr_require_cxx20_targets("${spr_subdirectory}")
  endforeach()
endfunction()

cmake_language(DEFER CALL spr_require_cxx20_targets "${CMAKE_SOURCE_DIR}")
