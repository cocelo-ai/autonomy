#----------------------------------------------------------------
# Generated CMake target import file for configuration "None".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "BT::behaviortree_cpp_v3" for configuration "None"
set_property(TARGET BT::behaviortree_cpp_v3 APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(BT::behaviortree_cpp_v3 PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_NONE "Boost::coroutine;ament_index_cpp::ament_index_cpp"
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/libbehaviortree_cpp_v3.so"
  IMPORTED_SONAME_NONE "libbehaviortree_cpp_v3.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS BT::behaviortree_cpp_v3 )
list(APPEND _IMPORT_CHECK_FILES_FOR_BT::behaviortree_cpp_v3 "${_IMPORT_PREFIX}/lib/libbehaviortree_cpp_v3.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
