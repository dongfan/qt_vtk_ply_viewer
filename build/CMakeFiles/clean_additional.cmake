# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Release")
  file(REMOVE_RECURSE
  "CMakeFiles\\qt_vtk_ply_viewer_autogen.dir\\AutogenUsed.txt"
  "CMakeFiles\\qt_vtk_ply_viewer_autogen.dir\\ParseCache.txt"
  "qt_vtk_ply_viewer_autogen"
  )
endif()
