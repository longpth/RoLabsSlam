#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "Tesseract::libtesseract" for configuration "Debug"
set_property(TARGET Tesseract::libtesseract APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(Tesseract::libtesseract PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "CXX"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/debug/lib/tesseract53d.lib"
  )

list(APPEND _cmake_import_check_targets Tesseract::libtesseract )
list(APPEND _cmake_import_check_files_for_Tesseract::libtesseract "${_IMPORT_PREFIX}/debug/lib/tesseract53d.lib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
