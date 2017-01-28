#edit the following line to add the librarie's header files
FIND_PATH(library_name_INCLUDE_DIR header_file /usr/include/project_name /usr/local/include/project_name)

FIND_LIBRARY(library_name_LIBRARY
    NAMES library_name
    PATHS /usr/lib /usr/local/lib /usr/local/lib/library_name) 

IF (library_name_INCLUDE_DIR AND library_name_LIBRARY)
   SET(library_name_FOUND TRUE)
ENDIF (library_name_INCLUDE_DIR AND library_name_LIBRARY)

IF (library_name_FOUND)
   IF (NOT library_name_FIND_QUIETLY)
      MESSAGE(STATUS "Found library_name: ${library_name_LIBRARY}")
   ENDIF (NOT library_name_FIND_QUIETLY)
ELSE (library_name_FOUND)
   IF (library_name_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find library_name")
   ENDIF (library_name_FIND_REQUIRED)
ENDIF (library_name_FOUND)

