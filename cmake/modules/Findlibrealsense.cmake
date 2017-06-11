###############################################################################
# Find CrossPlatform Real Sense library from Intel
#
#     find_package(librealsense`)
#
# Variables defined by this module:
#
#  LIBREALSENSE_FOUND                 True if librealsense was found
#  LIBREALSENSE_INCLUDE_DIRS          The location(s) of RealSense SDK headers
#  LIBREALSENSE_LIBRARIES             Libraries needed to use RealSense SDK

if(WIN32)
	find_path(LIBREALSENSE_DIR "include/librealsense/rs.h"
			  PATHS "$ENV{LIBREALSENSE_DIR}"
					"$ENV{PROGRAMFILES}/librealsense"
					"$ENV{PROGRAMW6432}/librealsense"
					"C:/Program Files (x86)/librealsense"
					"C:/Program Files/librealsense")
					
	if(LIBREALSENSE_DIR)
		# Include directories
		set(LIBREALSENSE_INCLUDE_DIRS ${LIBREALSENSE_DIR}/include)
		mark_as_advanced(LIBREALSENSE_INCLUDE_DIRS)

		# Libraries
		set(LIBREALSENSE_RELEASE_NAME realsense.lib)
		set(LIBREALSENSE_DEBUG_NAME realsense-d.lib)

		find_library(LIBREALSENSE_LIBRARY
				   NAMES ${LIBREALSENSE_RELEASE_NAME}
				   PATHS "${LIBREALSENSE_DIR}/lib/")
		find_library(LIBREALSENSE_LIBRARY_DEBUG
				   NAMES ${LIBREALSENSE_DEBUG_NAME} ${LIBREALSENSE_RELEASE_NAME}
				   PATHS "${LIBREALSENSE_DIR}/lib/")
		
		if(NOT LIBREALSENSE_LIBRARY_DEBUG)
			set(LIBREALSENSE_LIBRARY_DEBUG ${LIBREALSENSE_LIBRARY})
		endif()
		mark_as_advanced(LIBREALSENSE_LIBRARY LIBREALSENSE_LIBRARY_DEBUG)

		set(LIBREALSENSE_LIBRARIES optimized ${LIBREALSENSE_LIBRARY} debug ${LIBREALSENSE_LIBRARY_DEBUG})
	endif()


elseif(APPLE)
        find_path(LIBREALSENSE_DIR "include/librealsense/rs.h"
			  PATHS "$ENV{LIBREALSENSE_INCLUDE_DIR}"
                                        "/usr/local"
                                        "/usr")

     MESSAGE(STATUS ${LIBREALSENSE_DIR})

	if(LIBREALSENSE_DIR)
                set(LIBREALSENSE_INCLUDE_DIRS ${LIBREALSENSE_DIR}/include)
		mark_as_advanced(LIBREALSENSE_INCLUDE_DIRS)
	
		# Libraries
                set(LIBREALSENSE_RELEASE_NAME librealsense.dylib)
                set(LIBREALSENSE_DEBUG_NAME librealsense-d.dylib)
		
		find_library(LIBREALSENSE_LIBRARY
				   NAMES ${LIBREALSENSE_RELEASE_NAME}
				   PATHS 	"/usr/lib/"
							"/usr/local/lib")
		
		find_library(LIBREALSENSE_LIBRARY_DEBUG
				   NAMES ${LIBREALSENSE_DEBUG_NAME} ${LIBREALSENSE_RELEASE_NAME}
				   PATHS 	"/usr/lib/"
							"/usr/local/lib")
		
		if(NOT LIBREALSENSE_LIBRARY_DEBUG)
			set(LIBREALSENSE_LIBRARY_DEBUG ${LIBREALSENSE_LIBRARY})
		endif()
		mark_as_advanced(LIBREALSENSE_LIBRARY LIBREALSENSE_LIBRARY_DEBUG)
	
		set(LIBREALSENSE_LIBRARIES optimized ${LIBREALSENSE_LIBRARY} debug ${LIBREALSENSE_LIBRARY_DEBUG})
	endif(LIBREALSENSE_DIR)
	
elseif(UNIX)
        find_path(LIBREALSENSE_DIR "include/librealsense/rs.h"
			  PATHS "$ENV{LIBREALSENSE_INCLUDE_DIR}"
                                        "/usr/local"
                                        "/usr")

	if(LIBREALSENSE_DIR)
                set(LIBREALSENSE_INCLUDE_DIRS ${LIBREALSENSE_DIR}/include)
		mark_as_advanced(LIBREALSENSE_INCLUDE_DIRS)
	
		# Libraries
                set(LIBREALSENSE_RELEASE_NAME librealsense.so)
                set(LIBREALSENSE_DEBUG_NAME librealsense-d.so)
		
		find_library(LIBREALSENSE_LIBRARY
				   NAMES ${LIBREALSENSE_RELEASE_NAME}
				   PATHS 	"/usr/lib/"
							"/usr/local/lib")
		
		find_library(LIBREALSENSE_LIBRARY_DEBUG
				   NAMES ${LIBREALSENSE_DEBUG_NAME} ${LIBREALSENSE_RELEASE_NAME}
				   PATHS 	"/usr/lib/"
							"/usr/local/lib")
		
		if(NOT LIBREALSENSE_LIBRARY_DEBUG)
			set(LIBREALSENSE_LIBRARY_DEBUG ${LIBREALSENSE_LIBRARY})
		endif()
		mark_as_advanced(LIBREALSENSE_LIBRARY LIBREALSENSE_LIBRARY_DEBUG)
	
		set(LIBREALSENSE_LIBRARIES optimized ${LIBREALSENSE_LIBRARY} debug ${LIBREALSENSE_LIBRARY_DEBUG})
	endif(LIBREALSENSE_DIR)
endif()

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(LIBREALSENSE
	FOUND_VAR LIBREALSENSE_FOUND
	REQUIRED_VARS LIBREALSENSE_LIBRARIES LIBREALSENSE_INCLUDE_DIRS
	)
