add_subdirectory(pcl)

find_package(libpointmatcher 1.3.1)
if(LIBPOINTMATCHER_INCLUDE_DIRS)
	add_subdirectory(pointmatcher)
else()
	message(WARNING, "Library 'pointmatcher' not found, the module will be excluded from the build.")
endif()

find_package(GDAL)
if(GDAL_INCLUDE_DIRS)
	add_subdirectory(gdal)
else()
	message(WARNING, "Library 'gdal' not found, the module will be excluded from the build.")
endif()
