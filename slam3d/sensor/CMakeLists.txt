add_subdirectory(pcl)

if(libpointmatcher_FOUND)
	add_subdirectory(pointmatcher)
else()
	message(WARNING, "Library 'pointmatcher' not found, the module will be excluded from the build.")
endif()

if(GDAL_INCLUDE_DIRS)
	add_subdirectory(gdal)
else()
	message(WARNING, "Library 'gdal' not found, the module will be excluded from the build.")
endif()

add_subdirectory(rtls_flares)
