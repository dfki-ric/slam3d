add_subdirectory(boost)

find_package(cpprestsdk)
if (${cpprestsdk_FOUND})
    add_subdirectory(neo4j)
endif()


