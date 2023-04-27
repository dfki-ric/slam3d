

find_package(hiredis)

if (TARGET hiredis::hiredis)
    add_subdirectory(redis)
endif(TARGET hiredis::hiredis)