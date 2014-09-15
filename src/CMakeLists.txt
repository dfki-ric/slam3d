rock_library(slam3d
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(slam3d_bin Main.cpp
    DEPS slam3d)

