cmake_minimum_required(VERSION 3.7)
# project for library
project(BooleanTopological)
# version
set(serial "1.0")
# release build
set(CMAKE_BUILD_TYPE "Release")

# need to install following libraries can be found via find_package
# - Boost
# - CGAL
find_package(Boost COMPONENTS random system thread filesystem chrono atomic date_time regex)
find_package(CGAL REQUIRED COMPONENTS Core)

# Include directories for library
set(INCLUDE_DIR_LIB
    ../../submodule/eigen
    ../../submodule/libigl/include
)

# Source files for library
set(CPP_FILES_LIB
    BooleanTopological.cpp
    read_OBJ.cpp
    write_OBJ.cpp
)

# External libraries for library
set(LINK_LIBS_LIB
    ${Boost_LIBRARIES}
    CGAL::CGAL
    CGAL::CGAL_Core
)

# Create shared library
add_library(BooleanTopological SHARED ${CPP_FILES_LIB})

# compile options
target_compile_options(BooleanTopological PUBLIC -O2 -Wall)
target_compile_features(BooleanTopological PUBLIC cxx_std_11)

# version
set_target_properties(BooleanTopological PROPERTIES version ${serial})

# Add a include files
target_include_directories(BooleanTopological PUBLIC ${INCLUDE_DIR_LIB})

# link with library
target_link_libraries(BooleanTopological PRIVATE ${LINK_LIBS_LIB})
