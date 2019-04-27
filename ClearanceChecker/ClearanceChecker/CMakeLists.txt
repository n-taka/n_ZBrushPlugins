cmake_minimum_required(VERSION 3.7)
# project for library
project(ClearanceChecker)
# version
set(serial "1.0")
# release build
set(CMAKE_BUILD_TYPE "Release")

# need to install following libraries can be found via find_package
# - Boost
# - CGAL
####
# for windows (find boost installed via vcpkg)
# add 
# "-DCMAKE_TOOLCHAIN_FILE=C:/src/vcpkg/scripts/buildsystems/vcpkg.cmake -DVCPKG_TARGET_TRIPLET=x64-windows"
# to argument passed to cmake
# edit CGALConfig.cmake following the diff https://github.com/CGAL/cgal/commit/4782ea16656e0896dda83840397020c7d2b7d7b8#diff-2e556f99209b0fa55049bf5a5b8302cc
####
# find_package(Boost COMPONENTS random system thread filesystem chrono atomic date_time regex REQUIRED)
# find_package(CGAL CONFIG REQUIRED COMPONENTS Core)

# Include directories for library
if(WIN32)
    set(INCLUDE_DIR_LIB
        # ${CGAL_INCLUDE_DIRS}
        ../../submodule/eigen
        ../../submodule/libigl/include
    )
endif(WIN32)
if(UNIX)
    set(INCLUDE_DIR_LIB
        ../../submodule/eigen
        ../../submodule/libigl/include
    )
endif(UNIX)

# Source files for library
set(CPP_FILES_LIB
    ClearanceChecker.cpp
    read_OBJ.cpp
    write_OBJ.cpp
    parseParams.cpp
)

# External libraries for library
set(LINK_LIBS_LIB
    # ${Boost_LIBRARIES}
    # CGAL::CGAL
    # CGAL::CGAL_Core
)

# Create shared library
add_library(ClearanceChecker SHARED ${CPP_FILES_LIB})

# compile options
target_compile_options(ClearanceChecker PUBLIC -O2)
target_compile_features(ClearanceChecker PUBLIC cxx_std_11)
if(WIN32)
    # target_compile_options(ClearanceChecker PUBLIC -DCGAL_HEADER_ONLY)
    target_compile_options(ClearanceChecker PUBLIC -DNOMINMAX)
endif(WIN32)

# version
set_target_properties(ClearanceChecker PROPERTIES version ${serial})

# Add a include files
target_include_directories(ClearanceChecker PUBLIC ${INCLUDE_DIR_LIB})

# link with library
target_link_libraries(ClearanceChecker PRIVATE ${LINK_LIBS_LIB})
