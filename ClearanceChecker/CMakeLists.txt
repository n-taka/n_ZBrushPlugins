cmake_minimum_required(VERSION 3.7)
# project for library
project(ClearanceCheckerTest)
# version
set(serial "1.0")
# release build
set(CMAKE_BUILD_TYPE "Release")

# add subdirectories (custom library)
add_subdirectory(ClearanceChecker)

# Include directories for library
set(INCLUDE_DIR
    ClearanceChecker
)

# Source files for library
set(CPP_FILES
    ClearanceCheckerTest.cpp
)

# External libraries for library
set(LINK_LIBS
    ClearanceChecker
)

# Create shared library
add_executable(ClearanceCheckerTest ${CPP_FILES})

# compile options
target_compile_options(ClearanceCheckerTest PUBLIC -O2 -Wall)
target_compile_features(ClearanceCheckerTest PUBLIC cxx_std_11)

# version
set_target_properties(ClearanceCheckerTest PROPERTIES version ${serial})

# Add a include files
target_include_directories(ClearanceCheckerTest PUBLIC ${INCLUDE_DIR})

# link with library
target_link_libraries(ClearanceCheckerTest PRIVATE ${LINK_LIBS})
