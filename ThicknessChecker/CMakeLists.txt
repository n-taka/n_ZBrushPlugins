cmake_minimum_required(VERSION 3.7)
# project for library
project(ThicknessCheckerTest)
# version
set(serial "1.0")
# release build
set(CMAKE_BUILD_TYPE "Release")

# add subdirectories (custom library)
add_subdirectory(ThicknessChecker)

# Include directories for library
set(INCLUDE_DIR
    ThicknessChecker
)

# Source files for library
set(CPP_FILES
    ThicknessCheckerTest.cpp
)

# External libraries for library
set(LINK_LIBS
    ThicknessChecker
)

# Create shared library
add_executable(ThicknessCheckerTest ${CPP_FILES})

# compile options
target_compile_options(ThicknessCheckerTest PUBLIC -O2 -Wall)
target_compile_features(ThicknessCheckerTest PUBLIC cxx_std_11)

# version
set_target_properties(ThicknessCheckerTest PROPERTIES version ${serial})

# Add a include files
target_include_directories(ThicknessCheckerTest PUBLIC ${INCLUDE_DIR})

# link with library
target_link_libraries(ThicknessCheckerTest PRIVATE ${LINK_LIBS})
