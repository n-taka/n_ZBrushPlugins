cmake_minimum_required(VERSION 3.7)
# project for library
project(BooleanTopologicalTest)
# version
set(serial "1.0")
# release build
set(CMAKE_BUILD_TYPE "Release")

# add subdirectories (custom library)
add_subdirectory(BooleanTopological)

# Include directories for library
set(INCLUDE_DIR
    BooleanTopological
)

# Source files for library
set(CPP_FILES
    BooleanTopologicalTest.cpp
)

# External libraries for library
set(LINK_LIBS
    BooleanTopological
)

# Create shared library
add_executable(BooleanTopologicalTest ${CPP_FILES})

# compile options
target_compile_options(BooleanTopologicalTest PUBLIC -O2 -Wall)
target_compile_features(BooleanTopologicalTest PUBLIC cxx_std_11)

# version
set_target_properties(BooleanTopologicalTest PROPERTIES version ${serial})

# Add a include files
target_include_directories(BooleanTopologicalTest PUBLIC ${INCLUDE_DIR})

# link with library
target_link_libraries(BooleanTopologicalTest PRIVATE ${LINK_LIBS})
