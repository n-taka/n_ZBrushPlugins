cmake_minimum_required(VERSION 3.7)
# project for library
project(ThicknessCheckerTest)
# version
set(serial "2.1")
# release build
if (NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "Release")
endif()

# add subdirectories (custom library)
add_subdirectory(ThicknessChecker)

# external dependencies
find_package(OpenCL REQUIRED)

# Include directories for library
set(INCLUDE_DIR
    ThicknessChecker
    ${OpenCL_INCLUDE_DIRS}
)

# Source files for library
set(CPP_FILES
    ThicknessCheckerTest.cpp
)

# External libraries for library
set(LINK_LIBS
    ThicknessChecker
    ${OpenCL_LIBRARIES}
)

# Create shared library
add_executable(ThicknessCheckerTest ${CPP_FILES})

# compile options
if(WIN32)
set(
    MY_DEBUG_OPTIONS
)
set(
    MY_RELEASE_OPTIONS
    /O2
)
endif(WIN32)
if(UNIX)
set(
    MY_DEBUG_OPTIONS
    -g
)
set(
    MY_RELEASE_OPTIONS
    -O2
)
endif(UNIX)
target_compile_options(ThicknessCheckerTest PUBLIC "$<$<CONFIG:DEBUG>:${MY_DEBUG_OPTIONS}>")
target_compile_options(ThicknessCheckerTest PUBLIC "$<$<CONFIG:RELEASE>:${MY_RELEASE_OPTIONS}>")
target_compile_features(ThicknessCheckerTest PUBLIC cxx_std_11)
if(WIN32)
    target_compile_options(ThicknessCheckerTest PUBLIC -DNOMINMAX)
endif(WIN32)
if(UNIX)
    target_compile_options(ThicknessCheckerTest PUBLIC -DCL_SILENCE_DEPRECATION)
endif(UNIX)

# version
set_target_properties(ThicknessCheckerTest PROPERTIES version ${serial})

# Add a include files
target_include_directories(ThicknessCheckerTest PUBLIC ${INCLUDE_DIR})

# link with library
if(WIN32)
    target_link_libraries(ThicknessCheckerTest PRIVATE ${LINK_LIBS})
endif(WIN32)
if(UNIX)
    target_link_libraries(ThicknessCheckerTest PRIVATE ${LINK_LIBS} "-framework OpenCL")
endif(UNIX)
