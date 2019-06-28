cmake_minimum_required(VERSION 3.7)
# project for library
project(ThicknessChecker)
# version
set(serial "2.1")
# release build
# basically, we use Release
if (NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "Release")
endif()

# external dependencies
find_package(OpenCL REQUIRED)

# Include directories for library
set(INCLUDE_DIR_LIB
    ${OpenCL_INCLUDE_DIRS}
    ../../submodule/eigen
    ../../submodule/libigl/include
)

# Source files for library
set(CPP_FILES_LIB
    ThicknessChecker.cpp
    read_OBJ.cpp
    write_OBJ.cpp
    parseParams.cpp
    openCL_Util.cpp
    openCL_SDF.cpp
)

# External libraries for library
set(LINK_LIBS_LIB
    ${OpenCL_LIBRARIES}
)

# Create shared library
add_library(ThicknessChecker SHARED ${CPP_FILES_LIB})

# compile options
if(WIN32)
set(
    MY_DEBUG_OPTIONS
    /MTd
)
set(
    MY_RELEASE_OPTIONS
    /O2
    /MT
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
target_compile_options(ThicknessChecker PUBLIC "$<$<CONFIG:DEBUG>:${MY_DEBUG_OPTIONS}>")
target_compile_options(ThicknessChecker PUBLIC "$<$<CONFIG:RELEASE>:${MY_RELEASE_OPTIONS}>")
target_compile_features(ThicknessChecker PUBLIC cxx_std_11)
if(WIN32)
    target_compile_options(ThicknessChecker PUBLIC -DNOMINMAX -D_CRT_SECURE_NO_WARNINGS)
endif(WIN32)
if(UNIX)
    target_compile_options(ThicknessChecker PUBLIC -DCL_SILENCE_DEPRECATION)
endif(UNIX)

# version
set_target_properties(ThicknessChecker PROPERTIES version ${serial})

# Add a include files
target_include_directories(ThicknessChecker PUBLIC ${INCLUDE_DIR_LIB})

# link with library
if(WIN32)
    target_link_libraries(ThicknessChecker PRIVATE ${LINK_LIBS})
endif(WIN32)
if(UNIX)
    target_link_libraries(ThicknessChecker PRIVATE ${LINK_LIBS} "-framework OpenCL")
endif(UNIX)
