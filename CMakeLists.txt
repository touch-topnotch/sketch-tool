cmake_minimum_required(VERSION 3.10)
project(converter_lib)

set(CMAKE_CXX_STANDARD 17)

# Detect macOS and set up optimizations
if(APPLE)
    set(CMAKE_CXX_STANDARD 20)
    
    # Find Homebrew paths
    if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "arm64")
        # Apple Silicon
        set(HOMEBREW_PREFIX "/opt/homebrew")
    else()
        # Intel Mac
        set(HOMEBREW_PREFIX "/usr/local")
    endif()
    
    # Add Homebrew include and lib paths
    include_directories(${HOMEBREW_PREFIX}/include)
    link_directories(${HOMEBREW_PREFIX}/lib)
    
    # Find OpenMP
    find_package(OpenMP)
    if(OpenMP_CXX_FOUND)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
        message(STATUS "OpenMP found and enabled")
    else()
        message(WARNING "OpenMP not found - parallel processing will be disabled")
    endif()
    
    # Find Embree
    find_library(EMBREE_LIBRARY embree4 PATHS ${HOMEBREW_PREFIX}/lib)
    if(EMBREE_LIBRARY)
        message(STATUS "Embree4 found: ${EMBREE_LIBRARY}")
        set(EMBREE_FOUND TRUE)
    else()
        message(WARNING "Embree4 not found - BVH acceleration will be disabled")
        set(EMBREE_FOUND FALSE)
    endif()
    
    # Find TBB
    find_library(TBB_LIBRARY tbb PATHS ${HOMEBREW_PREFIX}/lib)
    if(TBB_LIBRARY)
        message(STATUS "TBB found: ${TBB_LIBRARY}")
        set(TBB_FOUND TRUE)
    else()
        message(WARNING "TBB not found - some optimizations will be disabled")
        set(TBB_FOUND FALSE)
    endif()
    
    # Set compiler flags for Apple Silicon
    if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "arm64")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mcpu=apple-m2")
        message(STATUS "Apple Silicon detected - using M2 optimizations")
    endif()
endif()

add_library(converter_lib
    src/ConverterFactory.cpp
    src/OBJ2NMapConverter.cpp
    src/OBJ2EdgesConverter.cpp
    src/OBJ2NMapEdgesConverter.cpp
    src/SVG2RoughConverter.cpp
    src/NMap2SurfacesConverter.cpp  
)

# Add optimized version for macOS
if(APPLE)
    target_sources(converter_lib PRIVATE src/OBJ2EdgesConverter_Optimized.mm)
    
    # Link Metal framework
    find_library(METAL_FRAMEWORK Metal)
    find_library(METALKIT_FRAMEWORK MetalKit)
    find_library(FOUNDATION_FRAMEWORK Foundation)
    find_library(QUARTZCORE_FRAMEWORK QuartzCore)
    
    if(METAL_FRAMEWORK)
        target_link_libraries(converter_lib PRIVATE ${METAL_FRAMEWORK})
        message(STATUS "Metal framework found and linked")
    endif()
    
    if(METALKIT_FRAMEWORK)
        target_link_libraries(converter_lib PRIVATE ${METALKIT_FRAMEWORK})
        message(STATUS "MetalKit framework found and linked")
    endif()
    
    if(FOUNDATION_FRAMEWORK)
        target_link_libraries(converter_lib PRIVATE ${FOUNDATION_FRAMEWORK})
        message(STATUS "Foundation framework found and linked")
    endif()
    
    if(QUARTZCORE_FRAMEWORK)
        target_link_libraries(converter_lib PRIVATE ${QUARTZCORE_FRAMEWORK})
        message(STATUS "QuartzCore framework found and linked")
    endif()
    
    if(EMBREE_FOUND)
        target_link_libraries(converter_lib PRIVATE ${EMBREE_LIBRARY})
        target_compile_definitions(converter_lib PRIVATE EMBREE_AVAILABLE)
    endif()
    
    if(TBB_FOUND)
        target_link_libraries(converter_lib PRIVATE ${TBB_LIBRARY})
        target_compile_definitions(converter_lib PRIVATE TBB_AVAILABLE)
    endif()
endif()

# в корне CMakeLists.txt
add_custom_target(svg2roughjs_build
  COMMAND npm install
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/external/svg2roughjs
  COMMENT "Installing svg2roughjs JS/TS dependencies"
)

target_include_directories(converter_lib PUBLIC 
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/include/converter_lib
)
target_include_directories(converter_lib
    PRIVATE
    ${PROJECT_SOURCE_DIR}/external   # now compiler finds headers here
)

add_executable(converter_cli src/main.cpp)
add_dependencies(converter_cli svg2roughjs_build)
target_link_libraries(converter_cli PRIVATE converter_lib)

# Add optimized CLI for macOS
if(APPLE)
    add_executable(converter_cli_optimized src/main.cpp)
    add_dependencies(converter_cli_optimized svg2roughjs_build)
    target_link_libraries(converter_cli_optimized PRIVATE converter_lib)
    target_compile_definitions(converter_cli_optimized PRIVATE USE_OPTIMIZED_VERSION)
endif()

enable_testing()
add_executable(converter_tests tests/converter_tests.cpp)
target_link_libraries(converter_tests PRIVATE converter_lib gtest)
add_test(NAME all_converters COMMAND converter_tests)