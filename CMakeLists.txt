cmake_minimum_required(VERSION 3.10)
project(converter_lib)

set(CMAKE_CXX_STANDARD 17)
add_library(converter_lib
    src/ConverterFactory.cpp
    src/OBJ2NMapConverter.cpp
    src/VisibleEdgesConverter.cpp
    src/SVG2RoughConverter.cpp
    src/NMap2SurfacesConverter.cpp
    src/Pipeline.cpp
)
# в корне CMakeLists.txt
add_custom_target(svg2roughjs_build
  COMMAND npm install
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/external/svg2roughjs
  COMMENT "Installing svg2roughjs JS/TS dependencies"
)
target_include_directories(converter_lib PUBLIC include)
target_include_directories(converter_lib
    PRIVATE
    ${PROJECT_SOURCE_DIR}/external   # now compiler finds headers here
)
add_executable(converter_cli src/main.cpp)
add_dependencies(converter_cli svg2roughjs_build)
target_link_libraries(converter_cli PRIVATE converter_lib)

enable_testing()
add_executable(converter_tests tests/converter_tests.cpp)
target_link_libraries(converter_tests PRIVATE converter_lib gtest)
add_test(NAME all_converters COMMAND converter_tests)