cmake_minimum_required(VERSION 3.16)
project(ink3d CXX)
find_package(CGAL REQUIRED)
find_package(jsoncpp REQUIRED)

add_executable(${PROJECT_NAME} 
               src/projection.cpp
               src/visibility.cpp
               src/config.cpp
               src/main.cpp
             )
target_precompile_headers(${PROJECT_NAME} PUBLIC "include/pch.h")
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(${PROJECT_NAME} CGAL::CGAL JsonCpp::JsonCpp)
