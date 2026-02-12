cmake_minimum_required(VERSION 3.15)
project(ink3d CXX)
find_package(CGAL REQUIRED)
add_executable(${PROJECT_NAME} 
               src/main.cpp
               src/projection.cpp
               src/visibility.cpp
             )
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(${PROJECT_NAME} CGAL::CGAL)
