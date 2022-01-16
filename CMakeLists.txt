cmake_minimum_required(VERSION 3.0.0)
project(cpp_proj)

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(DBSCAN_SOURCE ${PROJECT_SOURCE_DIR}/dbscan/dbscan.cpp  ${PROJECT_SOURCE_DIR}/dbscan/cluster.cpp ${PROJECT_SOURCE_DIR}/dbscan/point.cpp)
set(FLANN ${PROJECT_SOURCE_DIR}/dbscan/nanoflann.hpp)

add_executable(cpp_proj 
                ${PROJECT_SOURCE_DIR}/main.cpp 
                ${DBSCAN_SOURCE}
                ${FLANN}
                ${PROJECT_SOURCE_DIR}/GPF/GPF.cpp 
                )
                
target_link_libraries(cpp_proj 
                    ${PCL_LIBRARIES}
                    )