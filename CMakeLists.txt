# cmake_minimum_required(VERSION 3.16)
# project(decimator)

# list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# # Libigl
# option(LIBIGL_WITH_GLFW "Build target igl::glfw" ON)
# include(libigl)

# # Add your project files
# file(GLOB SRC_FILES *.cpp)
# add_executable(${PROJECT_NAME} ${SRC_FILES})
# target_link_libraries(${PROJECT_NAME} PUBLIC igl::glfw)

cmake_minimum_required(VERSION 3.16)
project(decimator)

SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)


# Libigl
option(LIBIGL_WITH_GLFW "Build target igl::glfw" ON)
include(libigl)

## We need Eigen
# find_package(EIGEN REQUIRED)
# include_directories( "${EIGEN_INCLUDE_DIR}" )

## We need libigl
# find_package(LIBIGL REQUIRED)
# include_directories( "${LIBIGL_INCLUDE_DIR}" )

## We don't have/want MOSEK
# add_definitions(-DIGL_NO_MOSEK)

link_directories(
	../lib
	# ${EIGEN_DIRS}
) 

## We need C++11. Put this directive after CGAL's include.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g " )

add_library(DEC_LIBS
    OBJECT
    objParser.cpp
    structurePreserve.cpp
    )
    
add_executable(decimater
	main.cpp
	$<TARGET_OBJECTS:DEC_LIBS>
	)
target_link_libraries ( decimater
	${LIBIGL_LIBRARIES}
    PUBLIC igl::glfw
)
