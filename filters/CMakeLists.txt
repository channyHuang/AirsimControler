cmake_minimum_required(VERSION 3.15)

project(imuFilters)

# find_package(OpenSceneGraph REQUIRED osg osgViewer osgUtil osgDB osgGA osgText)

find_package(Eigen3 REQUIRED)

list(APPEND CMAKE_MODULE_PATH /usr/share/cmake/geographiclib) # for ubuntu 18.04
find_package (GeographicLib REQUIRED)

find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
	include_directories(${OpenCV_INCLUDE_DIRS})
	# link_libraries(${OpenCV_LIBS})
endif()

find_package(Ceres REQUIRED)
if(Ceres_FOUND)
  include_directories(${CERES_INCLUDE_DIRS})
endif()

find_package(g2o REQUIRED)
if(g2o_FOUND)
    include_directories(${g2o_INCLUDE_DIRS})
    # link_directories(/usr/local/lib/)
    message("  =========== g2o libs: " ${g2o_INCLUDE_DIRS})
endif()
set(g2o_LIBS g2o_core g2o_stuff g2o_types_sba g2o_types_slam3d g2o_solver_csparse g2o_csparse_extension)
 
find_package(Boost REQUIRED COMPONENTS filesystem iostreams program_options system serialization)

include_directories(${PROJECT_NAME} 
include
${EIGEN3_INCLUDE_DIR}
${Boost_INCLUDE_DIRS}
./
./include/
)

file(GLOB HEADERS *.h include/*/*.hpp)
file(GLOB CPPS *.cpp)

set(PROJECT_SOURCES
    ${HEADERS}
    ${CPPS}
)

add_library(${PROJECT_NAME} 
               ${PROJECT_SOURCES}
)

# link_directories(${PROJECT_NAME}
# 	${GLFW3_LIBRARY_DIR}
# )

# target_link_libraries(${PROJECT_NAME} 
# ${OpenCV_LIBS} ${CERES_LIBRARIES} ${g2o_LIBS}

# ${Boost_LIBRARY_DIRS}
# )

