CMAKE_MINIMUM_REQUIRED(VERSION 3.8.2)

set(projectName airsimControl)
project(${projectName})

ADD_SUBDIRECTORY(imgui)

find_package(OpenSceneGraph REQUIRED osg osgViewer osgUtil osgDB osgGA osgText)
find_package(GLFW3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
#find_package(PCL REQUIRED)

FILE(GLOB FILES_C "*.cpp")
FILE(GLOB FILES_H "*.h" "*.inl")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(AIRSIM_DIR "E:/thirdLibs/AirSim" CACHE PATH "E:/thirdLibs/AirSim")

include_directories(${projectName}
	${OSG_INCLUDE_DIR}
	${GLFW3_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    #${PCL_INCLUDE_DIRS}
    ${AIRSIM_DIR}/AirLib/include
    ${AIRSIM_DIR}/AirLib/deps/rpclib/include
    ./
	)
	
link_directories(${projectName}
	${GLFW3_LIBRARY_DIR}
	${OPENSCENEGRAPH_LIBRARIES}
    ${AIRSIM_DIR}/AirLib/deps/rpclib/lib/x64/Release
    ${AIRSIM_DIR}/MavLinkCom/lib/x64/Release
    ${AIRSIM_DIR}/AirLib/lib/x64/Release
	)

add_executable(${projectName}
${FILES_C}
${FILES_H}
)

target_link_libraries(${projectName}
    imgui
	opengl32
	glfw3
	${OPENSCENEGRAPH_LIBRARIES}
    ${OpenCV_LIBS}
    #${PCL_LIBRARIES}
    rpc
    MavLinkCom
    AirLib
)
