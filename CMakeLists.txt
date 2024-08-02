CMAKE_MINIMUM_REQUIRED(VERSION 3.8.2)

set(projectName airsimControl)
project(${projectName})

set(CMAKE_CXX_STANDARD 14)

ADD_SUBDIRECTORY(imgui)

find_package(OpenSceneGraph REQUIRED osg osgViewer osgUtil osgDB osgGA osgText)
find_package(glfw3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
#find_package(PCL REQUIRED)

FILE(GLOB FILES_C "*.cpp")
FILE(GLOB FILES_H "*.h" "*.inl")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(AIRSIM_DIR "/home/channy/Documents/thirdlibs/AirSim" CACHE PATH "/home/channy/Documents/thirdlibs/AirSim")
else()
    set(AIRSIM_DIR "E:/thirdLibs/AirSim" CACHE PATH "E:/thirdLibs/AirSim")
endif()

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
)

if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    link_directories(${projectName}
        ${AIRSIM_DIR}/build_release/output/lib
    )
else()
    link_directories(${projectName}
        ${AIRSIM_DIR}/AirLib/deps/rpclib/lib
        ${AIRSIM_DIR}/MavLinkCom/lib/x64/Release
        ${AIRSIM_DIR}/AirLib/lib/x64/Release
    )
endif()

add_executable(${projectName}
${FILES_C}
${FILES_H}
)

target_link_libraries(${projectName} 
    ${OPENSCENEGRAPH_LIBRARIES}
    ${OpenCV_LIBS}
    #${PCL_LIBRARIES}
    imgui
    AirLib
    MavLinkCom
    rpc
    glfw
)

if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message("system is Linux")
    find_package(Threads REQUIRED)
    target_link_libraries(${projectName} 
        Threads::Threads
        # stdc++fs
        pthread
        dl
    )
else()
    target_link_libraries(${projectName}
        opengl32
    )
endif()
