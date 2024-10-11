CMAKE_MINIMUM_REQUIRED(VERSION 3.8.2)

project(airsimControl)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenSceneGraph REQUIRED osg osgViewer osgUtil osgDB osgGA osgText)
find_package(glfw3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
#find_package(PCL REQUIRED)

set(OUTER_DIR ${CMAKE_SOURCE_DIR}/../commonClass)
add_subdirectory(${OUTER_DIR}/imgui imgui)
add_subdirectory(${OUTER_DIR}/commonOsg commonOsg)
add_subdirectory(${OUTER_DIR}/nativefiledialog nativefiledialog)

FILE(GLOB FILES_C "*.cpp")
FILE(GLOB FILES_H "*.h" "*.inl")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(AIRSIM_DIR "/home/channy/Documents/thirdlibs/AirSim" CACHE PATH "/home/channy/Documents/thirdlibs/AirSim")
else()
    set(AIRSIM_DIR "E:/thirdLibs/AirSim" CACHE PATH "E:/thirdLibs/AirSim")
endif()

include_directories(${PROJECT_NAME}
	${OSG_INCLUDE_DIR}
	${GLFW3_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    #${PCL_INCLUDE_DIRS}
    ${AIRSIM_DIR}/AirLib/include
    ${AIRSIM_DIR}/AirLib/deps/rpclib/include
    ./
    ${OUTER_DIR}
	)
	
link_directories(${PROJECT_NAME}
	${GLFW3_LIBRARY_DIR}
	${OPENSCENEGRAPH_LIBRARIES}
)

if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    link_directories(${PROJECT_NAME}
        ${AIRSIM_DIR}/build_release/output/lib
    )
else()
    link_directories(${PROJECT_NAME}
        ${AIRSIM_DIR}/AirLib/deps/rpclib/lib
        ${AIRSIM_DIR}/MavLinkCom/lib/x64/Release
        ${AIRSIM_DIR}/AirLib/lib/x64/Release
    )
endif()

add_executable(${PROJECT_NAME}
${FILES_C}
${FILES_H}
)

target_link_libraries(${PROJECT_NAME}
	imgui
    commonOsg
    nativefiledialog
)

target_link_libraries(${PROJECT_NAME} 
    ${OPENSCENEGRAPH_LIBRARIES}
    ${OpenCV_LIBS}
    #${PCL_LIBRARIES}
    AirLib
    MavLinkCom
    rpc
)

if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message("system is Linux")
    find_package(Threads REQUIRED)
    target_link_libraries(${PROJECT_NAME} 
        Threads::Threads
        # stdc++fs
        pthread
        dl
        glfw
    )
else()
    target_link_libraries(${PROJECT_NAME}
        opengl32
        glfw3
    )
endif()
