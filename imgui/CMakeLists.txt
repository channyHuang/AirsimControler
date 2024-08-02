FILE(GLOB LIBRARY_FILES_C "*.cpp")
FILE(GLOB LIBRARY_FILES_H "*.h" "*.inl")

find_package(glfw3 REQUIRED)
project(imgui)

include_directories(imgui
	${GLFW3_INCLUDE_DIR}
	)
	
add_library(imgui
${LIBRARY_FILES_C} 
${LIBRARY_FILES_H}
)

if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
TARGET_LINK_LIBRARIES(imgui 
	glfw
)
else()
TARGET_LINK_LIBRARIES(imgui 
	glfw3
)
endif()
