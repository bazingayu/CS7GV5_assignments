cmake_minimum_required(VERSION 3.15)
project(assignment1)

set(CMAKE_CXX_STANDARD 14)

set(PROJECT_DIR "D:/Projects/realtime_animation/assignment1")
include_directories(${PROJECT_DIR})
include_directories(${PROJECT_DIR}/fonts)
include_directories(${PROJECT_DIR}/source)
include_directories(${PROJECT_DIR}/glm)
include_directories(${PROJECT_DIR}/freetype/include)
include_directories(${PROJECT_DIR}/freeglut/include)
include_directories(${PROJECT_DIR}/glew-1.10.0/include)
include_directories(${PROJECT_DIR}/assimp/include)

LINK_DIRECTORIES("${PROJECT_DIR}/freeglut/lib")
LINK_DIRECTORIES("${PROJECT_DIR}/assimp/lib/Debug")
LINK_DIRECTORIES("${PROJECT_DIR}/freetype/lib/win32")
LINK_DIRECTORIES("${PROJECT_DIR}/glew-1.10.0/lib/Release/Win32")

add_executable(assignment1
        source/main.cpp
        source/maths_funcs.cpp)

target_link_libraries(${PROJECT_NAME} "freeglut.lib")
target_link_libraries(${PROJECT_NAME} "glew32.lib")
target_link_libraries(${PROJECT_NAME} "assimp.lib")
target_link_libraries(${PROJECT_NAME} "freetype.lib")

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/freeglut/bin/freeglut.dll
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/glew-1.10.0/bin/Release/Win32/glew32.dll
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/assimp/bin/Assimp32d.dll
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/freetype/lib/win32/freetype.dll
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})










