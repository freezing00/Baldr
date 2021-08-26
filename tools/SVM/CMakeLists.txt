#Cmake最小版本
CMAKE_MINIMUM_REQUIRED(VERSION 3.5)

set(PROJECT_NAME SVM)
project(${PROJECT_NAME})
SET(CMAKE_BUILD_TYPE RELEASE)
#SET(CMAKE_BUILD_TYPE DEBUG)
set(CMAKE_CXX_STANDARD 11)

IF (CMAKE_SYSTEM_NAME MATCHES "Windows")
    set(OpenCV_DIR ${PROJECT_SOURCE_DIR}/../../../Include/opencv/lib)
ENDIF ()

#exe生成路径
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_SOURCE_DIR}/Build)

#找到OpenCV包
FIND_PACKAGE(OpenCV REQUIRED)
set(INC_DIR Include)

#引用库
include_directories(${INC_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/Src)


#生成指定目标
add_executable(${PROJECT_NAME} main.cpp)


#将第三方的库链接到可执行文件
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
