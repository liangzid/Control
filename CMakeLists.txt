#这是第一次尝试编写CMakeLists.txt文件.我有点激动.
cmake_minimum_required(VERSION 3.2)

PROJECT(Control)

include_directories(
        include
)

AUX_SOURCE_DIRECTORY(
        src SRC_LIST
)

SET(src
        ${DIR_SRCS}
)

ADD_EXECUTABLE(${SRC_LIST})


