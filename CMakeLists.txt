cmake_minimum_required(VERSION 3.21)
project(smart)

set(CMAKE_CXX_STANDARD 11)

enable_testing()

# 遍历所有源文件，增加到变量中
aux_source_directory(. DIR_SRCS)
aux_source_directory(${CMAKE_SOURCE_DIR}/serial/src DIR_SRCS_SERIAL)

# 子文件夹
add_subdirectory(uart/)
add_subdirectory(googletest/)

# 头文件
include_directories(uart/inc)
include_directories(googletest/)
set(GTEST_LIB gtest gtest_main)

# 可执行文件
add_executable(main ${DIR_SRCS})

# 链接库
target_link_libraries(main gtest gtest_main uart)
