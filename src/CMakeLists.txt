# 声明要求的 cmake 最低版本
cmake_minimum_required( VERSION 3.8 )

# 声明一个 cmake 工程
project( Main )

# 设置编译模式
set( CMAKE_BUILD_TYPE "Debug" )
set( CMAKE_CXX_FLAGS "-std=c++17" )

# 添加 OpenCV 库
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# 添加plc库
find_package( PCL REQUIRED COMPONENT common io )
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINTIONS} )

include_directories( "/usr/include/eigen3" )

# 添加一个可执行程序
# 语法：add_executable( 程序名 源代码文件 ）
add_executable( calib calib.cpp )
add_executable( rectify rectify.cpp )
add_executable( calib_ext calib_ext.cpp )
add_executable( undistort undistort.cpp )

# 链接到 OpenCV 库
target_link_libraries( calib ${OpenCV_LIBS} )
target_link_libraries( calib_ext ${OpenCV_LIBS} )

target_link_libraries( undistort ${OpenCV_LIBS} )

target_link_libraries( rectify ${OpenCV_LIBS} )

