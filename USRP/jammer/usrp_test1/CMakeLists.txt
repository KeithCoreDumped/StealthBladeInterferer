cmake_minimum_required(VERSION 3.10)
project(MyUSRPProject VERSION 1.0)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_PREFIX_PATH "/usr/lib/x86_64-linux-gnu/cmake/gnuradio;/usr/lib/cmake/uhd")

# 设置 CMake 前缀路径

include_directories("/usr/include/gnuradio")
include_directories("/usr/include/gnuradio/filter")

link_directories("/usr/lib/x86_64-linux-gnu")

include_directories("xlnt-master/include")

# 查找 UHD 库
set(UHD_DIR "/usr/lib/cmake/uhd")
find_package(UHD REQUIRED)
if (UHD_FOUND)
    message(STATUS "UHD found: ${UHD_LIBRARIES}")
else()
    message(FATAL_ERROR "UHD not found!")
endif()

# 查找 GNU 库
# 设置 Gnuradio_DIR 指向 GNU Radio CMake 配置文件所在的目录
set(Gnuradio_DIR "/usr/lib/x86_64-linux-gnu/cmake/gnuradio")

find_package(Gnuradio REQUIRED COMPONENTS runtime analog uhd blocks filter fft digital)
if (Gnuradio_FOUND)
    message(STATUS "GNU Radio found: ${GNURADIO_RUNTIME_LIBRARIES}")
else()
    message(FATAL_ERROR "GNU Radio not found!")
endif()


# 查找 Python3 开发环境
find_package(Python3 COMPONENTS Development NumPy REQUIRED)

# 添加可执行文件
add_executable(my_usrp_app main.cpp scan_enemy.cpp send_noise.cpp)

# 链接必要的库
target_link_libraries(my_usrp_app
    ${UHD_LIBRARIES}
    /usr/lib/x86_64-linux-gnu/libfftw3f.so   # fftw3f的库
    /usr/lib/x86_64-linux-gnu/libgnuradio-runtime.so
    /usr/lib/x86_64-linux-gnu/libgnuradio-analog.so
    /usr/lib/x86_64-linux-gnu/libgnuradio-uhd.so
    /usr/lib/x86_64-linux-gnu/libgnuradio-blocks.so
    /usr/lib/x86_64-linux-gnu/libgnuradio-filter.so
    /usr/lib/x86_64-linux-gnu/libgnuradio-fft.so
    /usr/lib/x86_64-linux-gnu/libgnuradio-digital.so
    Python3::Python 
    Python3::NumPy 
    pthread 
    fftw3
    log4cpp
)
