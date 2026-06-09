cmake_minimum_required(VERSION 3.10)
project(configserver)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找线程库
find_package(Threads REQUIRED)

# 添加可执行文件
add_executable(configserver 
    main.cpp
    src/network/socket.cpp
    src/network/networkBase.cpp
    src/network/tcpUdp.cpp
    src/ftp_server.cpp
)

# 添加包含目录
target_include_directories(configserver PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 链接所有依赖库
target_link_libraries(configserver PRIVATE 
    Threads::Threads
)

# 针对单文件头库httplib.h
if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include/httplib.h")
    target_include_directories(configserver PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
else()
    message(WARNING "httplib.h not found in include directory")
endif()
