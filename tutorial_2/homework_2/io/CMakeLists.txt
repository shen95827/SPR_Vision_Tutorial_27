# 把my_camera.cpp编译成io静态库
add_library(io STATIC
    my_camera.cpp
)

# 把头文件路径暴露给上层项目
target_include_directories(io PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_directories(io PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/hikrobot/lib/amd64)

# 链接所有需要的依赖
target_link_libraries(io
    # 链接上层的tools工具库
    tools
    # 链接OpenCV
    ${OpenCV_LIBS}
    # 链接yaml-cpp配置库
    yaml-cpp
    # 链接海康SDK核心库
    MvCameraControl
    # 链接USB依赖库
    usb-1.0
)