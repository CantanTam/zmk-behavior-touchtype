# 这里的 app 是 ZMK 的主目标
target_include_directories(app PRIVATE include)

# 指向你实际的源文件路径（相对于 CMakeLists.txt 的路径）
target_sources(app PRIVATE src/behaviors/behavior_led.c)