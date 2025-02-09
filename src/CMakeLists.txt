# src/CMakeLists.txt（正确示例）
set(SOURCE_FILES
    Visual_OreRedemption.cpp
    Serial.cpp
)

add_executable(Visual_OreRedemption ${SOURCE_FILES})
target_link_libraries(Visual_OreRedemption PRIVATE utility depthai::opencv usb-1.0 Threads::Threads)
