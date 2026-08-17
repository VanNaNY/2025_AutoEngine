# 2025 AutoEngine — RoboMaster 工程机器人矿石兑换视觉模块

基于 Luxonis DepthAI（OAK 相机 + ToF 深度传感器）实现的视觉感知模块，用于 RoboMaster 工程机器人「兑矿」任务中矿石兑换站角点的识别与三维位姿解算，并通过串口将解算结果发送给下位机。

## 功能流程

1. **相机采集**：DepthAI Pipeline 配置 RGB 彩色相机（CAM_C，800P/30fps）与 ToF 深度相机（CAM_A），通过 `SpatialLocationCalculator` 结合 ToF 深度图获取指定 ROI 的三维空间坐标。
2. **角点识别**：对图像进行轮廓提取，根据检测到的兑换站标志轮廓数量分情况处理：
   - 仅 1 个轮廓：使用 `minEnclosingTriangle` 求最小外接三角形近似角点；
   - 检测到 3 个角点：基于三角形重心法几何推算缺失的第 4 个角点；
   - 检测到 4 个角点：按逆时针排序后，利用向量叉乘计算平面法向量，构建局部正交坐标系（X/Y/Z 方向向量）。
3. **位姿解算**：得到兑换站中心点三维坐标及姿态方向向量。
4. **串口通信**：基于 Linux `termios` API 封装串口收发模块，通过自定义 49 字节数据帧结构体（位置坐标 + 三个方向向量 + 电磁阀开关状态）将解算结果实时发送给下位机。

## 技术栈

- C++17，CMake + Hunter 包管理
- DepthAI SDK（OAK 相机 / ToF 深度）
- OpenCV（轮廓检测、几何计算）
- Linux termios 串口通信

## 目录结构

- `src/Visual_OreRedemption.cpp`：主程序，相机 Pipeline 搭建与主循环
- `src/VisionFeatureExtractor.*`：角点提取、排序、坐标系构建等几何算法
- `src/Serial.*`：串口收发模块
- `utility/`：通用工具函数
- `cmake/HunterGate.cmake`：依赖包管理
