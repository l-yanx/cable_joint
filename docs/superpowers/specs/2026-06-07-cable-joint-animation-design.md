# 绳驱三自由度关节动画设计

## 1. 目标

在现有运动学仿真基础上新增 MATLAB 三维动画，根据每个采样时刻的动平台位姿更新机构模型，实时展示：

- 定平台圆盘；
- 动平台圆盘；
- 中心直线执行器；
- 三根绳索；
- 定平台和动平台上的绳连接节点。

现有运动学计算及 `plot_results.m` 中的五张结果图保持不变。

## 2. 已确认参数

| 参数 | 数值或行为 |
|---|---|
| 定平台半径 | 使用 `main_run.m` 中的 `fixedPlatformRadius` |
| 动平台半径 | 使用 `main_run.m` 中的 `movingPlatformRadius` |
| 两个平台厚度 | `1 mm` |
| 中心执行器模型 | 单一圆柱 |
| 中心执行器半径 | `5 mm` |
| 播放窗口 | 独立三维动画窗口 |
| 相机 | 固定三维视角 |
| 播放速度 | 按仿真时间 `1:1` |
| 视频导出 | 默认关闭，可选导出 MP4 |

## 3. 坐标和运动约定

定平台圆心位于固定坐标系原点：

```matlab
fixedCenter = [0; 0; 0];
```

动平台圆心为：

```matlab
movingCenter = [0; 0; h];
```

动平台旋转矩阵沿用当前工程约定：

```matlab
R = rpy_rotation(alpha, beta, 0);
```

即：

```matlab
R = Ry(beta) * Rx(alpha);
```

动平台局部坐标中的任意点 `pLocal` 转换到固定坐标系：

```matlab
pWorld = movingCenter + R * pLocal;
```

中心圆柱始终沿固定坐标系 `z` 轴，从定平台圆心延伸到动平台圆心，不随动平台倾转。

## 4. 软件结构

新增独立函数：

```matlab
animate_cable_joint(time, qTrajectory, fixedPoints, movingPoints, ...
    fixedPlatformRadius, movingPlatformRadius, options)
```

`options` 至少包含：

```matlab
options.PlatformThickness
options.ActuatorRadius
options.ExportVideo
options.VideoFile
```

动画函数只读取已有轨迹和几何参数，不重新计算轨迹、绳长或 Jacobian。

`main_run.m` 新增配置：

```matlab
platformThickness = 1;
actuatorRadius = 5;
exportVideo = false;
videoFile = "cable_joint_motion.mp4";
```

运动学计算完成后调用动画函数。现有 `plot_results(...)` 调用保留。

## 5. 图形对象

### 5.1 定平台

- 使用有厚度的圆盘表面模型；
- 圆心固定在原点；
- 半径为 `fixedPlatformRadius`；
- 厚度为 `1 mm`；
- 初始化后不再更新。

### 5.2 动平台

- 在局部坐标系中建立有厚度的圆盘网格；
- 半径为 `movingPlatformRadius`；
- 厚度为 `1 mm`；
- 每帧将局部网格通过 `movingCenter + R * pLocal` 转换到固定坐标系；
- 通过更新已有图形句柄的坐标数据完成动画。

### 5.3 中心执行器

- 使用单一圆柱模型；
- 半径为 `5 mm`；
- 轴线固定为 `z` 轴；
- 下端位于定平台圆心附近；
- 上端位于动平台圆心附近；
- 每帧根据 `h` 更新圆柱高度。

平台厚度仅用于显示。圆柱的轴向端点以两个平台圆心为基准，使其视觉上连接两个圆盘；允许圆柱与圆盘厚度区域轻微重叠，避免出现可见间隙。

### 5.4 绳索和节点

第 `i` 根绳连接：

```matlab
fixedPoints(:, i)
```

和：

```matlab
movingCenter + R * movingPoints(:, i)
```

三根绳使用相同颜色。六个连接节点使用统一且醒目的点标记或小球显示。

## 6. 动画更新流程

初始化阶段：

1. 创建独立动画窗口和三维坐标轴；
2. 创建定平台图形；
3. 创建动平台图形；
4. 创建中心圆柱；
5. 创建三根绳索；
6. 创建六个节点；
7. 创建状态文本；
8. 设置等比例坐标、固定视角和固定坐标范围。

每帧更新：

1. 从 `qTrajectory(:, k)` 读取 `alpha`、`beta`、`h`；
2. 计算当前旋转矩阵和动平台圆心；
3. 更新动平台全部顶点；
4. 更新动平台三个绳节点；
5. 更新三根绳索端点；
6. 更新中心圆柱高度；
7. 更新当前时间和位姿文本；
8. 调用 `drawnow`；
9. 按 `time(k)` 与实际经过时间的差值暂停，实现 `1:1` 播放。

若用户关闭动画窗口，循环应正常停止，不继续访问已失效的图形句柄。

## 7. 坐标范围与视图

坐标范围在播放前根据全部轨迹确定，并在播放过程中保持固定，防止自动缩放造成画面跳动。

- `x/y` 范围至少覆盖定平台半径、动平台姿态后的外接范围和适量边距；
- `z` 范围覆盖平台厚度及轨迹中的最小和最大 `h`；
- 使用 `axis equal` 保证几何比例正确；
- 使用固定三维观察角度；
- 显示网格和 `x/y/z` 轴标签，长度单位为毫米。

## 8. 实时播放与视频导出

默认：

```matlab
exportVideo = false;
```

此时动画按仿真时间实时播放，不创建视频文件。

启用导出后：

```matlab
exportVideo = true;
```

优先使用 MATLAB `VideoWriter` 的 `"MPEG-4"` profile 直接创建 MP4。若当前平台不提供该 profile，则先使用 `"Motion JPEG AVI"` 写入临时 AVI，再调用系统 `ffmpeg` 和 `libx264` 转码为 `yuv420p` MP4；临时文件在成功、失败或异常退出时均应清理。若 `ffmpeg` 不可用或转码失败，使用明确的动画模块错误标识报告原因。

视频时间尺度与仿真时间一致。由于当前默认步长 `0.01 s` 对应 `100 fps`，实现时应采用兼容的固定输出帧率并按时间采样轨迹帧，而不是假定所有编码器都支持 `100 fps`。默认输出 `30 fps`，目标帧数为 `max(2, round(duration * fps))`，目标时刻通过 `linspace` 包含轨迹首尾，再选择最近的仿真帧。重复索引必须保留，以维持固定帧率下的视频时长。

实时节拍仅用于交互播放。导出模式下无需通过额外暂停控制视频编码速度，但生成的视频时长应与仿真 `duration` 一致。

## 9. 错误处理

动画函数应检查：

- `time` 为非空、单调递增向量；
- `qTrajectory` 为 `3 x N`，并与 `time` 长度一致；
- 平台半径、平台厚度和执行器半径为正数；
- `fixedPoints` 和 `movingPoints` 均为 `3 x 3`；
- 导出开启时，视频文件名非空。
- MATLAB 无原生 MPEG-4 profile 且系统找不到 `ffmpeg` 时，错误标识为 `CableJointAnimation:Mp4EncoderUnavailable`；
- `ffmpeg` 转码失败时，错误标识为 `CableJointAnimation:Mp4EncodingFailed`。

无效输入使用明确的 MATLAB 错误标识和错误信息。

## 10. 测试与验收

新增测试覆盖：

1. 初始帧与末帧的动平台节点满足  
   `movingCenter + R * movingPoints`；
2. 每根绳的两端与对应的固定、动平台节点重合；
3. 中心圆柱半径为 `5 mm`，高度与当前 `h` 一致；
4. 两个平台的显示厚度均为 `1 mm`；
5. 动画窗口使用固定坐标范围和等比例坐标；
6. `ExportVideo=false` 时不创建视频文件；
7. 视频帧采样逻辑覆盖起点和终点，并保持目标时长；
8. 用户关闭窗口后动画能正常退出；
9. 现有 `CableJointKinematicsTest` 全部继续通过。

验收标准：

- 运行 `main_run.m` 后出现独立三维动画窗口；
- 定平台不动，动平台按轨迹平移和倾转；
- 三根绳始终连接正确的节点；
- 中心圆柱随 `h` 伸缩并保持沿固定 `z` 轴；
- 播放过程中视角和坐标范围不跳动；
- 默认不导出文件，打开选项后可生成 MP4；
- 原有结果图和数值结果不变。

## 11. 非目标

本次不包含：

- 绳索弯曲、弹性、垂度或碰撞；
- 平台与绳索的实体碰撞检测；
- 中心执行器缸筒和活塞杆的分体建模；
- 交互式相机环绕；
- 修改现有运动学模型或轨迹规划算法；
- 将动画嵌入现有五图结果窗口。
