# 三自由度绳驱关节仿真

MATLAB 工程，用于仿真三根绳索与中央直线执行器共同驱动的三自由度关节。工程包含姿态轨迹规划、绳长与绳速计算、卷筒指令转换、静力学张力分配、三维动画、结果绘图和正运动学工作空间分析。

## 模型约定

广义坐标为：

```matlab
q = [alpha; beta; h];
```

- `alpha`：绕固定坐标系 x 轴转动；
- `beta`：绕固定坐标系 y 轴转动；
- `h`：沿固定坐标系 z 轴平移。

旋转矩阵采用固定轴顺序：

```matlab
R = Ry(beta) * Rx(alpha)
```

当前模型令 z 轴转角 `gamma = 0`。主脚本中的姿态角输入使用度，内部运动学计算使用弧度。

| 物理量 | 单位 |
| --- | --- |
| 输入姿态角 | deg |
| 内部角度、角速度 | rad、rad/s |
| 长度、线速度 | mm、mm/s |
| 绳张力、执行器力 | N |
| 绕 x/y 轴力矩 | N·mm |

## 快速运行

在 MATLAB 中将当前文件夹切换到工程根目录，然后运行：

```matlab
main_run
```

主要输入集中在 `main_run.m`：

```matlab
fixedPlatformRadius = 160;   % 定平台半径 / mm
movingPlatformRadius = 80;   % 动平台半径 / mm
drumRadius = 15;             % 卷筒半径 / mm

q0Deg = [0; 0; 39];          % [alpha/deg; beta/deg; h/mm]
qfDeg = [5; -3; 42];
duration = 10;               % 运动时间 / s
timeStep = 0.01;             % 采样周期 / s

enableStaticAnalysis = true;
targetLoad = [0; 0; 50];     % [Mx/(N·mm); My/(N·mm); Fz/N]
preferredForce = [30; 30; 30; 20]; % [T1; T2; T3; Fa] / N
```

运行后将：

1. 在命令行输出终点绳长、卷筒角度和静力学结果；
2. 播放机构三维动画；
3. 绘制运动学结果；
4. 在启用静力学分析时绘制张力、执行器力、平衡残差和可行性。

### 工作空间分析

运行独立的正运动学工作空间分析：

```matlab
workspace_analysis/workspace_run
```

脚本在卷筒角度和中央执行器行程范围内进行 Monte Carlo 采样，通过正运动学求解姿态，并保留同时满足以下条件的点：

- 三个卷筒角度均在设定范围内；
- 正运动学收敛且绳长残差满足容差；
- 三根绳张力均大于设定下限；
- 静力学平衡残差满足容差。

运行结束后，命令行输出扫描点数、收敛点数、可行点数，以及可行姿态的 `alpha`、`beta`、`h` 范围；点云图使用颜色表示最小绳张力。默认采样 `100000` 点，当前环境通常耗时约 2–3 分钟。

主要参数集中在 `workspace_analysis/workspace_run.m`：

```matlab
actuatorAngleLimitDeg = [-135, 135];
linearActuatorRange = [50, 150];
sampleCount = 100000;
randomSeed = 0;

referencePoseDeg = [0; 0; 100];
poseAngleLimitDeg = [-90, 90];
targetLoad = [0; 0; 50];
preferredForce = [30; 30; 30; 20];
```

## 计算流程

```text
起止姿态
  -> 五次多项式轨迹 q(t), q_dot(t)
  -> 旋转矩阵与动平台位置
  -> 绳长 l(t)
  -> 绳长 Jacobian J(q)
  -> 绳速 l_dot(t) = J(q) * q_dot(t)
  -> 卷筒角度 theta(t)

q(t) + 目标载荷 [Mx; My; Fz]
  -> 静力学矩阵 A(q)
  -> 驱动力 [T1; T2; T3; Fa]
  -> 平衡残差与绳索受拉可行性
```

卷筒角度按初始绳长计算：

```matlab
theta = (cableLength - cableLength(:, 1)) / drumRadius;
```

## 文件说明

### 入口与输出

| 文件 | 内容 |
| --- | --- |
| `main_run.m` | 工程入口；定义几何、运动和载荷参数，串联全部计算、动画与绘图 |
| `plot_results.m` | 绘制姿态、绳长、绳速、执行器指令和 Jacobian 条件数 |
| `plot_static_results.m` | 绘制绳张力、中央执行器力、静力学残差和可行性 |
| `animate_cable_joint.m` | 创建三维机构动画；支持实时播放和可选 MP4 导出 |

### 轨迹与运动学

| 文件 | 内容 |
| --- | --- |
| `generate_pose_trajectory.m` | 使用五次时间标度生成起止速度、加速度为零的姿态轨迹 |
| `rpy_rotation.m` | 计算 `Rz(gamma) * Ry(beta) * Rx(alpha)` 旋转矩阵 |
| `calc_cable_length.m` | 根据定平台点、动平台点和位姿计算绳向量、绳长及单位方向 |
| `calc_cable_jacobian.m` | 解析计算 `3x3` 绳长速度 Jacobian |
| `convert_to_actuator_cmd.m` | 将相对绳长变化除以卷筒半径，得到卷筒转角 |

### 静力学

| 文件 | 内容 |
| --- | --- |
| `calc_static_matrix.m` | 构造 `3x4` 静力学矩阵，将 `[T1; T2; T3; Fa]` 映射到 `[Mx; My; Fz]` |
| `solve_tension_distribution.m` | 用伪逆和零空间将解投影到 `PreferredForce` 附近，并检查平衡残差与绳索受拉条件 |
| `tension_analysis.m` | 沿整条姿态轨迹逐点计算静力学矩阵、驱动力、残差和可行性 |

### 工作空间分析

| 文件 | 内容 |
| --- | --- |
| `workspace_analysis/workspace_run.m` | 独立入口；定义采样、执行器、正运动学和静力学参数 |
| `workspace_analysis/analyze_workspace.m` | 采样执行器输入并完成正运动学、执行器范围和张力可行性筛选 |
| `workspace_analysis/solve_forward_kinematics.m` | 使用多初值阻尼 Newton 方法求解单个执行器输入对应的姿态 |
| `workspace_analysis/plot_workspace_cloud.m` | 绘制 `alpha-beta-h` 可行工作空间点云 |

### 测试与资料

| 路径 | 内容 |
| --- | --- |
| `tests/CableJointKinematicsTest.m` | 绳长、Jacobian、轨迹、卷筒转换和运动学绘图测试 |
| `tests/CableJointStaticAnalysisTest.m` | 静力学矩阵、驱动力分配、轨迹分析和绘图测试 |
| `tests/CableJointAnimationTest.m` | 动画几何、位姿更新、播放时序、输入校验和视频导出测试 |
| `workspace_analysis/tests/WorkspaceAnalysisTest.m` | 正运动学、采样复现、可行性筛选和工作空间绘图测试 |
| `reference/` | 理论架构说明、论文 PDF 和原始 CAJ 文献 |
| `docs/superpowers/` | 功能设计记录与实施计划，不参与运行 |

## 静力学说明

静力学矩阵满足：

```matlab
A(q) * [T1; T2; T3; Fa] = [Mx; My; Fz]
```

其中三根绳只能提供拉力，中央执行器沿固定坐标系正 z 方向作用。当前求解器先计算伪逆解，再利用零空间接近 `PreferredForce`。`isFeasible` 仅表示：

- 三根绳张力均大于 `MinCableTension`；
- 平衡残差不超过 `ResidualTolerance`。

该求解器不是带上下界或最优性约束的张力优化器，也不计算 `[Fx; Fy; Mz]` 平衡。

## 动画与视频

`main_run.m` 默认仅播放动画：

```matlab
exportVideo = false;
videoFile = "cable_joint_motion.mp4";
```

将 `exportVideo` 设为 `true` 可导出 MP4。

## 当前边界

- 平台连接点固定为圆周上均匀分布的三个点，定义在 `main_run.m` 的局部函数 `platform_points` 中；
- 轨迹只支持两个转角和一个轴向位移；
- 工作空间点云基于 Monte Carlo 采样，不保证严格命中工作空间边界；
- 正运动学只保留姿态角范围内收敛到的一个连续解分支；
- 未建模绳索弹性、摩擦、质量、惯性和动力学；
- `Jacobian` 条件数仅用于数值状态观察，程序不会自动避开奇异位姿。
