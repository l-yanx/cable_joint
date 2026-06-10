# 绳驱三自由度关节正运动学工作空间分析设计

## 1. 目标

在现有绳驱三自由度关节工程中新增独立的正运动学工作空间分析模块。模块以卷筒角度和中央直线执行器行程为输入，通过正运动学求解动平台姿态，并筛选同时满足以下条件的工作空间：

- 三个卷筒角度均位于 `[-135 deg, 135 deg]`；
- 中央直线执行器行程位于 `[50 mm, 150 mm]`；
- 三根绳张力均严格大于 `0 N`；
- 正运动学方程收敛且几何残差不超过给定容差。

最终在姿态空间中生成三维点云图，以颜色表示每个位姿的最小绳张力。

本功能独立放置在 `workspace_analysis/` 文件夹中，不修改现有轨迹仿真入口 `main_run.m` 的运行流程。

## 2. 已确认的模型约定

广义坐标沿用现有工程：

```matlab
q = [alpha; beta; h];
```

其中：

- `alpha`：绕固定坐标系 `x` 轴转角；
- `beta`：绕固定坐标系 `y` 轴转角；
- `h`：中央直线执行器行程，同时也是动平台中心的 `z` 坐标；
- 内部角度使用弧度，输入输出界面中的角度使用度；
- 长度单位为毫米，张力单位为牛顿。

旋转矩阵继续使用：

```matlab
R = rpy_rotation(alpha, beta, 0);
```

即：

```matlab
R = Ry(beta) * Rx(alpha);
```

卷筒零角对应参考姿态：

```matlab
referencePoseDeg = [0; 0; 100];
```

静力学参数沿用当前默认值：

```matlab
targetLoad = [0; 0; 50];          % [Mx; My; Fz]
preferredForce = [30; 30; 30; 20]; % [T1; T2; T3; Fa]
```

## 3. 正运动学方程

参考姿态下三根绳长为：

```matlab
referenceRotation = rpy_rotation(0, 0, 0);
referencePosition = [0; 0; 100];
referenceCableLength = calc_cable_length( ...
    fixedPoints, movingPoints, referencePosition, referenceRotation);
```

卷筒角度和目标绳长的关系沿用现有工程的符号约定：

```matlab
targetCableLength = referenceCableLength + drumRadius * theta;
```

其中 `theta` 使用弧度。

由于中央直线执行器直接确定 `h`，给定 `theta1`、`theta2` 和 `h` 后，正运动学未知量只有：

```matlab
x = [alpha; beta];
```

使用前两根绳建立二维非线性方程：

\[
F(x)=
\begin{bmatrix}
l_1(\alpha,\beta,h)-l_{1,\mathrm{target}}\\
l_2(\alpha,\beta,h)-l_{2,\mathrm{target}}
\end{bmatrix}
=0
\]

求得 `alpha` 和 `beta` 后，由第三根绳的实际长度反算其所需卷筒角度：

```matlab
theta3 = (cableLength(3) - referenceCableLength(3)) / drumRadius;
```

因此工作空间扫描变量为：

```matlab
[theta1, theta2, h]
```

而不是扫描四个执行器的任意组合。第三卷筒角度由机构闭环几何唯一确定，再检查其是否位于允许范围内。这可以避免把几何不相容的四执行器组合送入求解器。

## 4. 数值求解方法

当前 MATLAB 环境未安装 Optimization Toolbox，因此不依赖 `fsolve` 或 `lsqnonlin`。新增求解器使用基础 MATLAB 可实现的带阻尼 Newton 方法。

二维 Newton Jacobian 直接复用现有解析绳长 Jacobian：

```matlab
fullJacobian = calc_cable_jacobian( ...
    fixedPoints, movingPoints, q, rotation);
forwardJacobian = fullJacobian(1:2, 1:2);
```

每次迭代计算：

```matlab
delta = -forwardJacobian \ residual;
```

若完整步长不能降低残差，则依次缩小步长，直到残差下降或达到最小步长。每次更新后将姿态角限制在：

```matlab
poseAngleLimitDeg = [-90, 90];
```

停止条件包括：

- 绳长残差无穷范数不大于 `forwardTolerance`；
- 达到 `maxIterations`；
- Jacobian 非有限、病态或无法产生下降步骤。

由于 Monte Carlo 样本不具有规则的相邻关系，求解器默认使用：

```matlab
initialPoseDeg = [0; 0];
```

为提高远离零位样本的收敛率，每个样本允许使用少量确定性的多初值重试：

```matlab
initialGuessDeg = [ ...
     0,   0;
    30,   0;
   -30,   0;
     0,  30;
     0, -30].';
```

只要任一初值收敛，就接受残差最小的角度范围内解。初值集合是求解器接口，后续可调整；随机样本的生成不参与 Newton 初值选择，从而保证同一输入的正运动学结果稳定。

求解器返回：

```matlab
solution.alpha
solution.beta
solution.cableLength
solution.residual
solution.residualNorm
solution.iterationCount
solution.converged
```

本阶段只保留角度范围内收敛到的一个连续姿态分支，不搜索平台翻转或其他周期性多解。

## 5. 工作空间扫描与可行性判据

默认接口参数集中在独立入口文件顶部：

```matlab
actuatorAngleLimitDeg = [-135, 135];
linearActuatorRange = [50, 150];
sampleCount = 100000;
randomSeed = 0;

referencePoseDeg = [0; 0; 100];
poseAngleLimitDeg = [-90, 90];
forwardTolerance = 1e-6;
maxIterations = 30;

targetLoad = [0; 0; 50];
preferredForce = [30; 30; 30; 20];
minCableTension = 0;
staticResidualTolerance = 1e-8;
```

参数使用普通变量和函数命名参数传递，方便后续修改执行器范围、Monte Carlo 样本数、随机种子、载荷和数值容差。

使用独立随机数流在执行器输入空间内均匀采样：

```matlab
stream = RandStream("mt19937ar", Seed=randomSeed);
theta1Deg = actuatorAngleLimitDeg(1) ...
    + diff(actuatorAngleLimitDeg) * rand(stream, 1, sampleCount);
theta2Deg = actuatorAngleLimitDeg(1) ...
    + diff(actuatorAngleLimitDeg) * rand(stream, 1, sampleCount);
h = linearActuatorRange(1) ...
    + diff(linearActuatorRange) * rand(stream, 1, sampleCount);
```

独立随机数流避免修改 MATLAB 全局随机状态。固定 `randomSeed` 时结果可复现；修改种子可以生成新的覆盖样本。

对每个 Monte Carlo 样本执行：

1. 将 `theta1`、`theta2` 从度转换为弧度；
2. 根据参考绳长计算第一、第二根目标绳长；
3. 给定 `h`，通过正运动学求解 `alpha`、`beta`；
4. 检查正运动学是否收敛且残差满足容差；
5. 由第三根绳长反算 `theta3`；
6. 检查 `theta3` 是否位于 `[-135 deg, 135 deg]`；
7. 调用现有 `calc_static_matrix` 和 `solve_tension_distribution`；
8. 检查静力学平衡残差和 `T1 > 0 && T2 > 0 && T3 > 0`；
9. 保存执行器输入、求解姿态、绳长、张力和可行性状态。

张力判据沿用现有求解器的定义：在 `preferredForce` 附近选择零空间解，再检查该解的三根绳张力。当前阶段不新增张力上下界优化器，也不判断是否存在其他正张力分配。

## 6. 软件结构

新增目录：

```text
workspace_analysis/
├── workspace_run.m
├── analyze_workspace.m
├── solve_forward_kinematics.m
├── plot_workspace_cloud.m
└── tests/
    └── WorkspaceAnalysisTest.m
```

### 6.1 `workspace_run.m`

独立运行入口，负责：

- 将工程根目录加入 MATLAB 路径；
- 定义机构、执行器、求解器、扫描和静力学参数；
- 生成定平台和动平台连接点；
- 调用 `analyze_workspace`；
- 输出扫描点数、正运动学收敛点数和最终可行点数；
- 调用 `plot_workspace_cloud`。

该文件不调用轨迹生成、动画或现有轨迹结果图。

### 6.2 `analyze_workspace.m`

负责 Monte Carlo 执行器采样和全部可行性筛选。接口采用结构化结果，避免向绘图函数传递大量平行数组。

结果至少包含：

```matlab
result.pose                 % 3xN，[alpha; beta; h]
result.actuatorCommand      % 4xN，[theta1; theta2; theta3; h]
result.cableLength          % 3xN
result.force                % 4xN，[T1; T2; T3; Fa]
result.minCableTension      % 1xN
result.forwardResidualNorm  % 1xN
result.staticResidualNorm   % 1xN
result.forwardConverged     % 1xN
result.withinActuatorLimits % 1xN
result.isTensionFeasible    % 1xN
result.isFeasible           % 1xN
result.config               % 本次分析配置
```

结果保留所有扫描点的状态，便于区分正运动学不收敛、第三卷筒超限和张力不可行三类原因。

### 6.3 `solve_forward_kinematics.m`

只负责单个 `[theta1; theta2; h]` 输入下的二维正运动学求解，不负责张力计算和工作空间扫描。

### 6.4 `plot_workspace_cloud.m`

创建独立窗口并绘制：

- `x` 轴：`alpha / deg`；
- `y` 轴：`beta / deg`；
- `z` 轴：`h / mm`；
- 点颜色：`min(T1,T2,T3) / N`；
- 仅将 `isFeasible=true` 的点作为最终工作空间云图。

图中使用固定三维视角、`grid on`、颜色条和明确单位。若没有可行点，函数给出明确警告并创建带说明的空坐标轴，而不是由 `scatter3` 产生难以理解的错误。

## 7. 输入校验和错误处理

新增函数检查：

- 平台连接点均为有限的 `3x3` 数组；
- 卷筒半径为正数；
- 角度和直线执行器上下限严格递增；
- Monte Carlo 样本数为正整数；
- 随机种子为有限的非负整数；
- 参考姿态位于允许的直线执行器范围内；
- Newton 容差为正数，最大迭代次数为正整数；
- 目标载荷为有限的 `3x1` 向量；
- `preferredForce` 为有限的 `4x1` 向量。

预期错误标识使用统一前缀：

```text
CableJointWorkspace:InvalidConfiguration
CableJointWorkspace:InvalidInitialGuess
CableJointWorkspace:DegenerateForwardJacobian
```

单个扫描点不收敛不终止整个分析，而是在结果中标记 `forwardConverged=false`。只有整体配置或输入数据无效时才抛出错误。

## 8. 性能与采样边界

默认使用：

```matlab
sampleCount = 100000;
```

实现采用普通循环和预分配，保证结果顺序稳定。本阶段不使用 Parallel Computing Toolbox、`parfor`、GPU、规则网格补点或自适应重要性采样。

Monte Carlo 点云反映抽样得到的可行工作空间，不保证严格命中工作空间边界，也不能证明未采样区域不可达。增加 `sampleCount` 或使用多个 `randomSeed` 可以提高覆盖密度。若默认运行时间过长，可直接减小 `sampleCount`，不需要修改求解逻辑。

## 9. 测试与验收

新增 `workspace_analysis/tests/WorkspaceAnalysisTest.m`，覆盖：

1. 零卷筒角和 `h=100 mm` 时，正运动学返回 `[0;0;100]`；
2. 从一个已知姿态通过现有逆运动学生成 `theta1`、`theta2` 后，正运动学恢复原姿态；
3. 已收敛解的前两根绳长残差不超过容差；
4. 第三卷筒角由解出的第三根绳长正确反算；
5. 第三卷筒角超出 `135 deg` 时，该点不可行；
6. 任一绳张力不大于 `0 N` 时，该点不可行；
7. 正运动学不收敛时只标记当前点，不中断完整采样；
8. 固定随机种子时，小规模 Monte Carlo 采样结果完全可复现；
9. 不同随机种子生成不同执行器样本；
10. 小规模采样结果的数组尺寸和逻辑标记一致；
11. 云图仅绘制 `isFeasible=true` 的点，颜色数据等于最小绳张力；
12. 现有运动学、静力学和动画测试继续通过。

验收标准：

- 运行 `workspace_analysis/workspace_run.m` 可独立完成分析；
- 执行器角度和直线行程可以在入口文件中直接修改；
- Monte Carlo 样本数和随机种子可以在入口文件中直接修改；
- 输出点全部由正运动学从执行器输入求得；
- 最终点全部满足三个卷筒角范围、直线执行器范围、正运动学残差和三绳正张力条件；
- 云图正确显示 `alpha-beta-h` 工作空间及最小绳张力；
- 不需要 Optimization Toolbox；
- 不改变 `main_run.m` 的现有行为。

## 10. 非目标

本次不包含：

- 随机采样任意四执行器组合；
- 用纯随机搜索代替正运动学方程求根；
- 搜索同一执行器输入对应的所有正运动学多解；
- 超过 `[-90 deg,90 deg]` 的平台翻转姿态；
- 张力上下界优化或最大化张力裕度；
- 绳索弹性、卷筒分层绕线、摩擦和传动间隙；
- 碰撞、自碰撞和机械结构干涉检查；
- Jacobian 奇异性指标作为额外可行性约束；
- 对工作空间边界进行确定性证明；
- 修改现有轨迹、动画和静力学分析入口。
