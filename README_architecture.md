# 三自由度绳驱关节运动学仿真

## 1. 工程目标

本工程计算三自由度绳驱关节的姿态轨迹、绳长、绳速和卷筒角度。广义坐标为：

```matlab
q = [alpha; beta; h];
```

- `alpha`：绕固定坐标系 x 轴的转角；
- `beta`：绕固定坐标系 y 轴的转角；
- `h`：沿固定坐标系 z 轴的平移量。

## 2. 单位和旋转约定

- `main_run.m` 中的姿态角输入使用度（deg）；
- `generate_pose_trajectory.m` 将角度转换为弧度（rad）；
- 其余计算函数的角度和角速度均使用 rad、rad/s；
- 长度和线速度使用 mm、mm/s；
- 旋转矩阵采用固定轴顺序 `R = Ry(beta) * Rx(alpha)`，即 `gamma = 0`。

## 3. 文件结构

```text
main_run.m                    总执行脚本和参数输入接口
rpy_rotation.m                旋转矩阵
calc_cable_length.m           逆运动学
calc_cable_jacobian.m         速度 Jacobian
generate_pose_trajectory.m    五次多项式姿态轨迹
convert_to_actuator_cmd.m     绳长到卷筒角度的转换
plot_results.m                结果绘图
tests/                        MATLAB 单元测试
reference/                    参考文献和原始架构说明
```

## 4. 数据流

```text
q0Deg, qfDeg
    -> generate_pose_trajectory
    -> q(t), q_dot(t) [rad, mm]
    -> calc_cable_length
    -> l(t)
    -> calc_cable_jacobian
    -> l_dot(t) = J(q) * q_dot(t)
    -> convert_to_actuator_cmd
    -> theta(t)
```

轨迹规划采用五次时间标度：

```matlab
tau = t / duration;
s = 10 * tau.^3 - 15 * tau.^4 + 6 * tau.^5;
q = q0 + (qf - q0) * s;
```

该轨迹在起点和终点的速度、加速度均为零。

逆运动学使用：

```matlab
L_i = d + R * P_i - Q_i;
l_i = norm(L_i);
u_i = L_i / l_i;
```

速度 Jacobian 使用 `R = Ry * Rx` 的解析偏导：

```matlab
J_i = [u_i' * dR_dalpha * P_i, ...
       u_i' * dR_dbeta  * P_i, ...
       u_i' * [0; 0; 1]];
```

## 5. 输入接口

在 `main_run.m` 中修改以下参数：

```matlab
fixedPlatformRadius = 160;  % mm
movingPlatformRadius = 80;  % mm
drumRadius = 15;            % mm，可按实际卷筒修改

q0Deg = [0; 0; 39];         % alpha/deg, beta/deg, h/mm
qfDeg = [5; 3; 120];
duration = 2;               % s
timeStep = 0.01;            % s
```

卷筒命令定义为：

```matlab
theta = (cableLength - cableLength(:, 1)) / drumRadius;
```

## 6. 运行和测试

在 MATLAB 中将当前文件夹切换到工程根目录，然后运行：

```matlab
main_run
```

运行测试：

```matlab
results = runtests("tests");
assertSuccess(results);
```
