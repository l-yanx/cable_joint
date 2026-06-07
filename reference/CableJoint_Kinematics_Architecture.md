# 三自由度绳驱关节运动学仿真工程架构说明

## 1. 工程目标

本工程用于完成三自由度绳驱关节的运动学仿真。机构自由度定义为：

\[
q =
\begin{bmatrix}
\alpha \\
\beta \\
h
\end{bmatrix}
\]

其中：

- \(\alpha\)：绕固定坐标系 \(x\) 轴的转角；
- \(\beta\)：绕固定坐标系 \(y\) 轴的转角；
- \(h\)：沿固定坐标系 \(z\) 轴的平移量。

工程的目标是：

1. 输入当前姿态、目标姿态以及目标速度；
2. 生成姿态轨迹 \(q(t)\) 和广义速度轨迹 \(\dot q(t)\)；
3. 通过逆运动学计算每一时刻的绳长 \(l(t)\)；
4. 通过速度雅各比矩阵计算绳长速度 \(\dot l(t)\)；
5. 根据执行器形式，将绳长或绳长变化量转换为电机角度、绕线轮角度或直线执行器行程；
6. 输出可用于执行器控制的轨迹曲线。

---

## 2. 工程输入与输出

### 2.1 输入量

主脚本中建议提供以下输入：

```matlab
q0 = [alpha0; beta0; h0];      % 当前姿态
qf = [alphaf; betaf; hf];      % 目标姿态
T  = 2.0;                      % 运动总时间
dt = 0.01;                     % 仿真步长
v = [v_alpha,v_beta,v_h]       % 期望速度
```

---

### 2.2 输出量

输出以下结果：

#### 1. 姿态轨迹(三维坐标系)

\[
q(t)=
\begin{bmatrix}
\alpha(t) \\
\beta(t) \\
h(t)
\end{bmatrix}
\]

#### 2. 速度轨迹（三维坐标系）

\[
\dot q(t)=
\begin{bmatrix}
\dot\alpha(t) \\
\dot\beta(t) \\
\dot h(t)
\end{bmatrix}
\]

#### 3. 绳长轨迹（二维坐标系+三根曲线 ）

\[
l(t)=
\begin{bmatrix}
l_1(t) \\
l_2(t) \\
l_3(t)
\end{bmatrix}
\]

#### 4. 三根绳速轨迹（二维坐标系+三根曲线 ）

\[
\dot l(t)=
\begin{bmatrix}
\dot l_1(t) \\
\dot l_2(t) \\
\dot l_3(t)
\end{bmatrix}
\]

#### 5. 执行器输入轨迹(有正负) （二维坐标系+三根曲线 ）

如果使用绕线轮执行器：

\[
\theta_i(t)=\frac{l_i(t)-l_i(0)}{r_{\text{spool}}}
\]

其中：

- \(\theta_i(t)\)：第 \(i\) 个绕线轮角度；
- \(r_{\text{spool}}\)：绕线轮半径；
- \(l_i(t)-l_i(0)\)：相对于初始状态的绳长变化量。

如果使用直线执行器：

\[
s_i(t)= v_h
\]

---

## 3. 推荐工程文件结构

建议工程目录如下：

```text
CableJoint_Kinematics/
│
├── main_run.m
│
├── rpy_rotation.m
├── calc_cable_length.m
├── calc_cable_jacobian.m
│
├── generate_pose_trajectory.m
├── convert_to_actuator_cmd.m
├── plot_results.m
│
└── README_architecture.md（写中文）
```

---

## 4. 各脚本功能说明

### 4.1 `main_run.m`

总执行脚本，负责提供输入接口并调用其他模块。

主要功能：

1. 定义机构几何参数；
2. 输入当前姿态 \(q_0\)、目标姿态 \(q_f\)、运动时间 \(T\)；
3. 调用轨迹生成函数，得到 \(q(t)\) 和 \(\dot q(t)\)；
4. 对每一个时间步调用逆运动学和速度雅各比；
5. 输出绳长、绳速、执行器角度或行程；
6. 绘制结果曲线。

主流程：

```matlab
% 1. 定义机构参数 P, Q

% 2. 输入初始姿态和目标姿态
q0 = [alpha0; beta0; h0];
qf = [alphaf; betaf; hf];

% 3. 生成姿态轨迹
[q_traj, qdot_traj, t] = generate_pose_trajectory(q0, qf, T, dt);

% 4. 遍历每个时间点
for k = 1:length(t)

    q    = q_traj(:, k);
    qdot = qdot_traj(:, k);

    R = rpy_rotation(q(1), q(2), 0);

    [l(:, k), L_vec(:, :, k), u(:, :, k)] = calc_cable_length(P, Q, q, R);

    J(:, :, k) = calc_cable_jacobian(P, Q, q, R);

    l_dot(:, k) = J(:, :, k) * qdot;

end

% 5. 执行器转换
actuator_cmd = convert_to_actuator_cmd(l);

% 6. 绘图
plot_results(t, q_traj, l, l_dot, actuator_cmd);
```

---

### 4.2 `rpy_rotation.m`

旋转矩阵计算函数。

输入：

```matlab
alpha
beta
gamma
```

输出：

```matlab
R
```

对于当前机构，可以只使用 \(\alpha,\beta\)，令 \(\gamma=0\)。

示例：

```matlab
function R = rpy_rotation(alpha, beta, gamma)

    Rx = [1, 0, 0;
          0, cos(alpha), -sin(alpha);
          0, sin(alpha),  cos(alpha)];

    Ry = [ cos(beta), 0, sin(beta);
           0,         1, 0;
          -sin(beta), 0, cos(beta)];

    Rz = [cos(gamma), -sin(gamma), 0;
          sin(gamma),  cos(gamma), 0;
          0,           0,          1];

    R = Rz * Ry * Rx;

end
```

注意：旋转顺序必须和理论推导保持一致。

---

### 4.3 `calc_cable_length.m`

逆运动学函数。

输入：

```matlab
P
Q
q
R
```

输出：

```matlab
l
L_vec
u
```

功能：

根据当前姿态计算三根绳长。

数学关系：

\[
\mathbf L_i =
\mathbf d + R\mathbf Q_i - \mathbf P_i
\]

\[
l_i=\|\mathbf L_i\|
\]

\[
\mathbf u_i=\frac{\mathbf L_i}{l_i}
\]

这部分是位置层逆运动学：

\[
q \rightarrow l
\]

---

### 4.4 `calc_cable_jacobian.m`

速度雅各比计算函数。

输入：

```matlab
P
Q
q
R
```

输出：

```matlab
J
```

功能：

根据当前姿态计算速度雅各比矩阵 \(J(q)\)。

推荐使用偏导形式：

\[
J_i =
\begin{bmatrix}
\mathbf u_i^T \frac{\partial R}{\partial \alpha}\mathbf Q_i &
\mathbf u_i^T \frac{\partial R}{\partial \beta}\mathbf Q_i &
\mathbf u_i^T \mathbf e_z
\end{bmatrix}
\]

这部分是速度层面的逆运动学：

\[
\dot q \rightarrow \dot l
\]

也就是：

\[
\dot l = J(q)\dot q
\]

---

### 4.5 `generate_pose_trajectory.m`

姿态轨迹生成函数。

输入：

```matlab
q0
qf
T
dt
```

输出：

```matlab
q_traj
qdot_traj
t
```

最简单的版本可以使用线性插值：

\[
q(t)=q_0+(q_f-q_0)\frac{t}{T}
\]

\[
\dot q(t)=\frac{q_f-q_0}{T}
\]

后续可以改成五次多项式轨迹，使速度和加速度更加平滑。

---

### 4.6 `convert_to_actuator_cmd.m`

执行器转换函数。

输入：

```matlab
l
```

输出：

```matlab
actuator_cmd
```

如果是绕线轮：

\[
\theta_i(t)=\frac{l_i(t)-l_i(0)}{r_{\text{spool}}}
\]

如果是直线执行器：

\[
s_i(t)=l_i(t)-l_i(0)
\]

这个函数用于把理论绳长轨迹转换为实际执行器可以执行的命令。

---

### 4.7 `plot_results.m`

绘图函数。

建议绘制：

1. \(\alpha(t),\beta(t),h(t)\)；
2. \(l_1(t),l_2(t),l_3(t)\)；
3. \(\dot l_1(t),\dot l_2(t),\dot l_3(t)\)；
4. 执行器角度或行程；
5. Jacobian 条件数变化曲线。

---

## 5. 推荐主流程

整个工程的计算链条为：

```text
输入当前姿态 q0 和目标姿态 qf
        ↓
生成姿态轨迹 q(t), q_dot(t)
        ↓
计算每个时刻的旋转矩阵 R(t)
        ↓
逆运动学：q(t) → l(t)
        ↓
速度雅各比：q(t) → J(q)
        ↓
速度映射：l_dot(t) = J(q) q_dot(t)
        ↓
执行器换算：l(t) → 电机角度/行程
        ↓
输出曲线与仿真结果
```
---


