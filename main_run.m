%% 三自由度绳驱关节运动学仿真
% 外部输入角度。
% 长度单位统一使用毫米。

clear;
clc;
close all;

%% 机构参数
fixedPlatformRadius = 160;
movingPlatformRadius = 80;
drumRadius = 15;
platformThickness = 1;
actuatorRadius = 5;
exportVideo = false;
videoFile = "cable_joint_motion.mp4";

fixedPoints = platform_points(fixedPlatformRadius);
movingPoints = platform_points(movingPlatformRadius);

%% 运动参数输入
q0Deg = [0; 0; 39];
qfDeg = [1; -2; 40];
duration = 2;
timeStep = 0.01;

%% 轨迹与运动学计算
[qTrajectory, qdotTrajectory, time] = generate_pose_trajectory(q0Deg, qfDeg, duration, timeStep);

sampleCount = numel(time);
cableLength = zeros(3, sampleCount);
cableSpeed = zeros(3, sampleCount);
jacobianCondition = zeros(1, sampleCount);

for sampleIndex = 1:sampleCount
    q = qTrajectory(:, sampleIndex);
    rotation = rpy_rotation(q(1), q(2), 0);
    position = [0; 0; q(3)];

    %逆运动学计算
    cableLength(:, sampleIndex) = calc_cable_length( fixedPoints, movingPoints, position, rotation);
    %速度雅各比计算
    jacobian = calc_cable_jacobian( fixedPoints, movingPoints, q, rotation);
    %计算绳长变化速度
    cableSpeed(:, sampleIndex) = jacobian * qdotTrajectory(:, sampleIndex);

    jacobianCondition(sampleIndex) = cond(jacobian);
end

actuatorCommand = convert_to_actuator_cmd(cableLength, drumRadius);

fprintf("最终绳长 / mm：\n");
disp(cableLength(:, end));
fprintf("最终卷筒角度 / deg：\n");
disp(rad2deg(actuatorCommand(:, end)));

animate_cable_joint( ...
    time, qTrajectory, fixedPoints, movingPoints, ...
    fixedPlatformRadius, movingPlatformRadius, ...
    PlatformThickness=platformThickness, ...
    ActuatorRadius=actuatorRadius, ...
    ExportVideo=exportVideo, ...
    VideoFile=videoFile);

plot_results(time, qTrajectory, cableLength, cableSpeed, ...
    actuatorCommand, jacobianCondition);

function points = platform_points(radius)
    % 返回圆周上均匀分布的三个连接点。

    points = [radius,    -radius / 2,          -radius / 2;
              0,         sqrt(3) * radius / 2, -sqrt(3) * radius / 2;
              0,         0,                    0];
end
