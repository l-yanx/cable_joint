function [qTrajectory, qdotTrajectory, time] = ...
        generate_pose_trajectory(q0Deg, qfDeg, duration, timeStep)
%   生成起止速度和加速度均为零的五次多项式姿态轨迹。
%   输入向量的前两个分量为角度，单位为度。
%   输出的内部角度和角速度单位分别为弧度和弧度每秒。

    q0 = [deg2rad(q0Deg(1:2)); q0Deg(3)];
    qf = [deg2rad(qfDeg(1:2)); qfDeg(3)];
    time = 0:timeStep:duration;
    if time(end) < duration
        time = [time, duration];
    end

    normalizedTime = time / duration;
    positionScale = 10 * normalizedTime.^3 ...
        - 15 * normalizedTime.^4 + 6 * normalizedTime.^5;
    velocityScale = (30 * normalizedTime.^2 ...
        - 60 * normalizedTime.^3 + 30 * normalizedTime.^4) / duration;

    displacement = qf - q0;
    qTrajectory = q0 + displacement * positionScale;
    qdotTrajectory = displacement * velocityScale;
end
