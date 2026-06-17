%% 绳驱三自由度关节正运动学工作空间分析
clear;
clc;
close all;

scriptFolder = fileparts(mfilename("fullpath"));
projectRoot = fileparts(scriptFolder);
addpath(projectRoot, scriptFolder);

fixedPlatformRadius = 93;
movingPlatformRadius = 64.5;
drumRadius = 22.5;
fixedPoints = platform_points(fixedPlatformRadius);
movingPoints = platform_points(movingPlatformRadius);

actuatorAngleLimitDeg = [-135, 135];
linearActuatorRange = [150, 160];
sampleCount = 100000;
randomSeed = 0;

referencePoseDeg = [0; 0; 151];
poseAngleLimitDeg = [-90, 90];
initialGuessDeg = [0, 30, -30, 0, 0; 0, 0, 0, 30, -30];
forwardTolerance = 1e-6;
maxIterations = 30;

targetLoad = [0; 0; 1.2];
preferredForce = [1; 1; 1; 20];
minCableTension = 0;
staticResidualTolerance = 1e-8;

result = analyze_workspace( ...
    fixedPoints, movingPoints, drumRadius, ...
    ActuatorAngleLimitDeg=actuatorAngleLimitDeg, ...
    LinearActuatorRange=linearActuatorRange, SampleCount=sampleCount, ...
    RandomSeed=randomSeed, ReferencePoseDeg=referencePoseDeg, ...
    PoseAngleLimitDeg=poseAngleLimitDeg, InitialGuessDeg=initialGuessDeg, ...
    ForwardTolerance=forwardTolerance, MaxIterations=maxIterations, ...
    TargetLoad=targetLoad, PreferredForce=preferredForce, ...
    MinCableTension=minCableTension, ...
    StaticResidualTolerance=staticResidualTolerance);

fprintf("工作空间扫描点数：%d\n", sampleCount);
fprintf("正运动学收敛点数：%d\n", nnz(result.forwardConverged));
fprintf("最终可行点数：%d\n", nnz(result.isFeasible));

if any(result.isFeasible)
    feasiblePose = result.pose(:, result.isFeasible);
    fprintf("可行姿态 alpha 范围 / deg：[%.3f, %.3f]\n", ...
        min(rad2deg(feasiblePose(1, :))), max(rad2deg(feasiblePose(1, :))));
    fprintf("可行姿态 beta 范围 / deg：[%.3f, %.3f]\n", ...
        min(rad2deg(feasiblePose(2, :))), max(rad2deg(feasiblePose(2, :))));
    fprintf("可行姿态 h 范围 / mm：[%.3f, %.3f]\n", ...
        min(feasiblePose(3, :)), max(feasiblePose(3, :)));
else
    fprintf("没有可行姿态，无法计算姿态范围。\n");
end

plot_workspace_cloud(result);

function points = platform_points(radius)
    points = [radius, -radius / 2, -radius / 2; ...
        0, sqrt(3) * radius / 2, -sqrt(3) * radius / 2; ...
        0, 0, 0];
end
