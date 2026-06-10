%% 绳驱三自由度关节正运动学工作空间分析
clear;
clc;
close all;

scriptFolder = fileparts(mfilename("fullpath"));
projectRoot = fileparts(scriptFolder);
addpath(projectRoot, scriptFolder);

fixedPlatformRadius = 160;
movingPlatformRadius = 80;
drumRadius = 15;
fixedPoints = platform_points(fixedPlatformRadius);
movingPoints = platform_points(movingPlatformRadius);

actuatorAngleLimitDeg = [-135, 135];
linearActuatorRange = [50, 150];
sampleCount = 100000;
randomSeed = 0;

referencePoseDeg = [0; 0; 100];
poseAngleLimitDeg = [-90, 90];
initialGuessDeg = [0, 30, -30, 0, 0; 0, 0, 0, 30, -30];
forwardTolerance = 1e-6;
maxIterations = 30;

targetLoad = [0; 0; 50];
preferredForce = [30; 30; 30; 20];
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
plot_workspace_cloud(result);

function points = platform_points(radius)
    points = [radius, -radius / 2, -radius / 2; ...
        0, sqrt(3) * radius / 2, -sqrt(3) * radius / 2; ...
        0, 0, 0];
end
