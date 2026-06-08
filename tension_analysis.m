function result = tension_analysis( ...
        fixedPoints, movingPoints, qTrajectory, targetLoadTrajectory, options)
%TENSION_ANALYSIS 沿位姿轨迹逐点执行静力学分析。
%   qTrajectory 为 3xN，[alpha; beta; h]，角度单位 rad，长度单位 mm。
%   targetLoadTrajectory 为 3xN，[Mx; My; Fz]，力矩单位 N*mm，力单位 N。

    arguments
        fixedPoints (3, 3) double {mustBeFinite}
        movingPoints (3, 3) double {mustBeFinite}
        qTrajectory (3, :) double {mustBeFinite}
        targetLoadTrajectory (3, :) double {mustBeFinite}
        options.PreferredForce (4, 1) double {mustBeFinite} = [30; 30; 30; 20]
        options.MinCableTension (1, 1) double {mustBeFinite} = 0
        options.ResidualTolerance (1, 1) double {mustBeFinite, mustBeNonnegative} = 1e-8
    end

    sampleCount = size(qTrajectory, 2);
    if size(targetLoadTrajectory, 2) ~= sampleCount
        error("CableJointStatic:TrajectorySize", ...
            "targetLoadTrajectory 必须和 qTrajectory 有相同的采样点数量。");
    end

    forceTrajectory = zeros(4, sampleCount);
    baseForceTrajectory = zeros(4, sampleCount);
    residualTrajectory = zeros(3, sampleCount);
    residualNorm = zeros(1, sampleCount);
    staticMatrixTrajectory = zeros(3, 4, sampleCount);
    cableLengthTrajectory = zeros(3, sampleCount);
    cableUnitTrajectory = zeros(3, 3, sampleCount);
    momentArmTrajectory = zeros(3, 3, sampleCount);
    isCableTaut = false(3, sampleCount);
    isFeasible = false(1, sampleCount);

    for sampleIndex = 1:sampleCount
        q = qTrajectory(:, sampleIndex);
        position = [0; 0; q(3)];
        rotation = rpy_rotation(q(1), q(2), 0);

        [staticMatrix, cableLength, cableUnit, momentArm] = ...
            calc_static_matrix(fixedPoints, movingPoints, position, rotation);
        sampleResult = solve_tension_distribution( ...
            staticMatrix, targetLoadTrajectory(:, sampleIndex), ...
            PreferredForce=options.PreferredForce, ...
            MinCableTension=options.MinCableTension, ...
            ResidualTolerance=options.ResidualTolerance);

        forceTrajectory(:, sampleIndex) = sampleResult.force;
        baseForceTrajectory(:, sampleIndex) = sampleResult.baseForce;
        residualTrajectory(:, sampleIndex) = sampleResult.residual;
        residualNorm(sampleIndex) = sampleResult.residualNorm;
        staticMatrixTrajectory(:, :, sampleIndex) = staticMatrix;
        cableLengthTrajectory(:, sampleIndex) = cableLength;
        cableUnitTrajectory(:, :, sampleIndex) = cableUnit;
        momentArmTrajectory(:, :, sampleIndex) = momentArm;
        isCableTaut(:, sampleIndex) = sampleResult.isCableTaut;
        isFeasible(sampleIndex) = sampleResult.isFeasible;
    end

    result = struct;
    result.forceTrajectory = forceTrajectory;
    result.baseForceTrajectory = baseForceTrajectory;
    result.residualTrajectory = residualTrajectory;
    result.residualNorm = residualNorm;
    result.staticMatrix = staticMatrixTrajectory;
    result.cableLength = cableLengthTrajectory;
    result.cableUnit = cableUnitTrajectory;
    result.momentArm = momentArmTrajectory;
    result.isCableTaut = isCableTaut;
    result.isFeasible = isFeasible;
    result.minCableTension = min(forceTrajectory(1:3, :), [], 1);
    result.targetLoadTrajectory = targetLoadTrajectory;
end
