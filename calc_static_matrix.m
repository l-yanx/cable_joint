function [staticMatrix, cableLength, cableUnit, momentArm] = ...
        calc_static_matrix(fixedPoints, movingPoints, position, rotation)
%CALC_STATIC_MATRIX 计算三绳加中央直线执行器的静力学矩阵。
%   staticMatrix 将 [T1; T2; T3; Fa] 映射到 [Mx; My; Fz]。

    arguments
        fixedPoints (3, 3) double {mustBeFinite}
        movingPoints (3, 3) double {mustBeFinite}
        position (3, 1) double {mustBeFinite}
        rotation (3, 3) double {mustBeFinite}
    end

    staticMatrix = zeros(3, 4);
    cableLength = zeros(3, 1);
    cableUnit = zeros(3, 3);
    momentArm = zeros(3, 3);

    movingWorld = position + rotation * movingPoints;

    for cableIndex = 1:3
        momentArm(:, cableIndex) = movingWorld(:, cableIndex) - position;

        % 绳对动平台的拉力方向：从动平台接绳点指向基座出绳点。
        cableVector = fixedPoints(:, cableIndex) - movingWorld(:, cableIndex);
        cableLength(cableIndex) = norm(cableVector);
        if cableLength(cableIndex) < 1e-9
            error("CableJointStatic:DegenerateCable", ...
                "第 %d 根绳长度过小，无法计算静力学矩阵。", cableIndex);
        end

        cableUnit(:, cableIndex) = cableVector / cableLength(cableIndex);
        moment = cross(momentArm(:, cableIndex), cableUnit(:, cableIndex));
        staticMatrix(:, cableIndex) = [ ...
            moment(1); ...
            moment(2); ...
            cableUnit(3, cableIndex)];
    end

    staticMatrix(:, 4) = [0; 0; 1];
end
