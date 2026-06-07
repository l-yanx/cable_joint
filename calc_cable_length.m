function [cableLength, cableVector, unitVector] = ...
        calc_cable_length(fixedPoints, movingPoints, position, rotation)
% 计算绳向量、绳长和绳单位方向向量

    cableVector = position + rotation * movingPoints - fixedPoints;
    cableLength = vecnorm(cableVector, 2, 1).';
    unitVector = cableVector ./ cableLength.';
end
