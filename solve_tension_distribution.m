function result = solve_tension_distribution(staticMatrix, targetLoad, options)
%SOLVE_TENSION_DISTRIBUTION 求解满足目标广义力的绳张力和直线执行器力。

    arguments
        staticMatrix (3, 4) double {mustBeFinite}
        targetLoad (3, 1) double {mustBeFinite}
        options.PreferredForce (4, 1) double {mustBeFinite} = [30; 30; 30; 20]
        options.MinCableTension (1, 1) double {mustBeFinite} = 0
        options.ResidualTolerance (1, 1) double {mustBeFinite, mustBeNonnegative} = 1e-8
    end

    baseForce = pinv(staticMatrix) * targetLoad;
    nullBasis = null(staticMatrix);

    if isempty(nullBasis)
        force = baseForce;
    else
        lambda = pinv(nullBasis) * (options.PreferredForce - baseForce);
        force = baseForce + nullBasis * lambda;
    end

    residual = staticMatrix * force - targetLoad;
    isCableTaut = force(1:3) > options.MinCableTension;

    result = struct;
    result.force = force;
    result.baseForce = baseForce;
    result.nullBasis = nullBasis;
    result.residual = residual;
    result.residualNorm = norm(residual);
    result.isCableTaut = isCableTaut;
    result.isFeasible = all(isCableTaut) ...
        && result.residualNorm <= options.ResidualTolerance;
end
