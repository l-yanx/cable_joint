function solution = solve_forward_kinematics( ...
        fixedPoints, movingPoints, actuatorInput, referenceCableLength, ...
        drumRadius, options)
%SOLVE_FORWARD_KINEMATICS Solve platform angles from two drum angles and h.

    arguments
        fixedPoints
        movingPoints
        actuatorInput
        referenceCableLength
        drumRadius
        options.InitialGuessDeg = [0, 30, -30, 0, 0; 0, 0, 0, 30, -30]
        options.PoseAngleLimitDeg = [-90, 90]
        options.ForwardTolerance = 1e-6
        options.MaxIterations = 30
    end

    validateConfiguration(fixedPoints, movingPoints, actuatorInput, ...
        referenceCableLength, drumRadius, options.PoseAngleLimitDeg, ...
        options.ForwardTolerance, options.MaxIterations);
    validateInitialGuesses(options.InitialGuessDeg, options.PoseAngleLimitDeg);

    targetCableLength = referenceCableLength(1:2) ...
        + drumRadius * actuatorInput(1:2);
    if any(targetCableLength <= 0)
        solution = emptySolution;
        return;
    end

    angleLimit = deg2rad(options.PoseAngleLimitDeg);
    initialGuess = deg2rad(options.InitialGuessDeg);
    best = emptySolution;

    for guessIndex = 1:size(initialGuess, 2)
        candidate = solveFromGuess(initialGuess(:, guessIndex));
        if candidate.residualNorm < best.residualNorm
            best = candidate;
        end
    end
    solution = best;

    function candidate = solveFromGuess(angles)
        candidate = evaluateCandidate(angles, 0, false);
        for iteration = 1:options.MaxIterations
            if candidate.residualNorm <= options.ForwardTolerance
                candidate.converged = true;
                return;
            end

            rotation = rpy_rotation(angles(1), angles(2), 0);
            fullJacobian = calc_cable_jacobian( ...
                fixedPoints, movingPoints, [angles; actuatorInput(3)], rotation);
            forwardJacobian = fullJacobian(1:2, 1:2);
            if any(~isfinite(forwardJacobian), "all") ...
                    || rcond(forwardJacobian) < 1e-12
                return;
            end

            delta = -forwardJacobian \ candidate.residual;
            if any(~isfinite(delta))
                return;
            end

            accepted = false;
            stepScale = 1;
            while stepScale >= 2^-12
                trialAngles = angles + stepScale * delta;
                trialAngles = min(max(trialAngles, angleLimit(1)), angleLimit(2));
                trial = evaluateCandidate(trialAngles, iteration, false);
                if trial.residualNorm < candidate.residualNorm
                    angles = trialAngles;
                    candidate = trial;
                    accepted = true;
                    break;
                end
                stepScale = stepScale / 2;
            end
            if ~accepted
                return;
            end
        end
        candidate.converged = ...
            candidate.residualNorm <= options.ForwardTolerance;
    end

    function candidate = evaluateCandidate(angles, iterationCount, converged)
        rotation = rpy_rotation(angles(1), angles(2), 0);
        cableLength = calc_cable_length( ...
            fixedPoints, movingPoints, [0; 0; actuatorInput(3)], rotation);
        residual = cableLength(1:2) - targetCableLength;
        candidate = struct( ...
            alpha=angles(1), beta=angles(2), cableLength=cableLength, ...
            residual=residual, residualNorm=norm(residual, inf), ...
            iterationCount=iterationCount, converged=converged);
    end
end

function validateConfiguration(fixedPoints, movingPoints, actuatorInput, ...
        referenceCableLength, drumRadius, poseAngleLimitDeg, ...
        forwardTolerance, maxIterations)
    isValid = isFiniteDoubleArray(fixedPoints, [3, 3]) ...
        && isFiniteDoubleArray(movingPoints, [3, 3]) ...
        && isFiniteDoubleArray(actuatorInput, [3, 1]) ...
        && isFiniteDoubleArray(referenceCableLength, [3, 1]) ...
        && all(referenceCableLength > 0) ...
        && isFiniteScalar(drumRadius) && drumRadius > 0 ...
        && isFiniteDoubleArray(poseAngleLimitDeg, [1, 2]) ...
        && poseAngleLimitDeg(1) < poseAngleLimitDeg(2) ...
        && isFiniteScalar(forwardTolerance) && forwardTolerance > 0 ...
        && isPositiveInteger(maxIterations);
    if ~isValid
        error("CableJointWorkspace:InvalidConfiguration", ...
            "正运动学配置无效。");
    end
end

function validateInitialGuesses(initialGuessDeg, poseAngleLimitDeg)
    isValid = isa(initialGuessDeg, "double") && isreal(initialGuessDeg) ...
        && ismatrix(initialGuessDeg) && size(initialGuessDeg, 1) == 2 ...
        && ~isempty(initialGuessDeg) && all(isfinite(initialGuessDeg), "all") ...
        && all(initialGuessDeg >= poseAngleLimitDeg(1), "all") ...
        && all(initialGuessDeg <= poseAngleLimitDeg(2), "all");
    if ~isValid
        error("CableJointWorkspace:InvalidInitialGuess", ...
            "初值必须是位于姿态角范围内的有限 2xN 数组。");
    end
end

function tf = isFiniteDoubleArray(value, expectedSize)
    tf = isa(value, "double") && isreal(value) ...
        && isequal(size(value), expectedSize) && all(isfinite(value), "all");
end

function tf = isFiniteScalar(value)
    tf = isa(value, "double") && isreal(value) ...
        && isscalar(value) && isfinite(value);
end

function tf = isPositiveInteger(value)
    tf = isFiniteScalar(value) && value > 0 && value == fix(value);
end

function solution = emptySolution
    solution = struct( ...
        alpha=NaN, beta=NaN, cableLength=NaN(3, 1), ...
        residual=[Inf; Inf], residualNorm=Inf, ...
        iterationCount=0, converged=false);
end
