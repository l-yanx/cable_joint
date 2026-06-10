classdef WorkspaceAnalysisTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addRequiredPaths(testCase)
            workspaceFolder = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fileparts(workspaceFolder)));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                workspaceFolder));
        end
    end

    methods (Test)
        function testForwardKinematicsReturnsReferencePose(testCase)
            s = WorkspaceAnalysisTest.defaultSetup();
            solution = WorkspaceAnalysisTest.solve(s, [0; 0; 100]);
            testCase.verifyTrue(solution.converged);
            testCase.verifyEqual([solution.alpha; solution.beta], ...
                zeros(2, 1), AbsTol=1e-12);
            testCase.verifyEqual(solution.cableLength, ...
                s.referenceCableLength, AbsTol=1e-12);
        end

        function testForwardKinematicsRecoversKnownPose(testCase)
            s = WorkspaceAnalysisTest.defaultSetup();
            expected = deg2rad([12; -9]);
            h = 115;
            lengths = calc_cable_length(s.fixedPoints, s.movingPoints, ...
                [0; 0; h], rpy_rotation(expected(1), expected(2), 0));
            input = [(lengths(1:2) - s.referenceCableLength(1:2)) ...
                / s.drumRadius; h];
            solution = WorkspaceAnalysisTest.solve(s, input);
            testCase.verifyTrue(solution.converged);
            testCase.verifyEqual([solution.alpha; solution.beta], ...
                expected, AbsTol=1e-8);
            testCase.verifyLessThanOrEqual(solution.residualNorm, 1e-6);
        end

        function testInvalidInitialGuessIdentifier(testCase)
            s = WorkspaceAnalysisTest.defaultSetup();
            testCase.verifyError(@() solve_forward_kinematics( ...
                s.fixedPoints, s.movingPoints, [0; 0; 100], ...
                s.referenceCableLength, s.drumRadius, ...
                InitialGuessDeg=[0; 100]), ...
                "CableJointWorkspace:InvalidInitialGuess");
        end

        function testInvalidConfigurationIdentifier(testCase)
            s = WorkspaceAnalysisTest.defaultSetup();
            testCase.verifyError(@() analyze_workspace( ...
                s.fixedPoints, s.movingPoints, -1, SampleCount=1), ...
                "CableJointWorkspace:InvalidConfiguration");
        end

        function testAnalysisArraySizes(testCase)
            s = WorkspaceAnalysisTest.defaultSetup();
            result = WorkspaceAnalysisTest.analyze(s, SampleCount=12);
            testCase.verifySize(result.pose, [3, 12]);
            testCase.verifySize(result.actuatorCommand, [4, 12]);
            testCase.verifySize(result.cableLength, [3, 12]);
            testCase.verifySize(result.force, [4, 12]);
            fields = ["minCableTension", "forwardResidualNorm", ...
                "staticResidualNorm", "forwardConverged", ...
                "withinActuatorLimits", "isTensionFeasible", "isFeasible"];
            for field = fields
                testCase.verifySize(result.(field), [1, 12]);
            end
        end

        function testSamplingReproducibility(testCase)
            s = WorkspaceAnalysisTest.defaultSetup();
            first = WorkspaceAnalysisTest.analyze(s, ...
                SampleCount=10, RandomSeed=23);
            second = WorkspaceAnalysisTest.analyze(s, ...
                SampleCount=10, RandomSeed=23);
            different = WorkspaceAnalysisTest.analyze(s, ...
                SampleCount=10, RandomSeed=24);
            testCase.verifyEqual(first.actuatorCommand, second.actuatorCommand);
            testCase.verifyEqual(first.pose, second.pose);
            testCase.verifyNotEqual(first.actuatorCommand([1, 2, 4], :), ...
                different.actuatorCommand([1, 2, 4], :));
        end

        function testThirdDrumAngleReconstruction(testCase)
            s = WorkspaceAnalysisTest.defaultSetup();
            result = WorkspaceAnalysisTest.analyze(s, ...
                SampleCount=15, RandomSeed=3);
            mask = result.forwardConverged;
            expected = (result.cableLength(3, mask) ...
                - s.referenceCableLength(3)) / s.drumRadius;
            testCase.verifyGreaterThan(nnz(mask), 0);
            testCase.verifyEqual(result.actuatorCommand(3, mask), ...
                expected, AbsTol=1e-12);
        end

        function testThirdAngleLimitRejectsSamples(testCase)
            s = WorkspaceAnalysisTest.defaultSetup();
            result = WorkspaceAnalysisTest.analyze(s, ...
                ActuatorAngleLimitDeg=[-1, 1], ...
                SampleCount=40, RandomSeed=4);
            theta3 = rad2deg(result.actuatorCommand(3, :));
            mask = result.forwardConverged & (theta3 < -1 | theta3 > 1);
            testCase.verifyGreaterThan(nnz(mask), 0);
            testCase.verifyFalse(any(result.withinActuatorLimits(mask)));
            testCase.verifyFalse(any(result.isFeasible(mask)));
        end

        function testMinimumTensionRejectsSamples(testCase)
            s = WorkspaceAnalysisTest.defaultSetup();
            result = WorkspaceAnalysisTest.analyze(s, ...
                SampleCount=20, RandomSeed=5, MinCableTension=1e6);
            testCase.verifyFalse(any(result.isTensionFeasible));
            testCase.verifyFalse(any(result.isFeasible));
        end

        function testForwardFailureDoesNotAbortSampling(testCase)
            s = WorkspaceAnalysisTest.defaultSetup();
            result = WorkspaceAnalysisTest.analyze(s, ...
                ActuatorAngleLimitDeg=[-500, 500], ...
                SampleCount=30, RandomSeed=8, MaxIterations=1);
            testCase.verifyGreaterThan(nnz(~result.forwardConverged), 0);
            testCase.verifySize(result.forwardConverged, [1, 30]);
            testCase.verifyFalse(any(result.isFeasible( ...
                ~result.forwardConverged)));
        end

        function testPlotUsesOnlyFeasiblePoints(testCase)
            WorkspaceAnalysisTest.hideFigures(testCase);
            result.pose = [deg2rad([1, 2, 3, 4]); ...
                deg2rad([-1, -2, -3, -4]); [80, 90, 100, 110]];
            result.minCableTension = [12, 22, 32, 42];
            result.isFeasible = [true, false, true, false];
            f = plot_workspace_cloud(result);
            scatterHandle = findobj(f, Type="Scatter");
            testCase.verifyEqual(scatterHandle.XData, [1, 3], AbsTol=1e-12);
            testCase.verifyEqual(scatterHandle.YData, [-1, -3], AbsTol=1e-12);
            testCase.verifyEqual(scatterHandle.ZData, [80, 100]);
            testCase.verifyEqual(scatterHandle.CData, [12, 32]);
        end

        function testPlotHandlesEmptySet(testCase)
            WorkspaceAnalysisTest.hideFigures(testCase);
            result.pose = zeros(3, 2);
            result.minCableTension = [NaN, NaN];
            result.isFeasible = [false, false];
            state = warning("off", "CableJointWorkspace:NoFeasiblePoints");
            testCase.addTeardown(@() warning(state));
            f = plot_workspace_cloud(result);
            testCase.verifyEmpty(findobj(f, Type="Scatter"));
            testCase.verifyNotEmpty(findobj(f, Type="axes"));
        end
    end

    methods (Static, Access = private)
        function s = defaultSetup()
            s.fixedPoints = WorkspaceAnalysisTest.platformPoints(160);
            s.movingPoints = WorkspaceAnalysisTest.platformPoints(80);
            s.drumRadius = 15;
            s.referenceCableLength = calc_cable_length( ...
                s.fixedPoints, s.movingPoints, [0; 0; 100], ...
                rpy_rotation(0, 0, 0));
        end

        function solution = solve(s, input)
            solution = solve_forward_kinematics( ...
                s.fixedPoints, s.movingPoints, input, ...
                s.referenceCableLength, s.drumRadius, ...
                InitialGuessDeg=[0, 30, -30, 0, 0; 0, 0, 0, 30, -30]);
        end

        function result = analyze(s, options)
            arguments
                s
                options.ActuatorAngleLimitDeg = [-135, 135]
                options.SampleCount = 20
                options.RandomSeed = 0
                options.MinCableTension = 0
                options.MaxIterations = 30
            end
            result = analyze_workspace(s.fixedPoints, s.movingPoints, ...
                s.drumRadius, ...
                ActuatorAngleLimitDeg=options.ActuatorAngleLimitDeg, ...
                SampleCount=options.SampleCount, ...
                RandomSeed=options.RandomSeed, ...
                MinCableTension=options.MinCableTension, ...
                MaxIterations=options.MaxIterations);
        end

        function hideFigures(testCase)
            old = get(groot, "defaultFigureVisible");
            set(groot, "defaultFigureVisible", "off");
            testCase.addTeardown( ...
                @() set(groot, "defaultFigureVisible", old));
            testCase.addTeardown(@() close("all"));
        end

        function points = platformPoints(radius)
            points = [radius, -radius / 2, -radius / 2; ...
                0, sqrt(3) * radius / 2, -sqrt(3) * radius / 2; ...
                0, 0, 0];
        end
    end
end
