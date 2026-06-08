classdef CableJointStaticAnalysisTest < matlab.unittest.TestCase
    %CableJointStaticAnalysisTest 静力学分析模块测试。

    methods (TestClassSetup)
        function addProjectRootToPath(testCase)
            projectRoot = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture( ...
                matlab.unittest.fixtures.PathFixture(projectRoot));
        end
    end

    methods (Test)
        function testStaticMatrixUsesCableForceDirection(testCase)
            fixedPoints = CableJointStaticAnalysisTest.platformPoints(160);
            movingPoints = CableJointStaticAnalysisTest.platformPoints(80);
            position = [0; 0; 39];
            rotation = rpy_rotation(0, 0, 0);

            [staticMatrix, cableLength, cableUnit, momentArm] = ...
                calc_static_matrix(fixedPoints, movingPoints, position, rotation);

            movingWorld = position + rotation * movingPoints;
            expectedCableVector = fixedPoints - movingWorld;
            expectedCableLength = vecnorm(expectedCableVector, 2, 1).';
            expectedCableUnit = expectedCableVector ./ expectedCableLength.';
            expectedMomentArm = movingWorld - position;
            expectedStaticMatrix = zeros(3, 4);
            for cableIndex = 1:3
                moment = cross( ...
                    expectedMomentArm(:, cableIndex), ...
                    expectedCableUnit(:, cableIndex));
                expectedStaticMatrix(:, cableIndex) = ...
                    [moment(1); moment(2); expectedCableUnit(3, cableIndex)];
            end
            expectedStaticMatrix(:, 4) = [0; 0; 1];

            testCase.verifyEqual(cableLength, expectedCableLength, AbsTol=1e-12);
            testCase.verifyEqual(cableUnit, expectedCableUnit, AbsTol=1e-12);
            testCase.verifyEqual(momentArm, expectedMomentArm, AbsTol=1e-12);
            testCase.verifyEqual(staticMatrix, expectedStaticMatrix, AbsTol=1e-12);
        end

        function testSolveTensionDistributionBalancesTargetLoad(testCase)
            staticMatrix = [ ...
                0, 10, -10, 0; ...
               -8,  4,   4, 0; ...
               -1, -1,  -1, 1];
            targetLoad = [20; -16; 30];

            result = solve_tension_distribution( ...
                staticMatrix, targetLoad, PreferredForce=[12; 12; 12; 50]);

            testCase.verifySize(result.force, [4, 1]);
            testCase.verifyEqual(staticMatrix * result.force, targetLoad, AbsTol=1e-10);
            testCase.verifyEqual(result.residual, zeros(3, 1), AbsTol=1e-10);
            testCase.verifyEqual(result.isCableTaut, result.force(1:3) > 0);
            testCase.verifyEqual(result.isFeasible, all(result.force(1:3) > 0));
        end

        function testTensionAnalysisComputesEveryPoseSample(testCase)
            fixedPoints = CableJointStaticAnalysisTest.platformPoints(160);
            movingPoints = CableJointStaticAnalysisTest.platformPoints(80);
            qTrajectory = [ ...
                0, deg2rad(4), deg2rad(-2); ...
                0, deg2rad(-3), deg2rad(5); ...
                39, 42, 45];
            targetLoadTrajectory = repmat([200; 300; 50], 1, 3);

            result = tension_analysis( ...
                fixedPoints, movingPoints, qTrajectory, targetLoadTrajectory, ...
                PreferredForce=[30; 30; 30; 20]);

            testCase.verifySize(result.forceTrajectory, [4, 3]);
            testCase.verifySize(result.residualTrajectory, [3, 3]);
            testCase.verifySize(result.staticMatrix, [3, 4, 3]);
            testCase.verifySize(result.cableUnit, [3, 3, 3]);
            testCase.verifySize(result.isFeasible, [1, 3]);
            for sampleIndex = 1:3
                testCase.verifyEqual( ...
                    result.staticMatrix(:, :, sampleIndex) ...
                    * result.forceTrajectory(:, sampleIndex), ...
                    targetLoadTrajectory(:, sampleIndex), AbsTol=1e-8);
            end
        end

        function testPlotStaticResultsOpensDedicatedPage(testCase)
            originalVisibility = get(groot, "defaultFigureVisible");
            set(groot, "defaultFigureVisible", "off");
            testCase.addTeardown( ...
                @() set(groot, "defaultFigureVisible", originalVisibility));
            testCase.addTeardown(@() close("all"));

            time = 0:0.5:1;
            staticResult.forceTrajectory = [ ...
                10, 12, 14; ...
                20, 22, 24; ...
                30, 32, 34; ...
                40, 44, 48];
            staticResult.residualTrajectory = zeros(3, 3);
            staticResult.minCableTension = [10, 12, 14];
            staticResult.isFeasible = [true, true, false];

            plot_static_results(time, staticResult);

            figures = findall(groot, Type="figure");
            axesHandles = findall(figures, Type="axes");
            axesTitles = string(arrayfun( ...
                @(axesHandle) axesHandle.Title.String, axesHandles, ...
                UniformOutput=false));

            testCase.verifyNumElements(figures, 1);
            testCase.verifyEqual(string(figures.Name), "绳驱关节静力学结果");
            testCase.verifyNumElements(axesHandles, 4);
            testCase.verifyTrue(any(axesTitles == "三根绳张力"));
            testCase.verifyTrue(any(axesTitles == "中央直线执行器力"));
            testCase.verifyTrue(any(axesTitles == "静力学平衡残差"));
            testCase.verifyTrue(any(axesTitles == "最小绳张力与可行性"));
        end
    end

    methods (Static, Access = private)
        function points = platformPoints(radius)
            points = [ ...
                radius, -radius / 2, -radius / 2; ...
                0, sqrt(3) * radius / 2, -sqrt(3) * radius / 2; ...
                0, 0, 0];
        end
    end
end
