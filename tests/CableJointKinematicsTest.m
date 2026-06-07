classdef CableJointKinematicsTest < matlab.unittest.TestCase
    %CableJointKinematicsTest 绳驱关节运动学模块测试。

    methods (TestClassSetup)
        function addProjectRootToPath(testCase)
            projectRoot = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture( ...
                matlab.unittest.fixtures.PathFixture(projectRoot));
        end
    end

    methods (Test)
        function testCableLengthAtZeroOrientation(testCase)
            fixedPoints = CableJointKinematicsTest.platformPoints(160);
            movingPoints = CableJointKinematicsTest.platformPoints(80);
            position = [0; 0; 39];
            rotation = rpy_rotation(0, 0, 0);

            cableLength = calc_cable_length( ...
                fixedPoints, movingPoints, position, rotation);

            expectedLength = sqrt(80^2 + 39^2) * ones(3, 1);
            testCase.verifyEqual(cableLength, expectedLength, AbsTol=1e-12);
        end

        function testJacobianMatchesFiniteDifference(testCase)
            fixedPoints = CableJointKinematicsTest.platformPoints(160);
            movingPoints = CableJointKinematicsTest.platformPoints(80);
            q = [deg2rad(5); deg2rad(3); 120];
            step = 1e-6;

            rotation = rpy_rotation(q(1), q(2), 0);
            jacobian = calc_cable_jacobian( ...
                fixedPoints, movingPoints, q, rotation);

            qPlus = q;
            qPlus(1) = qPlus(1) + step;
            qMinus = q;
            qMinus(1) = qMinus(1) - step;
            alphaDerivative = ( ...
                calc_cable_length(fixedPoints, movingPoints, qPlus(3) * [0; 0; 1], ...
                    rpy_rotation(qPlus(1), qPlus(2), 0)) ...
                - calc_cable_length(fixedPoints, movingPoints, qMinus(3) * [0; 0; 1], ...
                    rpy_rotation(qMinus(1), qMinus(2), 0))) / (2 * step);

            qPlus = q;
            qPlus(2) = qPlus(2) + step;
            qMinus = q;
            qMinus(2) = qMinus(2) - step;
            betaDerivative = ( ...
                calc_cable_length(fixedPoints, movingPoints, qPlus(3) * [0; 0; 1], ...
                    rpy_rotation(qPlus(1), qPlus(2), 0)) ...
                - calc_cable_length(fixedPoints, movingPoints, qMinus(3) * [0; 0; 1], ...
                    rpy_rotation(qMinus(1), qMinus(2), 0))) / (2 * step);

            qPlus = q;
            qPlus(3) = qPlus(3) + step;
            qMinus = q;
            qMinus(3) = qMinus(3) - step;
            heightDerivative = ( ...
                calc_cable_length(fixedPoints, movingPoints, qPlus(3) * [0; 0; 1], rotation) ...
                - calc_cable_length(fixedPoints, movingPoints, qMinus(3) * [0; 0; 1], rotation)) ...
                / (2 * step);

            finiteDifference = [alphaDerivative, betaDerivative, heightDerivative];
            testCase.verifyEqual(jacobian, finiteDifference, AbsTol=1e-6);
        end

        function testQuinticTrajectoryUsesRadiansInternally(testCase)
            q0Deg = [0; 0; 39];
            qfDeg = [10; -5; 49];

            [qTrajectory, qdotTrajectory, time] = ...
                generate_pose_trajectory(q0Deg, qfDeg, 2, 0.001);

            testCase.verifyEqual(time, 0:0.001:2, AbsTol=1e-12);
            testCase.verifyEqual(qTrajectory(:, 1), [0; 0; 39], AbsTol=1e-12);
            testCase.verifyEqual( ...
                qTrajectory(:, end), [deg2rad(10); deg2rad(-5); 49], AbsTol=1e-12);
            testCase.verifyEqual(qdotTrajectory(:, 1), zeros(3, 1), AbsTol=1e-12);
            testCase.verifyEqual(qdotTrajectory(:, end), zeros(3, 1), AbsTol=1e-12);
            testCase.verifyEqual( ...
                qTrajectory(:, 1001), [deg2rad(5); deg2rad(-2.5); 44], AbsTol=1e-12);

            startAcceleration = ...
                (qdotTrajectory(:, 2) - qdotTrajectory(:, 1)) / 0.001;
            endAcceleration = ...
                (qdotTrajectory(:, end) - qdotTrajectory(:, end - 1)) / 0.001;
            testCase.verifyLessThan(norm(startAcceleration), 0.5);
            testCase.verifyLessThan(norm(endAcceleration), 0.5);
        end

        function testDrumRadiusIsAnInput(testCase)
            cableLength = [10, 12; 20, 16; 30, 33];

            command15 = convert_to_actuator_cmd(cableLength, 15);
            command30 = convert_to_actuator_cmd(cableLength, 30);

            testCase.verifyEqual(command15, [0, 2; 0, -4; 0, 3] / 15, AbsTol=1e-12);
            testCase.verifyEqual(command30, command15 / 2, AbsTol=1e-12);
        end

        function testPlotResultsUsesOnePageWithFiveAxes(testCase)
            originalVisibility = get(groot, "defaultFigureVisible");
            set(groot, "defaultFigureVisible", "off");
            testCase.addTeardown( ...
                @() set(groot, "defaultFigureVisible", originalVisibility));
            testCase.addTeardown(@() close("all"));

            time = 0:0.5:1;
            qTrajectory = [deg2rad([0, 1, 2]); ...
                           deg2rad([0, 2, 4]); ...
                           [39, 40, 41]];
            cableLength = [80, 81, 82; 80, 82, 84; 80, 83, 86];
            cableSpeed = [2, 2, 2; 4, 4, 4; 6, 6, 6];
            actuatorCommand = cableLength / 15;
            jacobianCondition = [10, 11, 12];

            plot_results(time, qTrajectory, cableLength, cableSpeed, ...
                actuatorCommand, jacobianCondition);

            figures = findall(groot, Type="figure");
            axesHandles = findall(figures, Type="axes");
            lineCounts = arrayfun( ...
                @(axesHandle) numel(findobj(axesHandle, Type="line")), axesHandles);
            axesTitles = string(arrayfun( ...
                @(axesHandle) axesHandle.Title.String, axesHandles, ...
                UniformOutput=false));
            actuatorAxes = axesHandles(axesTitles == "执行器执行参数");
            actuatorLines = findobj(actuatorAxes, Type="line");

            testCase.verifyNumElements(figures, 1);
            testCase.verifyEqual(string(figures.Name), "绳驱关节运动学结果");
            testCase.verifyEqual(figures.Position, [100, 100, 1200, 800]);
            testCase.verifyNumElements(axesHandles, 5);
            testCase.verifyEqual(sort(lineCounts), [1; 1; 3; 3; 4]);
            testCase.verifyNumElements(actuatorAxes, 1);
            testCase.verifyNumElements(actuatorLines, 4);
            testCase.verifyEqual( ...
                sortrows(vertcat(actuatorLines.YData)), ...
                sortrows([rad2deg(actuatorCommand); qTrajectory(3, :)]), ...
                AbsTol=1e-12);
            testCase.verifyNotEmpty(findobj(figures, Type="axes", View=[-37.5, 30]));
        end
    end

    methods (Static, Access = private)
        function points = platformPoints(radius)
            points = [ ...
                radius, -radius / 2, -radius / 2;
                0, sqrt(3) * radius / 2, -sqrt(3) * radius / 2;
                0, 0, 0];
        end
    end
end
