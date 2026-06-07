classdef CableJointAnimationTest < matlab.unittest.TestCase
    %CableJointAnimationTest 绳驱关节动画行为测试。

    methods (TestClassSetup)
        function addProjectRootToPath(testCase)
            projectRoot = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture( ...
                matlab.unittest.fixtures.PathFixture(projectRoot));
        end
    end

    methods (TestMethodSetup)
        function hideFigures(testCase)
            originalVisibility = get(groot, "defaultFigureVisible");
            set(groot, "defaultFigureVisible", "off");
            testCase.addTeardown( ...
                @() set(groot, "defaultFigureVisible", originalVisibility));
            testCase.addTeardown(@() close("all", "force"));
        end
    end

    methods (Test)
        function testRejectsMismatchedTrajectoryLength(testCase)
            [fixedPoints, movingPoints] = ...
                CableJointAnimationTest.platformGeometry();

            testCase.verifyError(@() animate_cable_joint( ...
                [0, 0.1], zeros(3, 3), fixedPoints, movingPoints, ...
                160, 80, RealtimePlayback=false, Visible="off"), ...
                "CableJointAnimation:TrajectorySize");
        end

        function testCreatesTaggedStaticGeometry(testCase)
            [fixedPoints, movingPoints] = ...
                CableJointAnimationTest.platformGeometry();

            figureHandle = animate_cable_joint( ...
                0, [0; 0; 39], fixedPoints, movingPoints, ...
                160, 80, PlatformThickness=1, ActuatorRadius=5, ...
                RealtimePlayback=false, Visible="off");

            axesHandle = findobj(figureHandle, Tag="CableJointAxes");
            fixedPlatform = findobj(figureHandle, Tag="FixedPlatform");
            fixedPlatformSurfaces = findobj(fixedPlatform, Type="surface");
            actuator = findobj(figureHandle, Tag="CenterActuator");

            testCase.verifyNumElements(axesHandle, 1);
            testCase.verifyNumElements(fixedPlatform, 1);
            testCase.verifyNumElements(actuator, 1);
            testCase.verifyEqual(axesHandle.DataAspectRatio, [1, 1, 1]);
            testCase.verifyEqual( ...
                max(actuator.XData, [], "all"), 5, AbsTol=1e-10);
            testCase.verifyEqual( ...
                min(actuator.XData, [], "all"), -5, AbsTol=1e-10);
            testCase.verifyEqual( ...
                [min(arrayfun(@(surfaceHandle) ...
                    min(surfaceHandle.ZData, [], "all"), ...
                    fixedPlatformSurfaces)), ...
                 max(arrayfun(@(surfaceHandle) ...
                    max(surfaceHandle.ZData, [], "all"), ...
                    fixedPlatformSurfaces))], ...
                [-0.5, 0.5], AbsTol=1e-10);
        end
    end

    methods (Static, Access=private)
        function [fixedPoints, movingPoints] = platformGeometry()
            fixedPoints = [ ...
                160, -80, -80;
                0, 80 * sqrt(3), -80 * sqrt(3);
                0, 0, 0];
            movingPoints = [ ...
                80, -40, -40;
                0, 40 * sqrt(3), -40 * sqrt(3);
                0, 0, 0];
        end
    end
end
