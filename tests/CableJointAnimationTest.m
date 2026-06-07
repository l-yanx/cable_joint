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
            existingFigures = findall(groot, Type="figure");
            originalVisibility = get(groot, "defaultFigureVisible");
            set(groot, "defaultFigureVisible", "off");
            testCase.addTeardown( ...
                @() set(groot, "defaultFigureVisible", originalVisibility));
            testCase.addTeardown( ...
                @() CableJointAnimationTest.closeNewFigures(existingFigures));
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

            testCase.assertNumElements(axesHandle, 1);
            testCase.assertNumElements(fixedPlatform, 1);
            testCase.assertNumElements(actuator, 1);
            testCase.assertNumElements(fixedPlatformSurfaces, 3);
            testCase.verifyEqual(axesHandle.DataAspectRatio, [1, 1, 1]);
            radialDistance = hypot(actuator.XData, actuator.YData);
            testCase.verifyEqual( ...
                radialDistance, 5 * ones(size(radialDistance)), ...
                AbsTol=1e-10);
            testCase.verifyEqual( ...
                [min(arrayfun(@(surfaceHandle) ...
                    min(surfaceHandle.ZData, [], "all"), ...
                    fixedPlatformSurfaces)), ...
                 max(arrayfun(@(surfaceHandle) ...
                    max(surfaceHandle.ZData, [], "all"), ...
                    fixedPlatformSurfaces))], ...
                [-0.5, 0.5], AbsTol=1e-10);
        end

        function testAxesCoverFullyTiltedMovingPlatform(testCase)
            [fixedPoints, movingPoints] = ...
                CableJointAnimationTest.platformGeometry();
            height = 39;
            movingPlatformRadius = 80;
            halfThickness = 0.5;

            figureHandle = animate_cable_joint( ...
                0, [pi / 2; 0; height], fixedPoints, movingPoints, ...
                160, movingPlatformRadius, PlatformThickness=1, ...
                RealtimePlayback=false, Visible="off");

            axesHandle = findobj(figureHandle, Tag="CableJointAxes");
            theoreticalLimits = [ ...
                height - movingPlatformRadius - halfThickness, ...
                height + movingPlatformRadius + halfThickness];

            testCase.assertNumElements(axesHandle, 1);
            testCase.verifyLessThanOrEqual( ...
                axesHandle.ZLim(1), theoreticalLimits(1));
            testCase.verifyGreaterThanOrEqual( ...
                axesHandle.ZLim(2), theoreticalLimits(2));
        end

        function testRejectsInfinitePlatformRadius(testCase)
            [fixedPoints, movingPoints] = ...
                CableJointAnimationTest.platformGeometry();

            testCase.verifyError(@() animate_cable_joint( ...
                0, [0; 0; 39], fixedPoints, movingPoints, ...
                Inf, 80, RealtimePlayback=false, Visible="off"), ...
                "MATLAB:validators:mustBeFinite");
        end
    end

    methods (Static, Access=private)
        function closeNewFigures(existingFigures)
            currentFigures = findall(groot, Type="figure");
            newFigures = currentFigures(~ismember(currentFigures, existingFigures));
            delete(newFigures(isgraphics(newFigures)));
        end

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
