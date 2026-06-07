function figureHandle = animate_cable_joint( ...
        time, qTrajectory, fixedPoints, movingPoints, ...
        fixedPlatformRadius, movingPlatformRadius, options)
%ANIMATE_CABLE_JOINT Create the cable-joint animation scene.

arguments
    time (1, :) double
    qTrajectory (3, :) double
    fixedPoints (3, 3) double
    movingPoints (3, 3) double
    fixedPlatformRadius (1, 1) double {mustBePositive}
    movingPlatformRadius (1, 1) double {mustBePositive}
    options.PlatformThickness (1, 1) double {mustBePositive} = 1
    options.ActuatorRadius (1, 1) double {mustBePositive} = 5
    options.ExportVideo (1, 1) logical = false
    options.VideoFile (1, 1) string = "cable_joint_motion.mp4"
    options.VideoFrameRate (1, 1) double {mustBePositive} = 30
    options.RealtimePlayback (1, 1) logical = true
    options.Visible (1, 1) matlab.lang.OnOffSwitchState = "on"
end

sampleCount = numel(time);
if sampleCount == 0
    error("CableJointAnimation:InvalidTime", ...
        "time must contain at least one sample.");
end
if size(qTrajectory, 2) ~= sampleCount
    error("CableJointAnimation:TrajectorySize", ...
        "qTrajectory must contain one column for each time sample.");
end
if any(~isfinite(time)) || any(diff(time) <= 0)
    error("CableJointAnimation:InvalidTime", ...
        "time must be finite and strictly increasing.");
end
if any(~isfinite(qTrajectory), "all") || ...
        any(~isfinite(fixedPoints), "all") || ...
        any(~isfinite(movingPoints), "all")
    error("CableJointAnimation:NonfiniteInput", ...
        "Trajectory and geometry inputs must be finite.");
end
if options.ExportVideo && strlength(options.VideoFile) == 0
    error("CableJointAnimation:VideoFile", ...
        "VideoFile must be nonempty when ExportVideo is enabled.");
end

figureHandle = figure( ...
    Name="绳驱三自由度关节动画", ...
    Color="white", ...
    Position=[150, 100, 900, 760], ...
    Visible=options.Visible);
axesHandle = axes(figureHandle, Tag="CableJointAxes");
hold(axesHandle, "on");
grid(axesHandle, "on");
axis(axesHandle, "equal");
xlabel(axesHandle, "X / mm");
ylabel(axesHandle, "Y / mm");
zlabel(axesHandle, "Z / mm");
title(axesHandle, "绳驱三自由度关节运动");
view(axesHandle, 35, 24);

radialLimit = 1.15 * max(fixedPlatformRadius, movingPlatformRadius);
xlim(axesHandle, [-radialLimit, radialLimit]);
ylim(axesHandle, [-radialLimit, radialLimit]);
heightExtent = [ ...
    -options.PlatformThickness / 2, ...
    qTrajectory(3, :) - options.PlatformThickness / 2, ...
    qTrajectory(3, :) + options.PlatformThickness / 2];
heightRange = max(heightExtent) - min(heightExtent);
heightMargin = max(0.05 * heightRange, options.PlatformThickness);
zlim(axesHandle, ...
    [min(heightExtent) - heightMargin, max(heightExtent) + heightMargin]);

create_disc(axesHandle, fixedPlatformRadius, ...
    options.PlatformThickness, [0.72, 0.78, 0.86]);

[actuatorX, actuatorY, actuatorZ] = cylinder(options.ActuatorRadius, 36);
actuatorZ = actuatorZ * qTrajectory(3, 1);
surf(axesHandle, actuatorX, actuatorY, actuatorZ, ...
    FaceColor=[0.45, 0.48, 0.52], ...
    EdgeColor="none", ...
    Tag="CenterActuator");
end

function groupHandle = create_disc( ...
        axesHandle, radius, thickness, faceColor)
%CREATE_DISC Draw a closed cylindrical platform from three surfaces.

groupHandle = hggroup(axesHandle, Tag="FixedPlatform");
theta = linspace(0, 2 * pi, 37);
radialCoordinates = [zeros(size(theta)); radius * ones(size(theta))];
xCoordinates = radialCoordinates .* cos(theta);
yCoordinates = radialCoordinates .* sin(theta);
topZ = (thickness / 2) * ones(size(xCoordinates));
bottomZ = -topZ;

surf(xCoordinates, yCoordinates, topZ, Parent=groupHandle, ...
    FaceColor=faceColor, EdgeColor="none", Tag="Top");
surf(xCoordinates, yCoordinates, bottomZ, Parent=groupHandle, ...
    FaceColor=faceColor, EdgeColor="none", Tag="Bottom");

sideX = radius * [cos(theta); cos(theta)];
sideY = radius * [sin(theta); sin(theta)];
sideZ = repmat([-thickness / 2; thickness / 2], 1, numel(theta));
surf(sideX, sideY, sideZ, Parent=groupHandle, ...
    FaceColor=0.85 * faceColor, EdgeColor="none", Tag="Side");
end
