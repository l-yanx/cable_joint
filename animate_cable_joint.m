function figureHandle = animate_cable_joint( ...
        time, qTrajectory, fixedPoints, movingPoints, ...
        fixedPlatformRadius, movingPlatformRadius, options)
%ANIMATE_CABLE_JOINT Create the cable-joint animation scene.

arguments
    time (1, :) double
    qTrajectory (3, :) double
    fixedPoints (3, 3) double
    movingPoints (3, 3) double
    fixedPlatformRadius (1, 1) double {mustBePositive, mustBeFinite}
    movingPlatformRadius (1, 1) double {mustBePositive, mustBeFinite}
    options.PlatformThickness (1, 1) double ...
        {mustBePositive, mustBeFinite} = 1
    options.ActuatorRadius (1, 1) double ...
        {mustBePositive, mustBeFinite} = 5
    options.ExportVideo (1, 1) logical = false
    options.VideoFile (1, 1) string = "cable_joint_motion.mp4"
    options.VideoFrameRate (1, 1) double ...
        {mustBePositive, mustBeFinite} = 30
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
setappdata(figureHandle, "CableJointSceneComplete", false);
figureCleanup = onCleanup(@() delete_graphics(figureHandle));
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
    qTrajectory(3, :) - movingPlatformRadius ...
        - options.PlatformThickness / 2, ...
    qTrajectory(3, :) + movingPlatformRadius ...
        + options.PlatformThickness / 2];
heightRange = max(heightExtent) - min(heightExtent);
heightMargin = max(0.05 * heightRange, options.PlatformThickness);
zlim(axesHandle, ...
    [min(heightExtent) - heightMargin, max(heightExtent) + heightMargin]);

create_disc(axesHandle, fixedPlatformRadius, ...
    options.PlatformThickness, [0.72, 0.78, 0.86], "FixedPlatform");
movingDisc = create_disc(axesHandle, movingPlatformRadius, ...
    options.PlatformThickness, [0.92, 0.55, 0.24], "MovingPlatform");

[actuatorX, actuatorY, actuatorUnitZ] = ...
    cylinder(options.ActuatorRadius, 36);
actuatorHandle = surf(axesHandle, actuatorX, actuatorY, ...
    actuatorUnitZ * qTrajectory(3, 1), ...
    FaceColor=[0.45, 0.48, 0.52], ...
    EdgeColor="none", ...
    Tag="CenterActuator");

cableColor = [0.12, 0.32, 0.62];
cableHandles = gobjects(1, 3);
for cableIndex = 1:3
    cableHandles(cableIndex) = plot3(axesHandle, ...
        [fixedPoints(1, cableIndex), movingPoints(1, cableIndex)], ...
        [fixedPoints(2, cableIndex), movingPoints(2, cableIndex)], ...
        [fixedPoints(3, cableIndex), movingPoints(3, cableIndex)], ...
        Color=cableColor, LineWidth=2, ...
        Tag="Cable" + cableIndex);
end
scatter3(axesHandle, ...
    fixedPoints(1, :), fixedPoints(2, :), fixedPoints(3, :), ...
    48, [0.18, 0.18, 0.18], "filled", Tag="FixedNodes");
movingNodesHandle = scatter3(axesHandle, ...
    movingPoints(1, :), movingPoints(2, :), movingPoints(3, :), ...
    48, [0.86, 0.18, 0.12], "filled", Tag="MovingNodes");
poseStatusHandle = text(axesHandle, 0.02, 0.98, "", ...
    Units="normalized", VerticalAlignment="top", ...
    FontName="FixedWidth", Tag="PoseStatus");

for frameIndex = 1:sampleCount
    if ~isgraphics(figureHandle)
        break;
    end
    update_frame(frameIndex, time, qTrajectory, movingPoints, ...
        movingDisc, movingNodesHandle, cableHandles, ...
        actuatorHandle, actuatorUnitZ, poseStatusHandle);
    drawnow;
    if ~isgraphics(figureHandle)
        break;
    end
end

if ~isgraphics(figureHandle)
    clear figureCleanup;
    return;
end
setappdata(figureHandle, "CableJointSceneComplete", true);
clear figureCleanup;
end

function disc = create_disc( ...
        axesHandle, radius, thickness, faceColor, tag)
%CREATE_DISC Draw a closed cylindrical platform from three surfaces.

disc.Group = hggroup(axesHandle, Tag=tag);
theta = linspace(0, 2 * pi, 37);
radialCoordinates = [zeros(size(theta)); radius * ones(size(theta))];
xCoordinates = radialCoordinates .* cos(theta);
yCoordinates = radialCoordinates .* sin(theta);
topZ = (thickness / 2) * ones(size(xCoordinates));
bottomZ = -topZ;

disc.Top = surf(xCoordinates, yCoordinates, topZ, Parent=disc.Group, ...
    FaceColor=faceColor, EdgeColor="none", Tag="Top");
disc.Bottom = surf(xCoordinates, yCoordinates, bottomZ, Parent=disc.Group, ...
    FaceColor=faceColor, EdgeColor="none", Tag="Bottom");

sideX = radius * [cos(theta); cos(theta)];
sideY = radius * [sin(theta); sin(theta)];
sideZ = repmat([-thickness / 2; thickness / 2], 1, numel(theta));
disc.Side = surf(sideX, sideY, sideZ, Parent=disc.Group, ...
    FaceColor=0.85 * faceColor, EdgeColor="none", Tag="Side");
disc.Handles = [disc.Top, disc.Bottom, disc.Side];
disc.LocalPoints = { ...
    [xCoordinates(:)'; yCoordinates(:)'; topZ(:)'], ...
    [xCoordinates(:)'; yCoordinates(:)'; bottomZ(:)'], ...
    [sideX(:)'; sideY(:)'; sideZ(:)']};
disc.Sizes = {size(xCoordinates), size(xCoordinates), size(sideX)};
disc.Radius = radius;
disc.Thickness = thickness;
end

function update_frame(frameIndex, time, qTrajectory, movingPoints, ...
        movingDisc, movingNodesHandle, cableHandles, ...
        actuatorHandle, actuatorUnitZ, poseStatusHandle)
%UPDATE_FRAME Update all pose-dependent graphics for one trajectory sample.

q = qTrajectory(:, frameIndex);
rotation = rpy_rotation(q(1), q(2), 0);
position = [0; 0; q(3)];
movingNodes = transform_points(movingPoints, position, rotation);

update_moving_disc(movingDisc, position, rotation);
set(movingNodesHandle, ...
    XData=movingNodes(1, :), ...
    YData=movingNodes(2, :), ...
    ZData=movingNodes(3, :));
for cableIndex = 1:3
    set(cableHandles(cableIndex), ...
        XData=[cableHandles(cableIndex).XData(1), ...
            movingNodes(1, cableIndex)], ...
        YData=[cableHandles(cableIndex).YData(1), ...
            movingNodes(2, cableIndex)], ...
        ZData=[cableHandles(cableIndex).ZData(1), ...
            movingNodes(3, cableIndex)]);
end
set(actuatorHandle, ZData=actuatorUnitZ * q(3));
set(poseStatusHandle, String=sprintf( ...
    "t = %.3f s | alpha = %.3f deg | beta = %.3f deg | h = %.3f mm", ...
    time(frameIndex), rad2deg(q(1)), rad2deg(q(2)), q(3)));
end

function update_moving_disc(disc, position, rotation)
%UPDATE_MOVING_DISC Apply a rigid transform to all moving-platform surfaces.

for surfaceIndex = 1:numel(disc.Handles)
    worldPoints = transform_points( ...
        disc.LocalPoints{surfaceIndex}, position, rotation);
    update_surface(disc.Handles(surfaceIndex), worldPoints, ...
        disc.Sizes{surfaceIndex});
end
end

function update_surface(surfaceHandle, worldPoints, surfaceSize)
%UPDATE_SURFACE Replace one surface mesh with transformed coordinates.

set(surfaceHandle, ...
    XData=reshape(worldPoints(1, :), surfaceSize), ...
    YData=reshape(worldPoints(2, :), surfaceSize), ...
    ZData=reshape(worldPoints(3, :), surfaceSize));
end

function worldPoints = transform_points(localPoints, position, rotation)
%TRANSFORM_POINTS Transform local columns into world-coordinate columns.

worldPoints = position + rotation * localPoints;
end

function delete_graphics(graphicsHandle)
%DELETE_GRAPHICS Delete a graphics object when it is still valid.

if ~isgraphics(graphicsHandle)
    return;
end
if getappdata(graphicsHandle, "CableJointSceneComplete")
    rmappdata(graphicsHandle, "CableJointSceneComplete");
else
    delete(graphicsHandle);
end
end
