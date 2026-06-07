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
    Name="", ...
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

videoWriter = [];
temporaryVideoFile = "";
useFfmpegFallback = false;
if options.ExportVideo
    [videoWriter, temporaryVideoFile, useFfmpegFallback] = ...
        create_video_writer(options.VideoFile, options.VideoFrameRate);
    videoCleanup = onCleanup(@() close_video_writer(videoWriter));
    temporaryFileCleanup = onCleanup( ...
        @() delete_file_if_present(temporaryVideoFile));
    frameIndices = select_video_frames(time, options.VideoFrameRate);
else
    frameIndices = 1:sampleCount;
end

playbackClock = tic;
playbackStartTime = time(1);
figureHandle.Name = "绳驱三自由度关节动画";
for frameIndex = frameIndices
    if ~isgraphics(figureHandle)
        break;
    end
    update_frame(frameIndex, time, qTrajectory, fixedPoints, movingPoints, ...
        movingDisc, movingNodesHandle, cableHandles, ...
        actuatorHandle, actuatorUnitZ, poseStatusHandle);
    if options.ExportVideo
        drawnow;
    else
        drawnow limitrate;
    end
    if ~isgraphics(figureHandle)
        break;
    end
    if options.ExportVideo
        writeVideo(videoWriter, getframe(figureHandle));
    elseif options.RealtimePlayback
        targetElapsed = time(frameIndex) - playbackStartTime;
        remainingTime = targetElapsed - toc(playbackClock);
        if remainingTime > 0
            pause(remainingTime);
        end
        if ~isgraphics(figureHandle)
            break;
        end
    end
end

if options.ExportVideo
    clear videoCleanup;
    if useFfmpegFallback
        encode_mp4_with_ffmpeg(temporaryVideoFile, options.VideoFile);
    end
    clear temporaryFileCleanup;
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

function update_frame(frameIndex, time, qTrajectory, ...
        fixedPoints, movingPoints, movingDisc, movingNodesHandle, cableHandles, ...
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
        XData=[fixedPoints(1, cableIndex), movingNodes(1, cableIndex)], ...
        YData=[fixedPoints(2, cableIndex), movingNodes(2, cableIndex)], ...
        ZData=[fixedPoints(3, cableIndex), movingNodes(3, cableIndex)]);
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

function frameIndices = select_video_frames(time, frameRate)
%SELECT_VIDEO_FRAMES Map fixed-rate output times to nearest input samples.

duration = time(end) - time(1);
outputFrameCount = max(2, round(duration * frameRate));
outputTimes = linspace(time(1), time(end), outputFrameCount);
frameIndices = zeros(size(outputTimes));
for outputIndex = 1:numel(outputTimes)
    [~, frameIndices(outputIndex)] = ...
        min(abs(time - outputTimes(outputIndex)));
end
frameIndices(1) = 1;
frameIndices(end) = numel(time);
end

function [videoWriter, temporaryVideoFile, useFfmpegFallback] = ...
        create_video_writer(videoFile, frameRate)
%CREATE_VIDEO_WRITER Prefer native MP4 and otherwise prepare an AVI fallback.

profiles = VideoWriter.getProfiles;
profileNames = string({profiles.Name});
useFfmpegFallback = ~any(profileNames == "MPEG-4");
temporaryVideoFile = "";

if useFfmpegFallback
    [ffmpegStatus, ~] = system("command -v ffmpeg >/dev/null 2>&1");
    if ffmpegStatus ~= 0
        error("CableJointAnimation:Mp4EncoderUnavailable", ...
            ["MATLAB does not provide the MPEG-4 VideoWriter profile " ...
             "and ffmpeg is not available on PATH."]);
    end
    temporaryVideoFile = string(tempname) + ".avi";
    videoWriter = VideoWriter(temporaryVideoFile, "Motion JPEG AVI");
else
    videoWriter = VideoWriter(videoFile, "MPEG-4");
end
videoWriter.FrameRate = frameRate;
open(videoWriter);
end

function close_video_writer(videoWriter)
%CLOSE_VIDEO_WRITER Finalize a VideoWriter during normal or error cleanup.

if isempty(videoWriter)
    return;
end
try
    close(videoWriter);
catch exception
    if ~strcmp(exception.identifier, "MATLAB:audiovideo:VideoWriter:notOpen")
        rethrow(exception);
    end
end
end

function encode_mp4_with_ffmpeg(inputFile, outputFile)
%ENCODE_MP4_WITH_FFMPEG Transcode the fallback AVI to broadly compatible MP4.

command = "ffmpeg -y -loglevel error -i " + shell_quote(inputFile) + ...
    " -c:v libx264 -pix_fmt yuv420p " + shell_quote(outputFile);
[status, commandOutput] = system(command);
if status ~= 0 || ~isfile(outputFile)
    delete_file_if_present(outputFile);
    error("CableJointAnimation:Mp4EncodingFailed", ...
        "ffmpeg failed to encode MP4: %s", strtrim(commandOutput));
end
end

function quotedPath = shell_quote(filePath)
%SHELL_QUOTE Quote one path for a POSIX shell command.

quotedPath = "'" + replace(string(filePath), "'", "'""'""'") + "'";
end

function delete_file_if_present(filePath)
%DELETE_FILE_IF_PRESENT Remove a temporary or incomplete video file.

if strlength(filePath) > 0 && isfile(filePath)
    delete(filePath);
end
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
