# Cable Joint Animation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a real-time MATLAB animation of the three-DOF cable-driven joint, with optional MP4 export, while leaving the existing kinematics and result plotting architecture unchanged.

**Architecture:** Put all rendering, timing, validation, and video-export behavior in one new public function, `animate_cable_joint.m`, with focused local helper functions. Add a separate animation test class. Modify `main_run.m` only to define visualization options and call the new function; do not modify existing computational functions or `plot_results.m`.

**Tech Stack:** MATLAB graphics (`figure`, `surf`, `plot3`, `scatter3`), `VideoWriter`, optional `ffmpeg` MPEG-4 Part 2 fallback, `matlab.unittest`.

**Repository note:** `/home/lyx/cable joint` is not currently a Git worktree. The normal per-task commit steps are replaced by explicit verification checkpoints. Do not initialize Git as part of this feature.

---

## File Map

- Create `animate_cable_joint.m`: public animation entry point plus private local helpers for validation, mesh generation, coordinate transforms, timing, and video frame selection.
- Create `tests/CableJointAnimationTest.m`: isolated tests for geometry, validation, playback bypass, and export behavior.
- Modify `main_run.m`: add four user-facing visualization settings and one animation call.
- Do not modify `plot_results.m`.
- Do not modify `calc_cable_length.m`, `calc_cable_jacobian.m`, `rpy_rotation.m`, `generate_pose_trajectory.m`, or `convert_to_actuator_cmd.m`.
- Do not modify `tests/CableJointKinematicsTest.m`.

## Public Interface

Implement:

```matlab
function figureHandle = animate_cable_joint( ...
        time, qTrajectory, fixedPoints, movingPoints, ...
        fixedPlatformRadius, movingPlatformRadius, options)
```

Use an `arguments` block with these options:

```matlab
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
```

`RealtimePlayback` and `Visible` are testability controls. `main_run.m` should use their defaults and expose only the four user-facing settings already approved.

Tag graphics objects for deterministic tests:

```text
CableJointAxes
FixedPlatform
MovingPlatform
CenterActuator
Cable1, Cable2, Cable3
FixedNodes
MovingNodes
PoseStatus
```

---

### Task 1: Add Failing Validation and Static Geometry Tests

**Files:**
- Create: `tests/CableJointAnimationTest.m`
- Test: `tests/CableJointAnimationTest.m`

- [ ] **Step 1: Create the animation test class and common geometry helper**

Create this test skeleton:

```matlab
classdef CableJointAnimationTest < matlab.unittest.TestCase
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
```

- [ ] **Step 2: Run only the new tests and verify the expected failure**

Run:

```matlab
results = runtests("tests/CableJointAnimationTest.m");
disp(results);
```

Expected: tests fail because `animate_cable_joint` is undefined.

- [ ] **Step 3: Verification checkpoint**

Confirm no existing source file changed:

```bash
find . -maxdepth 2 -type f -name '*.m' -printf '%p\n' | sort
```

Expected new file only: `tests/CableJointAnimationTest.m`.

---

### Task 2: Implement Input Validation and Initial Figure

**Files:**
- Create: `animate_cable_joint.m`
- Test: `tests/CableJointAnimationTest.m`

- [ ] **Step 1: Add the public function, options, and semantic validation**

Start `animate_cable_joint.m` with the public signature from the Public Interface section. Immediately after the `arguments` block, add:

```matlab
    sampleCount = numel(time);
    if size(qTrajectory, 2) ~= sampleCount
        error("CableJointAnimation:TrajectorySize", ...
            "qTrajectory must contain one column for each time sample.");
    end
    if isempty(time) || any(~isfinite(time)) || ...
            any(diff(time) <= 0)
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
```

Handle the single-frame case explicitly because strict monotonicity applies only when multiple samples exist:

```matlab
    if sampleCount == 0
        error("CableJointAnimation:InvalidTime", ...
            "time must contain at least one sample.");
    end
```

Place the empty check before evaluating trajectory length and monotonicity.

- [ ] **Step 2: Create the figure and fixed axes**

Create the window and axes:

```matlab
    figureHandle = figure( ...
        Name="绳驱三自由度关节动画", ...
        Color="white", ...
        Position=[150, 100, 900, 760], ...
        Visible=options.Visible);
    axesHandle = axes(figureHandle, Tag="CableJointAxes");
    hold(axesHandle, "on");
    grid(axesHandle, "on");
    axis(axesHandle, "equal");
    xlabel(axesHandle, "x / mm");
    ylabel(axesHandle, "y / mm");
    zlabel(axesHandle, "z / mm");
    title(axesHandle, "绳驱三自由度关节运动");
    view(axesHandle, 35, 24);
```

Compute fixed limits once:

```matlab
    radialLimit = 1.15 * max(fixedPlatformRadius, movingPlatformRadius);
    halfThickness = options.PlatformThickness / 2;
    zMinimum = min(-halfThickness, min(qTrajectory(3, :)) - ...
        movingPlatformRadius - halfThickness);
    zMaximum = max(halfThickness, max(qTrajectory(3, :)) + ...
        movingPlatformRadius + halfThickness);
    zMargin = 0.05 * max(zMaximum - zMinimum, 1);

    xlim(axesHandle, radialLimit * [-1, 1]);
    ylim(axesHandle, radialLimit * [-1, 1]);
    zlim(axesHandle, [zMinimum - zMargin, zMaximum + zMargin]);
    axesHandle.XLimMode = "manual";
    axesHandle.YLimMode = "manual";
    axesHandle.ZLimMode = "manual";
```

- [ ] **Step 3: Add a reusable three-surface disc helper**

Add local helpers:

```matlab
function mesh = create_disc(parent, radius, thickness, color, tag)
    segmentCount = 72;
    theta = linspace(0, 2 * pi, segmentCount + 1);
    [thetaGrid, radiusGrid] = meshgrid(theta, [0, radius]);
    xFace = radiusGrid .* cos(thetaGrid);
    yFace = radiusGrid .* sin(thetaGrid);
    zTop = (thickness / 2) * ones(size(xFace));
    zBottom = -zTop;
    xSide = radius * [cos(theta); cos(theta)];
    ySide = radius * [sin(theta); sin(theta)];
    zSide = (thickness / 2) * [-ones(size(theta)); ones(size(theta))];

    group = hggroup(Parent=parent, Tag=tag);
    mesh.Top = surface(Parent=group, XData=xFace, YData=yFace, ZData=zTop, ...
        FaceColor=color, EdgeColor="none", FaceAlpha=0.85);
    mesh.Bottom = surface(Parent=group, ...
        XData=xFace, YData=yFace, ZData=zBottom, ...
        FaceColor=color, EdgeColor="none", FaceAlpha=0.85);
    mesh.Side = surface(Parent=group, ...
        XData=xSide, YData=ySide, ZData=zSide, ...
        FaceColor=color, EdgeColor="none", FaceAlpha=0.85);
    mesh.LocalTop = [xFace(:).'; yFace(:).'; zTop(:).'];
    mesh.LocalBottom = [xFace(:).'; yFace(:).'; zBottom(:).'];
    mesh.LocalSide = [xSide(:).'; ySide(:).'; zSide(:).'];
    mesh.TopSize = size(xFace);
    mesh.BottomSize = size(xFace);
    mesh.SideSize = size(xSide);
end
```

Create the fixed platform with:

```matlab
    create_disc(axesHandle, fixedPlatformRadius, ...
        options.PlatformThickness, [0.55, 0.62, 0.72], "FixedPlatform");
```

- [ ] **Step 4: Render the fixed platform and center actuator**

Create the actuator once with:

```matlab
    [actuatorXUnit, actuatorYUnit, actuatorZUnit] = cylinder( ...
        options.ActuatorRadius, 36);
    actuatorHandle = surf(axesHandle, ...
        actuatorXUnit, actuatorYUnit, ...
        actuatorZUnit * qTrajectory(3, 1), ...
        FaceColor=[0.35, 0.55, 0.78], ...
        EdgeColor="none", ...
        Tag="CenterActuator");
```

- [ ] **Step 5: Run the two tests**

Run:

```matlab
results = runtests("tests/CableJointAnimationTest.m");
assertSuccess(results);
```

Expected: both validation and static geometry tests pass.

---

### Task 3: Add Moving Platform, Cables, Nodes, and Frame Updates

**Files:**
- Modify: `animate_cable_joint.m`
- Modify: `tests/CableJointAnimationTest.m`

- [ ] **Step 1: Add failing tests for final-frame transforms and cable endpoints**

Add:

```matlab
function testFinalFrameMatchesPoseAndCableEndpoints(testCase)
    [fixedPoints, movingPoints] = ...
        CableJointAnimationTest.platformGeometry();
    alpha = deg2rad(4);
    beta = deg2rad(-3);
    height = 45;

    figureHandle = animate_cable_joint( ...
        [0, 0.01], [zeros(2, 1), [alpha; beta]; 39, height], ...
        fixedPoints, movingPoints, 160, 80, ...
        RealtimePlayback=false, Visible="off");

    expectedMovingNodes = [0; 0; height] + ...
        rpy_rotation(alpha, beta, 0) * movingPoints;
    movingNodes = findobj(figureHandle, Tag="MovingNodes");

    testCase.verifyEqual( ...
        [movingNodes.XData; movingNodes.YData; movingNodes.ZData], ...
        expectedMovingNodes, AbsTol=1e-10);

    for cableIndex = 1:3
        cable = findobj(figureHandle, ...
            Tag="Cable" + string(cableIndex));
        testCase.verifyEqual( ...
            [cable.XData(1); cable.YData(1); cable.ZData(1)], ...
            fixedPoints(:, cableIndex), AbsTol=1e-10);
        testCase.verifyEqual( ...
            [cable.XData(2); cable.YData(2); cable.ZData(2)], ...
            expectedMovingNodes(:, cableIndex), AbsTol=1e-10);
    end
end
```

- [ ] **Step 2: Run the new test and verify it fails**

Run:

```matlab
results = runtests( ...
    "tests/CableJointAnimationTest.m", ...
    ProcedureName="testFinalFrameMatchesPoseAndCableEndpoints");
disp(results);
```

Expected: FAIL because moving platform, nodes, and cable objects do not exist.

- [ ] **Step 3: Store moving-platform local mesh and create its graphics**

Create the moving disc as three surface handles under an `hggroup` tagged `MovingPlatform`:

```matlab
    movingMesh = create_disc(axesHandle, movingPlatformRadius, ...
        options.PlatformThickness, [0.90, 0.62, 0.25], "MovingPlatform");
```

Add:

```matlab
function worldCoordinates = transform_points(localCoordinates, position, rotation)
    worldCoordinates = position + rotation * localCoordinates;
end

function update_surface(surfaceHandle, worldCoordinates, surfaceSize)
    surfaceHandle.XData = reshape(worldCoordinates(1, :), surfaceSize);
    surfaceHandle.YData = reshape(worldCoordinates(2, :), surfaceSize);
    surfaceHandle.ZData = reshape(worldCoordinates(3, :), surfaceSize);
end

function update_moving_disc(mesh, position, rotation)
    topWorld = transform_points(mesh.LocalTop, position, rotation);
    bottomWorld = transform_points(mesh.LocalBottom, position, rotation);
    sideWorld = transform_points(mesh.LocalSide, position, rotation);

    update_surface(mesh.Top, topWorld, mesh.TopSize);
    update_surface(mesh.Bottom, bottomWorld, mesh.BottomSize);
    update_surface(mesh.Side, sideWorld, mesh.SideSize);
end
```

- [ ] **Step 4: Create cables, nodes, and status text once**

Use one cable color for all three cables:

```matlab
    cableColor = [0.10, 0.10, 0.12];
```

Create three `plot3` handles tagged `Cable1` through `Cable3`, all with `LineWidth=2`.

Create fixed and moving node handles:

```matlab
    fixedNodeHandle = scatter3(axesHandle, ...
        fixedPoints(1, :), fixedPoints(2, :), fixedPoints(3, :), ...
        45, [0.85, 0.20, 0.15], "filled", Tag="FixedNodes");
    movingNodeHandle = scatter3(axesHandle, ...
        nan(1, 3), nan(1, 3), nan(1, 3), ...
        45, [0.85, 0.20, 0.15], "filled", Tag="MovingNodes");
```

Create status text:

```matlab
    statusHandle = text(axesHandle, 0.02, 0.98, "", ...
        Units="normalized", ...
        VerticalAlignment="top", ...
        FontName="Consolas", ...
        BackgroundColor="white", ...
        Margin=5, ...
        Tag="PoseStatus");
```

- [ ] **Step 5: Implement one frame-update helper**

Add:

```matlab
function update_frame(frameIndex, time, qTrajectory, movingPoints, ...
        movingMesh, movingNodeHandle, cableHandles, ...
        fixedPoints, actuatorHandle, actuatorZUnit, statusHandle)
    q = qTrajectory(:, frameIndex);
    rotation = rpy_rotation(q(1), q(2), 0);
    position = [0; 0; q(3)];

    update_moving_disc(movingMesh, position, rotation);

    movingNodes = position + rotation * movingPoints;
    movingNodeHandle.XData = movingNodes(1, :);
    movingNodeHandle.YData = movingNodes(2, :);
    movingNodeHandle.ZData = movingNodes(3, :);

    for cableIndex = 1:3
        cableHandles(cableIndex).XData = ...
            [fixedPoints(1, cableIndex), movingNodes(1, cableIndex)];
        cableHandles(cableIndex).YData = ...
            [fixedPoints(2, cableIndex), movingNodes(2, cableIndex)];
        cableHandles(cableIndex).ZData = ...
            [fixedPoints(3, cableIndex), movingNodes(3, cableIndex)];
    end

    actuatorHandle.ZData = actuatorZUnit * q(3);
    statusHandle.String = sprintf( ...
        "t = %.2f s\\nalpha = %.2f deg\\nbeta = %.2f deg\\nh = %.2f mm", ...
        time(frameIndex), rad2deg(q(1)), rad2deg(q(2)), q(3));
end
```

- [ ] **Step 6: Add the frame loop without real-time waiting**

First implement:

```matlab
    frameIndices = 1:sampleCount;
    for frameIndex = frameIndices
        if ~isgraphics(figureHandle)
            break;
        end
        update_frame(...);
        drawnow;
    end
```

Do not add timing or video logic yet.

- [ ] **Step 7: Run the animation test class**

Run:

```matlab
results = runtests("tests/CableJointAnimationTest.m");
assertSuccess(results);
```

Expected: all tests pass and the returned figure contains the final frame.

---

### Task 4: Add Real-Time Playback and MP4 Frame Sampling

**Files:**
- Modify: `animate_cable_joint.m`
- Modify: `tests/CableJointAnimationTest.m`

- [x] **Step 1: Add failing tests for export, real-time pacing, and safe window closure**

Add an export integration test:

```matlab
function testExportsMp4WhenEnabled(testCase)
    [fixedPoints, movingPoints] = ...
        CableJointAnimationTest.platformGeometry();
    videoFile = string(tempname) + ".mp4";
    testCase.addTeardown(@() CableJointAnimationTest.deleteIfPresent(videoFile));

    animate_cable_joint( ...
        0:0.01:0.1, ...
        [zeros(2, 11); linspace(39, 45, 11)], ...
        fixedPoints, movingPoints, 160, 80, ...
        ExportVideo=true, VideoFile=videoFile, VideoFrameRate=30, ...
        RealtimePlayback=false, Visible="on");

    testCase.verifyTrue(isfile(videoFile));
    testCase.verifyGreaterThan(dir(videoFile).bytes, 0);
end
```

Add to the private static methods:

```matlab
function deleteIfPresent(fileName)
    if isfile(fileName)
        delete(fileName);
    end
end
```

Also add:

```matlab
function testDoesNotCreateVideoWhenDisabled(testCase)
    [fixedPoints, movingPoints] = ...
        CableJointAnimationTest.platformGeometry();
    videoFile = string(tempname) + ".mp4";

    animate_cable_joint( ...
        0, [0; 0; 39], fixedPoints, movingPoints, 160, 80, ...
        ExportVideo=false, VideoFile=videoFile, ...
        RealtimePlayback=false, Visible="off");

    testCase.verifyFalse(isfile(videoFile));
end
```

- [x] **Step 2: Run the export tests and verify the enabled case fails**

Run:

```matlab
results = runtests( ...
    "tests/CableJointAnimationTest.m", ...
    ProcedureName=["testExportsMp4WhenEnabled", ...
                   "testDoesNotCreateVideoWhenDisabled"]);
disp(results);
```

Expected: disabled case passes; enabled case fails because no video is written.

- [x] **Step 3: Add deterministic video-frame selection**

Add:

```matlab
function frameIndices = select_video_frames(time, frameRate)
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
```

Do not remove duplicate indices. Repeating a nearest simulation frame is required when the simulation sampling rate is lower than the requested video frame rate; removing duplicates would shorten the encoded duration.

- [x] **Step 4: Add safe `VideoWriter` lifecycle**

Before the loop, prefer the native `"MPEG-4"` profile. If it is unavailable,
create a temporary `"Motion JPEG AVI"` writer and require `ffmpeg` on `PATH`.
Create the temporary-file `onCleanup` before calling `open(videoWriter)`, then
create the writer cleanup after a successful open. This guarantees cleanup when
opening the temporary AVI itself fails.

```matlab
    videoWriter = [];
    if options.ExportVideo
        videoWriter = VideoWriter(options.VideoFile, "MPEG-4");
        videoWriter.FrameRate = options.VideoFrameRate;
        open(videoWriter);
        videoCleanup = onCleanup(@() close_video_writer(videoWriter));
        frameIndices = select_video_frames(time, options.VideoFrameRate);
    else
        frameIndices = 1:sampleCount;
    end
```

After each rendered export frame:

```matlab
    if options.ExportVideo
        writeVideo(videoWriter, getframe(figureHandle));
    end
```

Add:

```matlab
function close_video_writer(videoWriter)
    if ~isempty(videoWriter)
        close(videoWriter);
    end
end
```

After the loop, explicitly clear the writer cleanup so the file is finalized.
For the fallback path, transcode with `-c:v mpeg4 -q:v 3 -pix_fmt yuv420p`,
then delete the temporary AVI. This format is readable by MATLAB
`VideoReader` on the target R2025a/Linux installation:

```matlab
    if options.ExportVideo
        clear videoCleanup;
    end
```

- [x] **Step 5: Add `1:1` real-time pacing for interactive playback**

Only pace frames when `RealtimePlayback=true` and `ExportVideo=false`:

```matlab
    playbackClock = tic;
    playbackStartTime = time(1);
```

Before updating each interactive frame:

```matlab
    if options.RealtimePlayback && ~options.ExportVideo
        targetElapsed = time(frameIndex) - playbackStartTime;
        remainingTime = targetElapsed - toc(playbackClock);
        if remainingTime > 0
            pause(remainingTime);
        end
    end
```

The loop must check `isgraphics(figureHandle)` before and after `drawnow` so closing the window exits cleanly.

- [x] **Step 6: Run all animation tests**

Run:

```matlab
results = runtests("tests/CableJointAnimationTest.m");
assertSuccess(results);
```

Expected: validation, geometry, endpoint, disabled export, and MP4 export tests all pass.

If the installed MATLAB lacks the `"MPEG-4"` profile, use the documented
Motion JPEG AVI plus `ffmpeg` fallback. Raise
`CableJointAnimation:Mp4EncoderUnavailable` when `ffmpeg` is missing and
`CableJointAnimation:Mp4EncodingFailed` when transcoding fails.

---

### Task 5: Integrate with `main_run.m` Minimally

**Files:**
- Modify: `main_run.m:9-15`
- Modify: `main_run.m:53-54`

- [ ] **Step 1: Add only the approved visualization parameters**

Immediately after `drumRadius = 15;`, add:

```matlab
platformThickness = 1;
actuatorRadius = 5;
exportVideo = false;
videoFile = "cable_joint_motion.mp4";
```

Do not move or rename the existing radius variables.

- [ ] **Step 2: Add one animation call before the existing result plotting call**

Immediately before `plot_results(...)`, add:

```matlab
animate_cable_joint( ...
    time, qTrajectory, fixedPoints, movingPoints, ...
    fixedPlatformRadius, movingPlatformRadius, ...
    PlatformThickness=platformThickness, ...
    ActuatorRadius=actuatorRadius, ...
    ExportVideo=exportVideo, ...
    VideoFile=videoFile);
```

Keep the existing `plot_results(...)` call byte-for-byte unchanged.

- [ ] **Step 3: Run MATLAB Code Analyzer on the changed and new files**

Run `checkcode` or the MATLAB code analyzer for:

```text
animate_cable_joint.m
main_run.m
tests/CableJointAnimationTest.m
```

Expected: no errors. Resolve warnings caused by the new code without refactoring unrelated existing code.

- [ ] **Step 4: Run the full automated suite**

Run:

```matlab
results = runtests("tests");
disp(results);
assertSuccess(results);
```

Expected: all existing kinematics tests and all new animation tests pass.

---

### Task 6: Manual Visual and Timing Verification

**Files:**
- Verify: `main_run.m`
- Verify: `animate_cable_joint.m`

- [ ] **Step 1: Run the default interactive simulation**

Run:

```matlab
main_run
```

Verify:

- a separate animation window appears;
- fixed platform remains stationary;
- moving platform follows `alpha`, `beta`, and `h`;
- all three same-color cables remain attached to corresponding nodes;
- the `5 mm` center cylinder stays on the fixed `z` axis;
- both platforms visibly retain `1 mm` thickness;
- camera and axis limits do not change during playback;
- elapsed playback is approximately the configured `2 s`;
- the original five result plots still appear and retain their existing layout.

- [ ] **Step 2: Verify optional MP4 export**

Temporarily set in `main_run.m`:

```matlab
exportVideo = true;
```

Run `main_run`, then verify:

```matlab
isfile("cable_joint_motion.mp4")
```

Expected: logical `1`, with a playable video whose duration is approximately the trajectory duration.

Restore:

```matlab
exportVideo = false;
```

- [ ] **Step 3: Confirm architecture preservation**

Check the final changed-file set. It should contain only:

```text
animate_cable_joint.m
main_run.m
tests/CableJointAnimationTest.m
docs/superpowers/specs/2026-06-07-cable-joint-animation-design.md
docs/superpowers/plans/2026-06-07-cable-joint-animation.md
```

The `.superpowers/brainstorm/` visual-companion artifacts are design-session files, not runtime dependencies.

- [ ] **Step 4: Record final verification**

Report:

- MATLAB version and renderer used;
- automated test count and pass/fail result;
- Code Analyzer result;
- observed interactive playback duration;
- generated MP4 path and size when export is tested;
- any renderer-specific limitations.
