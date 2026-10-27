%% mobRob_Lidar
% Demo zu lidar-basierter Odometrie, Kartierung und SLAM.

clear; clc; close all;
rng(7);
fastTestMode = false;

% Ausfuehrungsoptionen fuer die drei Abschnitte
runPart1 = false;
runPart2 = false;
runPart3 = true;

% Darstellungsoptionen fuer Occupancy-Maps in Teil 2 und Teil 3
% true: Waende schwarz, freie Bereiche weiss
% false: Standard-Grauskala (Waende weiss, freie Bereiche schwarz)
swapBlackWhitePart2 = true;
swapBlackWhitePart3 = true;

exportVideos = true;
videoFrameRate = 20;
videoSpeedFactor = 2.0; % Beispiel: 2.0 erzeugt ein 2x schnelleres Export-Video.
videoOutputFolder = pwd;
animationPlaybackFactor = 1;

effectiveVideoFrameRate = max(1, round(videoFrameRate * max(videoSpeedFactor, 0.1)));

if fastTestMode
    exportVideos = false;
    animationPlaybackFactor = 20;
end

if fastTestMode
    localizationBeamStride = 1;
    localizationRayStep = 0.02;
    enableRecoverySearch = false;
    slamPoseCorrectionInterval = 1;
    slamRenderInterval = 10;
    slamMapUpdateInterval = 3;
    slamRenderBeamStride = 3;

    part3LocalizationBeamStride = 2;
    part3LocalizationRayStep = 0.025;
    part3RefineEveryNSteps = 3;
    part3MapUpdateInterval = 4;
    part3RenderInterval = 4;
    part3RealtimePlayback = false;
else
    localizationBeamStride = 1;
    localizationRayStep = 0.02;
    enableRecoverySearch = false;
    slamPoseCorrectionInterval = 1;
    slamRenderInterval = 4;
    slamMapUpdateInterval = 2;
    slamRenderBeamStride = 2;

    part3LocalizationBeamStride = 2;
    part3LocalizationRayStep = 0.03;
    part3RefineEveryNSteps = 2;
    part3MapUpdateInterval = 1;
    part3RenderInterval = 1;
    part3RealtimePlayback = true;
end

%% Gemeinsame Szenenvorgaben
sampleTime = 0.04;

if fastTestMode
    scanAngles = linspace(-pi/2, pi/2, 91);
else
    scanAngles = linspace(-pi/4, pi/4, 180);
end

maxRange = 5.5;
rayStep = 0.01;
animationStride = 3;
beamsPerFrameForMap = 20;

trajectoryCornersYX = [-2.0, 9.0;
                      5.2, 9.0;
                      5.2, 1.2;
                     -2.0, 1.2;
                     -2.0, 5.0];
trajectoryWaypointsXY = [trajectoryCornersYX(:, 2), trajectoryCornersYX(:, 1)];
trajectorySpeed = 0.25;
cornerTurnDuration = 1.8;
startScanHoldSeconds = 10;
endScanHoldSeconds = 10;

scene = createDemoScene();

trajectoryDuration = computePolylineDuration(trajectoryWaypointsXY, trajectorySpeed) + cornerTurnDuration * max(size(trajectoryWaypointsXY, 1) - 2, 0);
timeVector = 0:sampleTime:(startScanHoldSeconds + trajectoryDuration + endScanHoldSeconds);
numSteps = numel(timeVector);

truePoses = zeros(3, numSteps);
for stepIdx = 1:numSteps
    motionClock = min(max(timeVector(stepIdx) - startScanHoldSeconds, 0), trajectoryDuration);
    truePoses(:, stepIdx) = polylinePoseWithCornerTurns(motionClock, trajectoryWaypointsXY, trajectorySpeed, cornerTurnDuration);
end

if any(isTrajectoryInCollision(scene, truePoses))
    error('Configured trajectory intersects occupied map cells. Adjust trajectory parameters.');
end
%% Abschnitt 1: Lidar-basierte Odometrie und Scan Matching
if runPart1
    scanIndexA = 131;
    scanIndexB = 133;

    poseA = truePoses(:, scanIndexA);
    poseB = truePoses(:, scanIndexB);

    scanA = simulateLidarScan(scene, poseA, scanAngles, maxRange, rayStep);
    scanB = simulateLidarScan(scene, poseB, scanAngles, maxRange, rayStep);

    odomRelativeGuess = estimateRelativeMotion(truePoses(:, scanIndexA), truePoses(:, scanIndexB));
    odomRelativeGuess(1:2) = odomRelativeGuess(1:2) + [0.15; -0.10];
    odomRelativeGuess(3) = odomRelativeGuess(3) + deg2rad(4);

    [~, scanBTransformedMatched] = matchScanPairs(scanB.pointsBody, scanA.pointsBody, odomRelativeGuess, 25);
    scanBTransformedInitial = transformPoints2D(scanB.pointsBody, poseToTransform(odomRelativeGuess));
    scanBTransformedMatchedWorld = transformPoints2D(scanBTransformedMatched, poseToTransform(poseA));

    figure('Name', 'mobRob Lidar - Abschnitt 1', 'Color', 'w');
    subplot(2,2,1);
    plotScanComparison(scanA.pointsBody, scanB.pointsBody, 'Scan 131', 'Scan 133');
    title('Raw scans in sensor coordinates');

    subplot(2,2,2);
    plotScanComparison(scanA.pointsBody, scanBTransformedInitial, 'Scan 131', 'Scan 133 (initial guess)');
    title('Scan 133 with odometry guess');

    subplot(2,2,3);
    plotScanComparison(scanA.pointsBody, scanBTransformedMatched, 'Scan 131', 'Scan 133 (transformed)');
    title('Scan matching result');

    subplot(2,2,4);
    plotSceneMap(scene, scanA, scanB, scanBTransformedMatchedWorld);
    title('Map view with aligned scans');
end

%% Abschnitt 2: Lidar-basierte Kartierung mit Animation
if runPart2
    probabilityGrid = 0.5 * ones(scene.rows, scene.cols);
    mapTrajectoryX = [];
    mapTrajectoryY = [];
    mapBeamCursor = 1;
    scanSweepVisibilityMask = false(numel(scanAngles), 1);

    mapFigure = figure('Name', 'mobRob Lidar - Abschnitt 2', 'Color', 'w');
    mapAxis = subplot(1,2,1);
    scanAxis = subplot(1,2,2);

    if exportVideos
        mapVideo = VideoWriter(fullfile(videoOutputFolder, 'mobRob_Lidar_map_building.mp4'), 'MPEG-4');
        mapVideo.FrameRate = effectiveVideoFrameRate;
        open(mapVideo);
    end

    mapImageHandle = imagesc(mapAxis, scene.yCenters, scene.xCenters, probabilityGrid);
    set(mapAxis, 'YDir', 'normal');
    axis(mapAxis, 'equal');
    xlim(mapAxis, scene.yLimits);
    ylim(mapAxis, scene.xLimits);
    set(mapAxis, 'XLimMode', 'manual', 'YLimMode', 'manual');
    if swapBlackWhitePart2
        colormap(mapAxis, flipud(gray));
    else
        colormap(mapAxis, gray);
    end
    clim(mapAxis, [0 1]);
    colorbar(mapAxis);
    xlabel(mapAxis, 'x [m]');
    ylabel(mapAxis, 'y [m]');
    title(mapAxis, 'Kartierung - Occupancy Grid Map');
    hold(mapAxis, 'on');
    robotPathHandle = plot(mapAxis, nan, nan, 'b-', 'LineWidth', 1.2);
    robotHandle = plot(mapAxis, nan, nan, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6);
    headingHandle = plot(mapAxis, nan, nan, 'r-', 'LineWidth', 1.2);
    mapFovHandle = patch(mapAxis, nan, nan, [1.0 0.85 0.15], 'FaceAlpha', 0.28, 'EdgeColor', [1.0 0.8 0.05], 'LineWidth', 1.2);
    laserHandle = plot(mapAxis, nan, nan, 'g-', 'LineWidth', 1.0);
    hold(mapAxis, 'off');

    axis(scanAxis, 'equal');
    xlim(scanAxis, scene.yLimits);
    ylim(scanAxis, scene.xLimits);
    set(scanAxis, 'XLimMode', 'manual', 'YLimMode', 'manual');
    grid(scanAxis, 'on');
    hold(scanAxis, 'on');
    scanCloudHandle = scatter(scanAxis, nan, nan, 10, 'filled');
    scanBeamHandle = plot(scanAxis, nan, nan, 'g-', 'LineWidth', 1.2);
    scanRobotHandle = plot(scanAxis, nan, nan, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6);
    scanHeadingHandle = plot(scanAxis, nan, nan, 'r-', 'LineWidth', 1.2);
    xlabel(scanAxis, 'x [m]');
    ylabel(scanAxis, 'y [m]');
    title(scanAxis, 'Aktueller Lidar Scan');
    hold(scanAxis, 'off');

    for stepIdx = 1:animationStride:numSteps
        currentPose = truePoses(:, stepIdx);
        currentScan = simulateLidarScan(scene, currentPose, scanAngles, maxRange, rayStep);

        beamCursorBefore = mapBeamCursor;
        beamIndices = mod((mapBeamCursor:mapBeamCursor + beamsPerFrameForMap - 1) - 1, numel(scanAngles)) + 1;
        sweepWrapped = (beamCursorBefore + beamsPerFrameForMap - 1) > numel(scanAngles);
        if sweepWrapped
            scanSweepVisibilityMask(:) = false;
        end
        scanSweepVisibilityMask(beamIndices) = true;

        integratedBeamScan = extractBeamBatch(currentScan, beamIndices);
        probabilityGrid = updateOccupancyGrid(probabilityGrid, scene, currentPose, integratedBeamScan, 0.08, 0.14);
        beamIndex = beamIndices(end);
        mapBeamCursor = beamIndex + 1;

        mapImageHandle.CData = probabilityGrid;

        mapTrajectoryX = [mapTrajectoryX, currentPose(2)]; %#ok<AGROW>
        mapTrajectoryY = [mapTrajectoryY, currentPose(1)]; %#ok<AGROW>
        set(robotPathHandle, 'XData', mapTrajectoryX, 'YData', mapTrajectoryY);
        set(robotHandle, 'XData', currentPose(2), 'YData', currentPose(1));

        headingLength = 0.45;
        headingX = currentPose(2) + [0, headingLength * sin(currentPose(3))];
        headingY = currentPose(1) + [0, headingLength * cos(currentPose(3))];
        set(headingHandle, 'XData', headingX, 'YData', headingY);

        [conePlotX, conePlotY] = buildFovConePlot(currentPose, scanAngles, maxRange, 40);
        set(mapFovHandle, 'XData', conePlotX, 'YData', conePlotY);
        uistack(mapFovHandle, 'top');
        uistack(robotHandle, 'top');
        uistack(headingHandle, 'top');

        beamX = [currentPose(2), currentScan.pointsWorld(beamIndex, 2)];
        beamY = [currentPose(1), currentScan.pointsWorld(beamIndex, 1)];
        set(laserHandle, 'XData', beamX, 'YData', beamY);

        visibleScanPoints = currentScan.pointsWorld(scanSweepVisibilityMask, :);
        set(scanCloudHandle, 'XData', visibleScanPoints(:, 2), 'YData', visibleScanPoints(:, 1));
        set(scanBeamHandle, 'XData', beamX, 'YData', beamY);
        set(scanRobotHandle, 'XData', currentPose(2), 'YData', currentPose(1));
        set(scanHeadingHandle, 'XData', headingX, 'YData', headingY);

        drawnow limitrate;
        if exportVideos
            if isgraphics(mapFigure)
                writeVideo(mapVideo, getframe(mapFigure));
            elseif isgraphics(mapAxis)
                writeVideo(mapVideo, getframe(mapAxis));
            end
        end
    end

    if exportVideos
        close(mapVideo);
    end
end

%% Abschnitt 3: Lidar-SLAM mit gemeinsamer Animation
if runPart3
    slamProbabilityGrid = 0.5 * ones(scene.rows, scene.cols);
    slamAnimationStride = 1;
    maxAllowedDeviation = 0.20;
    sceneDistanceMap = buildSceneDistanceMap(scene);

    capturedScans = cell(1, numSteps);
    for stepIdx = 1:numSteps
        capturedScans{stepIdx} = simulateLidarScan(scene, truePoses(:, stepIdx), scanAngles, maxRange, rayStep);
    end

    estimatedPoses = zeros(3, numSteps);
    estimatedPoses(:, 1) = truePoses(:, 1);
    matchedScanWorldCache = cell(1, numSteps);

    % EKF-SLAM Posezustand (2D) mit pseudo-absoluter Lidarmessung
    ekfCovariance = diag([0.015^2, 0.015^2, deg2rad(1.0)^2]);
    processNoiseBase = diag([0.006^2, 0.006^2, deg2rad(0.3)^2]);
    processNoiseVelocityGain = diag([0.014^2, 0.014^2, deg2rad(0.8)^2]);
    lidarMeasurementNoise = diag([0.060^2, 0.060^2, deg2rad(6.0)^2]);

    localizationScans = cell(1, numSteps);
    for stepIdx = 1:numSteps
        localizationScans{stepIdx} = downsampleScanForLocalization(capturedScans{stepIdx}, max(1, part3LocalizationBeamStride));
    end

    firstScan = localizationScans{1};
    matchedScanWorldCache{1} = transformPoints2D(firstScan.pointsBody, poseToTransform(estimatedPoses(:, 1)));

    for stepIdx = 2:numSteps
        scanCurr = localizationScans{stepIdx};

        previousIdx = stepIdx - 1;
        effectiveDt = timeVector(stepIdx) - timeVector(previousIdx);
        relativeIncrement = estimateRelativeMotion(truePoses(:, previousIdx), truePoses(:, stepIdx));
        trueCommand = commandFromPoseStep(truePoses(:, previousIdx), truePoses(:, stepIdx), effectiveDt);
        noisyRelativeIncrement = relativeIncrement;

        predictedPose = composePose(estimatedPoses(:, previousIdx), noisyRelativeIncrement);
        previousHeading = estimatedPoses(3, previousIdx);
        dxBody = noisyRelativeIncrement(1);
        dyBody = noisyRelativeIncrement(2);
        stateJacobian = [1, 0, -sin(previousHeading) * dxBody - cos(previousHeading) * dyBody;
             0, 1,  cos(previousHeading) * dxBody - sin(previousHeading) * dyBody;
                 0, 0, 1];
        linearIncrementNorm = norm(noisyRelativeIncrement(1:2));
        processNoise = processNoiseBase + linearIncrementNorm * processNoiseVelocityGain;
        predictedCovariance = stateJacobian * ekfCovariance * stateJacobian.' + processNoise;
        predictedCovariance = 0.5 * (predictedCovariance + predictedCovariance.');

        isCornerTurn = abs(trueCommand(1)) < 0.03 && abs(trueCommand(2)) > deg2rad(6);
        doRefineNow = isCornerTurn || mod(stepIdx - 1, part3RefineEveryNSteps) == 0;
        if doRefineNow
            [coarsePoseEstimate, ~, coarseResidualCost] = refinePoseByDistanceField( ...
                scanCurr, predictedPose, scene, sceneDistanceMap, ...
                0.08, deg2rad(6), 0.04, deg2rad(2.0));
            [refinedPoseEstimate, ~, refinedResidualCost] = refinePoseByDistanceField( ...
                scanCurr, coarsePoseEstimate, scene, sceneDistanceMap, ...
                0.02, deg2rad(2), 0.02, deg2rad(1.0));
            residualImprovement = coarseResidualCost - refinedResidualCost;
            isReliableRefinement = refinedResidualCost < 0.18 && residualImprovement > -0.02;

            if isReliableRefinement
                boundedMeasurementPose = applyBoundedIcpUpdate(predictedPose, refinedPoseEstimate(:), 0.015, deg2rad(1.5));
                [estimatedPoses(:, stepIdx), ekfCovariance] = ekfUpdatePoseMeasurement(predictedPose, predictedCovariance, boundedMeasurementPose, lidarMeasurementNoise);
            else
                estimatedPoses(:, stepIdx) = predictedPose;
                ekfCovariance = predictedCovariance;
            end
        else
            estimatedPoses(:, stepIdx) = predictedPose;
            ekfCovariance = predictedCovariance;
        end
        estimatedPoses(3, stepIdx) = wrapToPiLocal(estimatedPoses(3, stepIdx));

        reprojectedCurr = reprojectScanAtPose(scanCurr, estimatedPoses(:, stepIdx));
        matchedScanWorldCache{stepIdx} = reprojectedCurr.pointsWorld;
    end

    slamFigure = figure('Name', 'mobRob Lidar - Abschnitt 3', 'Color', 'w');
    slamMapAxis = subplot(1,2,1);
    slamScanAxis = subplot(1,2,2);

    if exportVideos
        slamVideo = VideoWriter(fullfile(videoOutputFolder, 'mobRob_Lidar_slam.mp4'), 'MPEG-4');
        slamVideo.FrameRate = effectiveVideoFrameRate;
        open(slamVideo);
    end

    slamMapImageHandle = imagesc(slamMapAxis, scene.yCenters, scene.xCenters, slamProbabilityGrid);
    set(slamMapAxis, 'YDir', 'normal');
    axis(slamMapAxis, 'equal');
    xlim(slamMapAxis, scene.yLimits);
    ylim(slamMapAxis, scene.xLimits);
    set(slamMapAxis, 'XLimMode', 'manual', 'YLimMode', 'manual');
    if swapBlackWhitePart3
        colormap(slamMapAxis, flipud(gray));
    else
        colormap(slamMapAxis, gray);
    end
    clim(slamMapAxis, [0 1]);
    colorbar(slamMapAxis);
    xlabel(slamMapAxis, 'x [m]');
    ylabel(slamMapAxis, 'y [m]');
    title(slamMapAxis, '2D-Lidar-SLAM');
    hold(slamMapAxis, 'on');
    slamTruePathHandle = plot(slamMapAxis, nan, nan, 'r--', 'LineWidth', 1.5);
    slamEstimatePathHandle = plot(slamMapAxis, nan, nan, 'b-', 'LineWidth', 1.8);
    slamTrueRobotHandle = plot(slamMapAxis, nan, nan, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 5);
    slamEstimateRobotHandle = plot(slamMapAxis, nan, nan, 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 5);
    slamFovHandle = patch(slamMapAxis, nan, nan, [1.0 0.85 0.15], 'FaceAlpha', 0.24, 'EdgeColor', [1.0 0.8 0.05], 'LineWidth', 1.0);
    slamLaserHandle = plot(slamMapAxis, nan, nan, 'g-', 'LineWidth', 1.0);
    hold(slamMapAxis, 'off');

    axis(slamScanAxis, 'equal');
    xlim(slamScanAxis, scene.yLimits);
    ylim(slamScanAxis, scene.xLimits);
    set(slamScanAxis, 'XLimMode', 'manual', 'YLimMode', 'manual');
    grid(slamScanAxis, 'on');
    hold(slamScanAxis, 'on');
    slamScanCloudHandle = scatter(slamScanAxis, nan, nan, 10, 'filled');
    slamMatchedCloudHandle = scatter(slamScanAxis, nan, nan, 10, 'filled');
    slamBeamHandle = plot(slamScanAxis, nan, nan, 'g-', 'LineWidth', 1.0);
    slamScanRobotHandle = plot(slamScanAxis, nan, nan, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6);
    slamScanHeadingHandle = plot(slamScanAxis, nan, nan, 'r-', 'LineWidth', 1.2);
    xlabel(slamScanAxis, 'x [m]');
    ylabel(slamScanAxis, 'y [m]');
    title(slamScanAxis, 'Scan Messung / Schätzung');
    hold(slamScanAxis, 'off');

    slamTrueHistory = [truePoses(2, 1), truePoses(1, 1)];
    slamEstimateHistory = [estimatedPoses(2, 1), estimatedPoses(1, 1)];
    slamBeamCursor = 1;
    lastSlamStepIdx = 1;

    rtClock = tic;
    for stepIdx = 1:slamAnimationStride:numSteps
        currentTruePose = truePoses(:, stepIdx);
        currentEstimatedPose = estimatedPoses(:, stepIdx);
        currentScan = capturedScans{stepIdx};
        reprojectedEstimatedScan = reprojectScanAtPose(currentScan, currentEstimatedPose);

        deviation = norm(currentEstimatedPose(1:2) - currentTruePose(1:2));
        if deviation > maxAllowedDeviation
            error('Simulation abgebrochen in Teil 3 bei t=%.2f s: Abweichung %.3f m > %.2f m', timeVector(stepIdx), deviation, maxAllowedDeviation);
        end

        doMapUpdateNow = mod(stepIdx - 1, part3MapUpdateInterval) == 0;
        if doMapUpdateNow
            slamBeamIndices = mod((slamBeamCursor:slamBeamCursor + beamsPerFrameForMap - 1) - 1, numel(scanAngles)) + 1;
            integratedSlamBeamScan = extractBeamBatch(reprojectedEstimatedScan, slamBeamIndices);
            slamProbabilityGrid = updateOccupancyGrid(slamProbabilityGrid, scene, currentEstimatedPose, integratedSlamBeamScan, 0.06, 0.12);
            slamBeamIndex = slamBeamIndices(end);
            slamBeamCursor = slamBeamIndex + 1;
        elseif stepIdx == 1
            slamBeamIndex = 1;
        end

        if ~isgraphics(slamMapImageHandle)
            break;
        end

        shouldRenderNow = mod(stepIdx - 1, part3RenderInterval) == 0 || stepIdx == numSteps;
        if shouldRenderNow
            slamMapImageHandle.CData = slamProbabilityGrid;
            slamTrueHistory = [slamTrueHistory; currentTruePose(2), currentTruePose(1)]; %#ok<AGROW>
            slamEstimateHistory = [slamEstimateHistory; currentEstimatedPose(2), currentEstimatedPose(1)]; %#ok<AGROW>

            set(slamTruePathHandle, 'XData', slamTrueHistory(:, 1), 'YData', slamTrueHistory(:, 2));
            set(slamEstimatePathHandle, 'XData', slamEstimateHistory(:, 1), 'YData', slamEstimateHistory(:, 2));
            set(slamTrueRobotHandle, 'XData', currentTruePose(2), 'YData', currentTruePose(1));
            set(slamEstimateRobotHandle, 'XData', currentEstimatedPose(2), 'YData', currentEstimatedPose(1));

            [slamConeX, slamConeY] = buildFovConePlot(currentEstimatedPose, scanAngles, maxRange, 40);
            set(slamFovHandle, 'XData', slamConeX, 'YData', slamConeY);
            uistack(slamFovHandle, 'top');
            uistack(slamTrueRobotHandle, 'top');
            uistack(slamEstimateRobotHandle, 'top');

            slamBeamX = [currentEstimatedPose(2), reprojectedEstimatedScan.pointsWorld(slamBeamIndex, 2)];
            slamBeamY = [currentEstimatedPose(1), reprojectedEstimatedScan.pointsWorld(slamBeamIndex, 1)];
            set(slamLaserHandle, 'XData', slamBeamX, 'YData', slamBeamY);

            renderBeamIndices = 1:slamRenderBeamStride:size(reprojectedEstimatedScan.pointsWorld, 1);
            set(slamScanCloudHandle, 'XData', reprojectedEstimatedScan.pointsWorld(renderBeamIndices, 2), 'YData', reprojectedEstimatedScan.pointsWorld(renderBeamIndices, 1));
            matchedScanWorld = matchedScanWorldCache{stepIdx};
            matchedRenderIndices = 1:slamRenderBeamStride:size(matchedScanWorld, 1);
            set(slamMatchedCloudHandle, 'XData', matchedScanWorld(matchedRenderIndices, 2), 'YData', matchedScanWorld(matchedRenderIndices, 1));
            set(slamBeamHandle, 'XData', slamBeamX, 'YData', slamBeamY);
            set(slamScanRobotHandle, 'XData', currentEstimatedPose(2), 'YData', currentEstimatedPose(1));

            scanHeadingLength = 0.45;
            scanHeadingX = currentEstimatedPose(2) + [0, scanHeadingLength * sin(currentEstimatedPose(3))];
            scanHeadingY = currentEstimatedPose(1) + [0, scanHeadingLength * cos(currentEstimatedPose(3))];
            set(slamScanHeadingHandle, 'XData', scanHeadingX, 'YData', scanHeadingY);

            lastSlamStepIdx = stepIdx;
            drawnow limitrate;

            if part3RealtimePlayback
                targetElapsed = timeVector(stepIdx) - timeVector(1);
                elapsed = toc(rtClock);
                if elapsed < targetElapsed
                    pause(targetElapsed - elapsed);
                end
            end

            if exportVideos
                if isgraphics(slamFigure)
                    writeVideo(slamVideo, getframe(slamFigure));
                elseif isgraphics(slamMapAxis)
                    writeVideo(slamVideo, getframe(slamMapAxis));
                end
            end
        end
    end

    if exportVideos
        close(slamVideo);
    end

    finalTruePose = truePoses(:, lastSlamStepIdx);
    finalEstimatePose = estimatedPoses(:, lastSlamStepIdx);
    finalPositionError = norm(finalEstimatePose(1:2) - finalTruePose(1:2));
    finalHeadingErrorDeg = rad2deg(abs(wrapToPiLocal(finalEstimatePose(3) - finalTruePose(3))));
    fprintf('Teil 3 Endfehler: Position = %.3f m, Heading = %.2f deg\n', finalPositionError, finalHeadingErrorDeg);
end

%% Lokale Funktionen
function scene = createDemoScene()
resolution = 20;
xLimits = [0, 10];
yLimits = [-4, 6];

numRows = round(diff(xLimits) * resolution);
numCols = round(diff(yLimits) * resolution);
occupiedMask = false(numRows, numCols);

occupiedMask = paintRectangleWorld(occupiedMask, xLimits, yLimits, [0.0, 10.0, -4.0, -3.82], resolution);
occupiedMask = paintRectangleWorld(occupiedMask, xLimits, yLimits, [0.0, 10.0, 5.82, 6.0], resolution);
occupiedMask = paintRectangleWorld(occupiedMask, xLimits, yLimits, [0.0, 0.18, -4.0, 6.0], resolution);
occupiedMask = paintRectangleWorld(occupiedMask, xLimits, yLimits, [9.82, 10.0, -4.0, 6.0], resolution);

occupiedMask = paintRectangleWorld(occupiedMask, xLimits, yLimits, [2.10, 5.70, -0.15, 0.15], resolution);
occupiedMask = paintRectangleWorld(occupiedMask, xLimits, yLimits, [5.55, 5.85, -0.15, 4.40], resolution);
occupiedMask = paintRectangleWorld(occupiedMask, xLimits, yLimits, [1.35, 2.10, 1.95, 2.25], resolution);
occupiedMask = paintRectangleWorld(occupiedMask, xLimits, yLimits, [2.10, 7.70, 4.35, 4.65], resolution);
occupiedMask = paintRectangleWorld(occupiedMask, xLimits, yLimits, [7.45, 7.75, 1.55, 4.65], resolution);

[xCenterGrid, yCenterGrid] = ndgrid(xLimits(1) + (0.5:1:numRows - 0.5) / resolution, ...
                                     yLimits(1) + (0.5:1:numCols - 0.5) / resolution);
occupiedPointsWorld = [xCenterGrid(occupiedMask), yCenterGrid(occupiedMask)];

scene = struct();
scene.resolution = resolution;
scene.xLimits = xLimits;
scene.yLimits = yLimits;
scene.rows = numRows;
scene.cols = numCols;
scene.occupiedMask = occupiedMask;
scene.occupiedPointsWorld = occupiedPointsWorld;
scene.xCenters = xLimits(1) + (0.5:1:numRows - 0.5) / resolution;
scene.yCenters = yLimits(1) + (0.5:1:numCols - 0.5) / resolution;
end

function occupancyMask = paintRectangleWorld(occupancyMask, xLimits, yLimits, rectangleWorld, resolution)
rowMin = max(1, floor((rectangleWorld(1) - xLimits(1)) * resolution) + 1);
rowMax = min(size(occupancyMask, 1), ceil((rectangleWorld(2) - xLimits(1)) * resolution));
colMin = max(1, floor((rectangleWorld(3) - yLimits(1)) * resolution) + 1);
colMax = min(size(occupancyMask, 2), ceil((rectangleWorld(4) - yLimits(1)) * resolution));

if rowMin <= rowMax && colMin <= colMax
    occupancyMask(rowMin:rowMax, colMin:colMax) = true;
end
end

function collisionMask = isTrajectoryInCollision(scene, poses)
rowIndices = floor((poses(1, :) - scene.xLimits(1)) * scene.resolution) + 1;
colIndices = floor((poses(2, :) - scene.yLimits(1)) * scene.resolution) + 1;

insideMap = rowIndices >= 1 & rowIndices <= scene.rows & colIndices >= 1 & colIndices <= scene.cols;
collisionMask = ~insideMap;

validRows = rowIndices(insideMap);
validCols = colIndices(insideMap);
linearIndices = sub2ind(size(scene.occupiedMask), validRows, validCols);
collisionMask(insideMap) = scene.occupiedMask(linearIndices);
end

function scanData = simulateLidarScan(scene, pose, scanAngles, maxRange, rayStep)
numBeams = numel(scanAngles);
pointsWorld = nan(numBeams, 2);
pointsBody = nan(numBeams, 2);
ranges = maxRange * ones(numBeams, 1);
hitMask = false(numBeams, 1);

for beamIdx = 1:numBeams
    beamAngle = pose(3) + scanAngles(beamIdx);
    distanceVector = 0:rayStep:maxRange;
    currentRange = maxRange;
    currentPoint = [pose(1) + maxRange * cos(beamAngle), pose(2) + maxRange * sin(beamAngle)];

    for sampleIdx = 1:numel(distanceVector)
        sampledDistance = distanceVector(sampleIdx);
        sampleX = pose(1) + sampledDistance * cos(beamAngle);
        sampleY = pose(2) + sampledDistance * sin(beamAngle);

        if sampleX < scene.xLimits(1) || sampleX > scene.xLimits(2) || sampleY < scene.yLimits(1) || sampleY > scene.yLimits(2)
            currentRange = sampledDistance;
            currentPoint = [sampleX, sampleY];
            break;
        end

        rowIndex = floor((sampleX - scene.xLimits(1)) * scene.resolution) + 1;
        colIndex = floor((sampleY - scene.yLimits(1)) * scene.resolution) + 1;

        if rowIndex >= 1 && rowIndex <= scene.rows && colIndex >= 1 && colIndex <= scene.cols && scene.occupiedMask(rowIndex, colIndex)
            currentRange = sampledDistance;
            currentPoint = [sampleX, sampleY];
            hitMask(beamIdx) = true;
            break;
        end
    end

    ranges(beamIdx) = currentRange;
    pointsWorld(beamIdx, :) = currentPoint;
    pointsBody(beamIdx, :) = [currentRange * cos(scanAngles(beamIdx)), currentRange * sin(scanAngles(beamIdx))];
end

scanData = struct();
scanData.ranges = ranges;
scanData.pointsWorld = pointsWorld;
scanData.pointsBody = pointsBody;
scanData.hitMask = hitMask;
scanData.pose = pose(:);
scanData.scanAngles = scanAngles(:);
end

function singleBeamScan = extractBeamScan(scanData, beamIndex)
singleBeamScan = struct();
singleBeamScan.ranges = scanData.ranges(beamIndex);
singleBeamScan.pointsWorld = scanData.pointsWorld(beamIndex, :);
singleBeamScan.pointsBody = scanData.pointsBody(beamIndex, :);
singleBeamScan.hitMask = scanData.hitMask(beamIndex);
singleBeamScan.pose = scanData.pose;
singleBeamScan.scanAngles = scanData.scanAngles(beamIndex);
end

function beamBatchScan = extractBeamBatch(scanData, beamIndices)
beamBatchScan = struct();
beamBatchScan.ranges = scanData.ranges(beamIndices);
beamBatchScan.pointsWorld = scanData.pointsWorld(beamIndices, :);
beamBatchScan.pointsBody = scanData.pointsBody(beamIndices, :);
beamBatchScan.hitMask = scanData.hitMask(beamIndices);
beamBatchScan.pose = scanData.pose;
beamBatchScan.scanAngles = scanData.scanAngles(beamIndices);
end

function reprojectedScan = reprojectScanAtPose(scanData, pose)
pose = pose(:);
angles = scanData.scanAngles(:);
ranges = scanData.ranges(:);

worldX = pose(1) + ranges .* cos(pose(3) + angles);
worldY = pose(2) + ranges .* sin(pose(3) + angles);

reprojectedScan = scanData;
reprojectedScan.pointsWorld = [worldX, worldY];
reprojectedScan.pose = pose;
end

function probabilityGrid = updateOccupancyGrid(probabilityGrid, scene, pose, scanData, freeGain, occupiedGain)
sensorRow = floor((pose(1) - scene.xLimits(1)) * scene.resolution) + 1;
sensorCol = floor((pose(2) - scene.yLimits(1)) * scene.resolution) + 1;

for beamIdx = 1:numel(scanData.ranges)
    endpoint = scanData.pointsWorld(beamIdx, :);
    endpointRow = floor((endpoint(1) - scene.xLimits(1)) * scene.resolution) + 1;
    endpointCol = floor((endpoint(2) - scene.yLimits(1)) * scene.resolution) + 1;
    [rowIndices, colIndices] = lineCells(sensorRow, sensorCol, endpointRow, endpointCol);

    if isempty(rowIndices)
        continue;
    end

    freeRows = rowIndices(1:max(1, numel(rowIndices) - 1));
    freeCols = colIndices(1:max(1, numel(colIndices) - 1));

    validFree = freeRows >= 1 & freeRows <= scene.rows & freeCols >= 1 & freeCols <= scene.cols;
    linearFree = sub2ind(size(probabilityGrid), freeRows(validFree), freeCols(validFree));
    probabilityGrid(linearFree) = max(0, probabilityGrid(linearFree) - freeGain);

    if scanData.hitMask(beamIdx)
        if endpointRow >= 1 && endpointRow <= scene.rows && endpointCol >= 1 && endpointCol <= scene.cols
            probabilityGrid(endpointRow, endpointCol) = min(1, probabilityGrid(endpointRow, endpointCol) + occupiedGain);
        end
    end
end
end

function mapPoints = extractOccupancyPoints(probabilityGrid, scene, thresholdValue)
occupiedIndices = probabilityGrid >= thresholdValue;
if ~any(occupiedIndices(:))
    mapPoints = zeros(0, 2);
    return;
end

[rowIndices, colIndices] = find(occupiedIndices);
mapPoints = [scene.xCenters(rowIndices(:)).', scene.yCenters(colIndices(:)).'];
end

function [bestPose, bestScanWorld, bestScore] = refinePoseByMapCorrelation(scanData, initialPose, scene, transWindow, rotWindow, transStep, rotStep)
bestPose = initialPose(:);
bestScore = -inf;

dxCandidates = -transWindow:transStep:transWindow;
dyCandidates = -transWindow:transStep:transWindow;
dThetaCandidates = -rotWindow:rotStep:rotWindow;

hitMask = scanData.hitMask(:);
if ~any(hitMask)
    reprojected = reprojectScanAtPose(scanData, bestPose);
    bestScanWorld = reprojected.pointsWorld;
    bestScore = 0;
    return;
end

for dx = dxCandidates
    for dy = dyCandidates
        for dTheta = dThetaCandidates
            candidatePose = [initialPose(1) + dx; initialPose(2) + dy; wrapToPiLocal(initialPose(3) + dTheta)];
            candidateScan = reprojectScanAtPose(scanData, candidatePose);
            hitPoints = candidateScan.pointsWorld(hitMask, :);

            rowIndices = floor((hitPoints(:, 1) - scene.xLimits(1)) * scene.resolution) + 1;
            colIndices = floor((hitPoints(:, 2) - scene.yLimits(1)) * scene.resolution) + 1;

            insideMask = rowIndices >= 1 & rowIndices <= scene.rows & colIndices >= 1 & colIndices <= scene.cols;
            if ~any(insideMask)
                continue;
            end

            rowValid = rowIndices(insideMask);
            colValid = colIndices(insideMask);
            linearIndices = sub2ind(size(scene.occupiedMask), rowValid, colValid);

            occupiedHits = nnz(scene.occupiedMask(linearIndices));
            insideCount = numel(linearIndices);
            outsideCount = nnz(~insideMask);
            motionPenalty = 0.25 * hypot(dx, dy) + 0.15 * abs(dTheta);

            candidateScore = occupiedHits / max(insideCount, 1) - 0.03 * outsideCount - motionPenalty;
            if candidateScore > bestScore
                bestScore = candidateScore;
                bestPose = candidatePose;
            end
        end
    end
end

reprojected = reprojectScanAtPose(scanData, bestPose);
bestScanWorld = reprojected.pointsWorld;
end

function [bestPose, bestScanWorld, bestCost] = refinePoseByRangeResidual(scanData, initialPose, scene, maxRange, rayStep, transWindow, rotWindow, transStep, rotStep)
bestPose = initialPose(:);
bestCost = inf;

dxCandidates = -transWindow:transStep:transWindow;
dyCandidates = -transWindow:transStep:transWindow;
dThetaCandidates = -rotWindow:rotStep:rotWindow;

validRangeMask = scanData.hitMask(:);
if nnz(validRangeMask) < 8
    validRangeMask = true(size(scanData.ranges));
end
observedRanges = scanData.ranges(:);

for dx = dxCandidates
    for dy = dyCandidates
        for dTheta = dThetaCandidates
            candidatePose = [initialPose(1) + dx; initialPose(2) + dy; wrapToPiLocal(initialPose(3) + dTheta)];
            predictedScan = simulateLidarScan(scene, candidatePose, scanData.scanAngles(:).', maxRange, rayStep);
            predictedRanges = predictedScan.ranges(:);

            rangeResidual = predictedRanges(validRangeMask) - observedRanges(validRangeMask);
            rmseCost = sqrt(mean(rangeResidual.^2));
            motionPenalty = 0.10 * hypot(dx, dy) + 0.05 * abs(dTheta);
            candidateCost = rmseCost + motionPenalty;

            if candidateCost < bestCost
                bestCost = candidateCost;
                bestPose = candidatePose;
            end
        end
    end
end

bestScan = reprojectScanAtPose(scanData, bestPose);
bestScanWorld = bestScan.pointsWorld;
end

function [bestPose, bestScanWorld, bestCost] = refinePoseByDistanceField(scanData, initialPose, scene, distanceMap, transWindow, rotWindow, transStep, rotStep)
bestPose = initialPose(:);
bestCost = inf;

dxCandidates = -transWindow:transStep:transWindow;
dyCandidates = -transWindow:transStep:transWindow;
dThetaCandidates = -rotWindow:rotStep:rotWindow;

useMask = scanData.hitMask(:);
if nnz(useMask) < 8
    useMask = true(size(scanData.ranges));
end

scanAngles = scanData.scanAngles(:);
scanRanges = scanData.ranges(:);
scanAngles = scanAngles(useMask);
scanRanges = scanRanges(useMask);

for dx = dxCandidates
    for dy = dyCandidates
        for dTheta = dThetaCandidates
            candidatePose = [initialPose(1) + dx; initialPose(2) + dy; wrapToPiLocal(initialPose(3) + dTheta)];
            worldX = candidatePose(1) + scanRanges .* cos(candidatePose(3) + scanAngles);
            worldY = candidatePose(2) + scanRanges .* sin(candidatePose(3) + scanAngles);

            endpointDistance = sampleDistanceField(distanceMap, scene, worldX, worldY);
            rmseCost = sqrt(mean(endpointDistance.^2));
            motionPenalty = 0.08 * hypot(dx, dy) + 0.04 * abs(dTheta);
            candidateCost = rmseCost + motionPenalty;

            if candidateCost < bestCost
                bestCost = candidateCost;
                bestPose = candidatePose;
            end
        end
    end
end

bestScan = reprojectScanAtPose(scanData, bestPose);
bestScanWorld = bestScan.pointsWorld;
end

function distanceMap = buildSceneDistanceMap(scene)
distanceMap = bwdist(scene.occupiedMask) / scene.resolution;
end

function sampledDistance = sampleDistanceField(distanceMap, scene, worldX, worldY)
numPoints = numel(worldX);
sampledDistance = 1.0 * ones(numPoints, 1);

rowCoords = (worldX - scene.xLimits(1)) * scene.resolution + 1;
colCoords = (worldY - scene.yLimits(1)) * scene.resolution + 1;

insideMask = rowCoords >= 1 & rowCoords <= scene.rows & colCoords >= 1 & colCoords <= scene.cols;
if any(insideMask)
    sampledDistance(insideMask) = interp2(1:scene.cols, 1:scene.rows, distanceMap, colCoords(insideMask), rowCoords(insideMask), 'linear', 1.0);
end
end

function reducedScan = downsampleScanForLocalization(scanData, beamStride)
beamStride = max(1, round(beamStride));
beamIndices = 1:beamStride:numel(scanData.ranges);

reducedScan = struct();
reducedScan.ranges = scanData.ranges(beamIndices);
reducedScan.pointsWorld = scanData.pointsWorld(beamIndices, :);
reducedScan.pointsBody = scanData.pointsBody(beamIndices, :);
reducedScan.hitMask = scanData.hitMask(beamIndices);
reducedScan.pose = scanData.pose;
reducedScan.scanAngles = scanData.scanAngles(beamIndices);
end

function localPoints = selectLocalMapPoints(pointsWorld, pose, radiusValue)
if isempty(pointsWorld)
    localPoints = zeros(0, 2);
    return;
end

deltaX = pointsWorld(:, 1) - pose(1);
deltaY = pointsWorld(:, 2) - pose(2);
distanceMask = (deltaX.^2 + deltaY.^2) <= radiusValue^2;
localPoints = pointsWorld(distanceMask, :);
end

function updatedPose = applyBoundedIcpUpdate(predictedPose, candidatePose, maxTranslation, maxRotation)
relativeCorrection = estimateRelativeMotion(predictedPose, candidatePose);

translationNorm = norm(relativeCorrection(1:2));
if translationNorm > maxTranslation
    relativeCorrection(1:2) = relativeCorrection(1:2) * (maxTranslation / max(translationNorm, 1e-9));
end
relativeCorrection(3) = min(max(relativeCorrection(3), -maxRotation), maxRotation);

updatedPose = composePose(predictedPose, relativeCorrection);
updatedPose(3) = wrapToPiLocal(updatedPose(3));
end

function composedPose = composePose(basePose, relativePose)
baseTransform = poseToTransform(basePose);
relativeTransform = poseToTransform(relativePose);
composedPose = transformToPose(relativeTransform * baseTransform);
end

function [rowIndices, colIndices] = lineCells(startRow, startCol, endRow, endCol)
numSamples = max(abs(endRow - startRow), abs(endCol - startCol)) + 1;
rowIndices = round(linspace(startRow, endRow, numSamples));
colIndices = round(linspace(startCol, endCol, numSamples));
[~, uniqueIndices] = unique([rowIndices(:), colIndices(:)], 'rows', 'stable');
rowIndices = rowIndices(uniqueIndices);
colIndices = colIndices(uniqueIndices);
end

function [poseEstimate, transformedSource] = matchScanPairs(sourcePoints, targetPoints, initialGuessPose, maxIterations)
poseTransform = poseToTransform(initialGuessPose);
maxCorrespondenceDistance = 1.8;
minCorrespondences = 20;

for iterationIdx = 1:maxIterations
    transformedSource = transformPoints2D(sourcePoints, poseTransform);
    nearestTargetIndices = nearestNeighborIndices(transformedSource, targetPoints);
    targetCorrespondences = targetPoints(nearestTargetIndices, :);

    residuals = targetCorrespondences - transformedSource;
    correspondenceDistances = sqrt(sum(residuals.^2, 2));
    validMask = correspondenceDistances < maxCorrespondenceDistance;
    if nnz(validMask) < minCorrespondences
        break;
    end

    deltaTransform = estimateRigidTransform(transformedSource(validMask, :), targetCorrespondences(validMask, :));
    poseTransform = deltaTransform * poseTransform;

    if norm(deltaTransform - eye(3), 'fro') < 1e-4
        break;
    end
end

poseEstimate = transformToPose(poseTransform);
transformedSource = transformPoints2D(sourcePoints, poseTransform);
end

function nearestIndices = nearestNeighborIndices(sourcePoints, targetPoints)
numSourcePoints = size(sourcePoints, 1);
nearestIndices = zeros(numSourcePoints, 1);

for sourceIdx = 1:numSourcePoints
    deltaX = targetPoints(:, 1) - sourcePoints(sourceIdx, 1);
    deltaY = targetPoints(:, 2) - sourcePoints(sourceIdx, 2);
    [~, nearestIndices(sourceIdx)] = min(deltaX.^2 + deltaY.^2);
end
end

function transformMatrix = estimateRigidTransform(sourcePoints, targetPoints)
sourceCentroid = mean(sourcePoints, 1);
targetCentroid = mean(targetPoints, 1);

centeredSource = sourcePoints - sourceCentroid;
centeredTarget = targetPoints - targetCentroid;

covarianceMatrix = centeredSource.' * centeredTarget;
[leftSingularVectors, ~, rightSingularVectors] = svd(covarianceMatrix);
rotationMatrix = rightSingularVectors * leftSingularVectors.';
if det(rotationMatrix) < 0
    rightSingularVectors(:, end) = -rightSingularVectors(:, end);
    rotationMatrix = rightSingularVectors * leftSingularVectors.';
end

translationVector = targetCentroid(:) - rotationMatrix * sourceCentroid(:);
transformMatrix = [rotationMatrix, translationVector; 0, 0, 1];
end

function transformedPoints = transformPoints2D(points, poseOrTransform)
if numel(poseOrTransform) == 3
    transformMatrix = poseToTransform(poseOrTransform);
else
    transformMatrix = poseOrTransform;
end

homogeneousPoints = [points, ones(size(points, 1), 1)].';
transformedHomogeneous = transformMatrix * homogeneousPoints;
transformedPoints = transformedHomogeneous(1:2, :).';
end

function transformMatrix = poseToTransform(pose)
pose = pose(:);
heading = pose(3);
rotationMatrix = [cos(heading), -sin(heading);
                  sin(heading),  cos(heading)];
transformMatrix = [rotationMatrix, pose(1:2); 0, 0, 1];
end

function pose = transformToPose(transformMatrix)
heading = atan2(transformMatrix(2,1), transformMatrix(1,1));
pose = [transformMatrix(1,3); transformMatrix(2,3); wrapToPiLocal(heading)];
end

function relativePose = estimateRelativeMotion(poseStart, poseEnd)
startTransform = poseToTransform(poseStart);
endTransform = poseToTransform(poseEnd);
relativeTransform = endTransform / startTransform;
relativePose = transformToPose(relativeTransform);
end

function relativePose = invertRelativePose(relativePoseIn)
relativeTransform = poseToTransform(relativePoseIn);
inverseTransform = inv(relativeTransform);
relativePose = transformToPose(inverseTransform);
end

function limitedRelativePose = clampRelativePose(relativePoseIn, maxTranslation, maxRotation)
limitedRelativePose = relativePoseIn(:);
translationNorm = norm(limitedRelativePose(1:2));
if translationNorm > maxTranslation
    limitedRelativePose(1:2) = limitedRelativePose(1:2) * (maxTranslation / max(translationNorm, 1e-9));
end
limitedRelativePose(3) = min(max(limitedRelativePose(3), -maxRotation), maxRotation);
limitedRelativePose(3) = wrapToPiLocal(limitedRelativePose(3));
end

function plotScanComparison(scanReference, scanCandidate, referenceLabel, candidateLabel)
scatter(scanReference(:, 2), scanReference(:, 1), 8, 'b', 'filled'); hold on;
scatter(scanCandidate(:, 2), scanCandidate(:, 1), 8, 'r', 'filled');
grid on; axis equal;
xlabel('Y'); ylabel('X');
legend(referenceLabel, candidateLabel, 'Location', 'best');
end

function plotSceneMap(scene, scanA, scanB, scanBTransformed)
occupiedPlot = 0.75 * ones(size(scene.occupiedMask));
occupiedPlot(scene.occupiedMask) = 0.45;
imagesc(scene.yCenters, scene.xCenters, occupiedPlot);
colormap(gca, gray);
caxis([0, 1]);
set(gca, 'YDir', 'normal'); axis equal tight; hold on;
refHandle = scatter(scanA.pointsWorld(:, 2), scanA.pointsWorld(:, 1), 8, 'b', 'filled');
scanHandle = scatter(scanB.pointsWorld(:, 2), scanB.pointsWorld(:, 1), 8, 'r', 'filled');
alignedHandle = scatter(scanBTransformed(:, 2), scanBTransformed(:, 1), 8, 'm', 'filled');
xlabel('Y'); ylabel('X');
legend([refHandle, scanHandle, alignedHandle], {'Scan 131', 'Scan 133', 'Scan 133 transformed'}, 'Location', 'best');
hold off;
end

function [plotX, plotY] = buildFovConePlot(pose, scanAngles, maxRange, numSamples)
pose = pose(:);
leftEdge = min(scanAngles);
rightEdge = max(scanAngles);
angleSamples = linspace(leftEdge, rightEdge, numSamples);

coneXWorld = pose(1) + maxRange * cos(pose(3) + angleSamples);
coneYWorld = pose(2) + maxRange * sin(pose(3) + angleSamples);

plotX = [pose(2), coneYWorld, pose(2)];
plotY = [pose(1), coneXWorld, pose(1)];
end

function ang = wrapToPiLocal(ang)
ang = mod(ang + pi, 2*pi) - pi;
end

function poseNext = propagateState(poseCurrent, vCommand, omegaCommand, dt)
heading = poseCurrent(3);
poseNext = poseCurrent + dt * [vCommand * cos(heading); vCommand * sin(heading); omegaCommand];
poseNext(3) = wrapToPiLocal(poseNext(3));
end

function [updatedPose, updatedCovariance] = ekfUpdatePoseMeasurement(predictedPose, predictedCovariance, measuredPose, measurementCovariance)
measurementModel = eye(3);
innovation = measuredPose(:) - predictedPose(:);
innovation(3) = wrapToPiLocal(innovation(3));

% Begrenze Messkorrektur pro Schritt fuer robustes Verhalten bei Fehlzuordnung.
innovation(1) = min(max(innovation(1), -0.02), 0.02);
innovation(2) = min(max(innovation(2), -0.02), 0.02);
innovation(3) = min(max(innovation(3), -deg2rad(2.0)), deg2rad(2.0));

innovationCovariance = measurementModel * predictedCovariance * measurementModel.' + measurementCovariance;

% Chi-Quadrat-Gating (3 DoF): verwerfe unplausible Messungen.
normalizedInnovationSquared = innovation.' * (innovationCovariance \ innovation);
if normalizedInnovationSquared > 7.81
    updatedPose = predictedPose(:);
    updatedCovariance = predictedCovariance;
    return;
end

kalmanGain = predictedCovariance * measurementModel.' / innovationCovariance;

updatedPose = predictedPose(:) + kalmanGain * innovation;
updatedPose(3) = wrapToPiLocal(updatedPose(3));

identityMatrix = eye(3);
updatedCovariance = (identityMatrix - kalmanGain * measurementModel) * predictedCovariance;
updatedCovariance = 0.5 * (updatedCovariance + updatedCovariance.');
end

function durationValue = computePolylineDuration(waypointsXY, speedValue)
segmentVectors = diff(waypointsXY, 1, 1);
segmentLengths = vecnorm(segmentVectors, 2, 2);
totalLength = sum(segmentLengths);
durationValue = totalLength / max(speedValue, 1e-6);
end

function poseValue = polylinePose(currentTime, waypointsXY, speedValue)
segmentVectors = diff(waypointsXY, 1, 1);
segmentLengths = vecnorm(segmentVectors, 2, 2);
cumulativeLengths = [0; cumsum(segmentLengths)];
totalLength = cumulativeLengths(end);

distanceTravelled = min(max(currentTime, 0) * speedValue, totalLength);
segmentIndex = find(distanceTravelled <= cumulativeLengths(2:end), 1, 'first');
if isempty(segmentIndex)
    segmentIndex = size(segmentVectors, 1);
end

segmentStart = waypointsXY(segmentIndex, :);
segmentVector = segmentVectors(segmentIndex, :);
segmentLength = max(segmentLengths(segmentIndex), 1e-9);
distanceOnSegment = distanceTravelled - cumulativeLengths(segmentIndex);
segmentFraction = min(max(distanceOnSegment / segmentLength, 0), 1);

positionXY = segmentStart + segmentFraction * segmentVector;
headingValue = atan2(segmentVector(2), segmentVector(1));

poseValue = [positionXY(1); positionXY(2); wrapToPiLocal(headingValue)];
end

function poseValue = polylinePoseWithCornerTurns(currentTime, waypointsXY, speedValue, cornerTurnDuration)
segmentVectors = diff(waypointsXY, 1, 1);
segmentLengths = vecnorm(segmentVectors, 2, 2);
numSegments = size(segmentVectors, 1);

if numSegments < 1
    poseValue = [waypointsXY(1, 1); waypointsXY(1, 2); 0];
    return;
end

segmentHeadings = atan2(segmentVectors(:, 2), segmentVectors(:, 1));
clock = max(currentTime, 0);

for segIdx = 1:numSegments
    moveDuration = segmentLengths(segIdx) / max(speedValue, 1e-6);
    if clock <= moveDuration
        moveFraction = min(max(clock / max(moveDuration, 1e-9), 0), 1);
        currentPos = waypointsXY(segIdx, :) + moveFraction * segmentVectors(segIdx, :);
        poseValue = [currentPos(1); currentPos(2); wrapToPiLocal(segmentHeadings(segIdx))];
        return;
    end
    clock = clock - moveDuration;

    hasFollowingSegment = segIdx < numSegments;
    if hasFollowingSegment
        if clock <= cornerTurnDuration
            turnFraction = min(max(clock / max(cornerTurnDuration, 1e-9), 0), 1);
            startHeading = segmentHeadings(segIdx);
            headingStep = wrapToPiLocal(segmentHeadings(segIdx + 1) - segmentHeadings(segIdx));
            turnHeading = wrapToPiLocal(startHeading + turnFraction * headingStep);
            turnPoint = waypointsXY(segIdx + 1, :);
            poseValue = [turnPoint(1); turnPoint(2); turnHeading];
            return;
        end
        clock = clock - cornerTurnDuration;
    end
end

poseValue = [waypointsXY(end, 1); waypointsXY(end, 2); wrapToPiLocal(segmentHeadings(end))];
end

function commandValue = commandFromPoseStep(previousPose, currentPose, dt)
relativeMotion = estimateRelativeMotion(previousPose, currentPose);
speedValue = norm(relativeMotion(1:2)) / max(dt, 1e-6);
headingRate = wrapToPiLocal(relativeMotion(3)) / max(dt, 1e-6);
commandValue = [speedValue; headingRate];
end

function command = roundedRectCommand(currentTime, widthValue, heightValue, cornerRadiusValue, periodValue)
cornerRadiusValue = min(cornerRadiusValue, 0.49 * min(widthValue, heightValue));

perimeterLength = 2 * (widthValue - 2*cornerRadiusValue) + 2 * (heightValue - 2*cornerRadiusValue) + 2*pi*cornerRadiusValue;
speedValue = perimeterLength / periodValue;
positionAlongPath = mod(speedValue * currentTime, perimeterLength);

segment1 = widthValue - 2*cornerRadiusValue;
segment2 = 0.5*pi*cornerRadiusValue;
segment3 = heightValue - 2*cornerRadiusValue;
segment4 = 0.5*pi*cornerRadiusValue;
segment5 = widthValue - 2*cornerRadiusValue;
segment6 = 0.5*pi*cornerRadiusValue;
segment7 = heightValue - 2*cornerRadiusValue;

if positionAlongPath < segment1 || positionAlongPath >= segment1 + segment2 + segment3 + segment4 + segment5 + segment6 + segment7
    omegaValue = 0;
elseif positionAlongPath < segment1 + segment2
    omegaValue = speedValue / cornerRadiusValue;
elseif positionAlongPath < segment1 + segment2 + segment3
    omegaValue = 0;
elseif positionAlongPath < segment1 + segment2 + segment3 + segment4
    omegaValue = speedValue / cornerRadiusValue;
elseif positionAlongPath < segment1 + segment2 + segment3 + segment4 + segment5
    omegaValue = 0;
elseif positionAlongPath < segment1 + segment2 + segment3 + segment4 + segment5 + segment6
    omegaValue = speedValue / cornerRadiusValue;
else
    omegaValue = 0;
end

command = [speedValue; omegaValue];
end

function pose = roundedRectPose(currentTime, widthValue, heightValue, cornerRadiusValue, periodValue, rotationDeg)
cornerRadiusValue = min(cornerRadiusValue, 0.49 * min(widthValue, heightValue));
perimeterLength = 2 * (widthValue - 2*cornerRadiusValue) + 2 * (heightValue - 2*cornerRadiusValue) + 2*pi*cornerRadiusValue;
speedValue = perimeterLength / periodValue;
positionAlongPath = mod(speedValue * currentTime, perimeterLength);

segment1 = widthValue - 2*cornerRadiusValue;
segment2 = 0.5*pi*cornerRadiusValue;
segment3 = heightValue - 2*cornerRadiusValue;
segment4 = 0.5*pi*cornerRadiusValue;
segment5 = widthValue - 2*cornerRadiusValue;
segment6 = 0.5*pi*cornerRadiusValue;
segment7 = heightValue - 2*cornerRadiusValue;

if positionAlongPath < segment1
    localX = -widthValue/2 + cornerRadiusValue + positionAlongPath;
    localY = -heightValue/2;
    localHeading = 0;
elseif positionAlongPath < segment1 + segment2
    arcDistance = positionAlongPath - segment1;
    localAngle = -pi/2 + arcDistance / cornerRadiusValue;
    arcCenter = [widthValue/2 - cornerRadiusValue; -heightValue/2 + cornerRadiusValue];
    arcPoint = arcCenter + cornerRadiusValue * [cos(localAngle); sin(localAngle)];
    localX = arcPoint(1);
    localY = arcPoint(2);
    localHeading = localAngle + pi/2;
elseif positionAlongPath < segment1 + segment2 + segment3
    arcDistance = positionAlongPath - (segment1 + segment2);
    localX = widthValue/2;
    localY = -heightValue/2 + cornerRadiusValue + arcDistance;
    localHeading = pi/2;
elseif positionAlongPath < segment1 + segment2 + segment3 + segment4
    arcDistance = positionAlongPath - (segment1 + segment2 + segment3);
    localAngle = 0 + arcDistance / cornerRadiusValue;
    arcCenter = [widthValue/2 - cornerRadiusValue; heightValue/2 - cornerRadiusValue];
    arcPoint = arcCenter + cornerRadiusValue * [cos(localAngle); sin(localAngle)];
    localX = arcPoint(1);
    localY = arcPoint(2);
    localHeading = localAngle + pi/2;
elseif positionAlongPath < segment1 + segment2 + segment3 + segment4 + segment5
    arcDistance = positionAlongPath - (segment1 + segment2 + segment3 + segment4);
    localX = widthValue/2 - cornerRadiusValue - arcDistance;
    localY = heightValue/2;
    localHeading = pi;
elseif positionAlongPath < segment1 + segment2 + segment3 + segment4 + segment5 + segment6
    arcDistance = positionAlongPath - (segment1 + segment2 + segment3 + segment4 + segment5);
    localAngle = pi/2 + arcDistance / cornerRadiusValue;
    arcCenter = [-widthValue/2 + cornerRadiusValue; heightValue/2 - cornerRadiusValue];
    arcPoint = arcCenter + cornerRadiusValue * [cos(localAngle); sin(localAngle)];
    localX = arcPoint(1);
    localY = arcPoint(2);
    localHeading = localAngle + pi/2;
elseif positionAlongPath < segment1 + segment2 + segment3 + segment4 + segment5 + segment6 + segment7
    arcDistance = positionAlongPath - (segment1 + segment2 + segment3 + segment4 + segment5 + segment6);
    localX = -widthValue/2;
    localY = heightValue/2 - cornerRadiusValue - arcDistance;
    localHeading = -pi/2;
else
    arcDistance = positionAlongPath - (segment1 + segment2 + segment3 + segment4 + segment5 + segment6 + segment7);
    localAngle = pi + arcDistance / cornerRadiusValue;
    arcCenter = [-widthValue/2 + cornerRadiusValue; -heightValue/2 + cornerRadiusValue];
    arcPoint = arcCenter + cornerRadiusValue * [cos(localAngle); sin(localAngle)];
    localX = arcPoint(1);
    localY = arcPoint(2);
    localHeading = localAngle + pi/2;
end

rotationAngle = deg2rad(rotationDeg);
rotationMatrix = [cos(rotationAngle), -sin(rotationAngle); sin(rotationAngle), cos(rotationAngle)];
rotatedPoint = rotationMatrix * [localX; localY];
pose = [rotatedPoint(1); rotatedPoint(2); wrapToPiLocal(localHeading + rotationAngle)];
end
