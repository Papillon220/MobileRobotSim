% ==========   PATH PLANNING ALGORITHMS   ============
%
%   Prof. Dr.-Ing. Frank Bender
%   Elektrotechnik und Informationstechnik
%   DHBW Stuttgart
%   25.06.2026
%
%=====================================================

%% Define Vehicle
trackWidth = 0.2;       % Distance between left and right wheels [m]
vehicle = differentialDriveKinematics("TrackWidth",trackWidth, ...
    "VehicleInputs","VehicleSpeedHeadingRate");

startPose = [2 2 0];            % Start pose [x y theta]
goalPose = [18 18 -pi/2];       % Goal pose [x y theta]


% Create visualizer
viz = Visualizer2D;
viz.robotRadius = trackWidth/2;
viz.hasWaypoints = false;
viz.mapName = 'map';

%% Choose path planner


planningAlgorithm = "plannerPRMsmooth"; 


% Options: 
% plannerBug2
% plannerAStar
% plannerRRT
% plannerRRTStar
% plannerRRTdubins
% distanceTransform
% plannerPRM
% plannerPRMsmooth


% Configurations for Bug2 planner: "left" or "right" wall following direction
bug2Direction = "right"; 

% Configuration for path following overshoot reduction at sharp corners:
% 0 - None (Standard Pure Pursuit)
% 1 - Rotate in place (Stop and turn on the spot when heading error > threshold)
% 2 - Dynamic speed scaling (Slow down forward progress based on steering rate)
overshootReductionMode = 2;


%% Simulation parameters
sampleTime = 0.1;               % Sample time [s]

% Allow longer travel time for slow planners like Bug2
if exist('planningAlgorithm','var') && contains(lower(planningAlgorithm), "bug2")
    tVec = 0:sampleTime:250;
else
    tVec = 0:sampleTime:150;        % Time array
end

if contains(lower(planningAlgorithm), "prm") 
    enablePathSmoothing = contains(lower(planningAlgorithm), "smooth");
else
    enablePathSmoothing = false; % Toggle path smoothing / shortcutting for graph-based planners (PRM & A*)
end

% Toggle representation of gradient descent arrows for distanceTransform planner
showArrows = true; 

globalRngSeed = 1; % Seed for reproducibility (PRM, RRT, A* backbone, and short-cutting)


rttStarNumRuns = 3;
rttStarSeedBase = 1;
rttStarShortcutTrials = 150;
fprintf('[INFO] Starting planning with %s\n', planningAlgorithm);
fprintf('[INFO] Start pose: [%.2f %.2f %.2f], Goal pose: [%.2f %.2f %.2f]\n', ...
    startPose(1),startPose(2),startPose(3),goalPose(1),goalPose(2),goalPose(3));


prmNumNodes = 1200;
prmMaxConnectionDistance = 2.5;
astarPrmNodeTrials = [800 1200];
astarPrmConnTrials = [1.8 2.5];

load complexMap

% Increase map resolution to 10 cells/m for finer obstacle edges
newResolution = 10;
if map.Resolution < newResolution
    scaleFactor = newResolution / map.Resolution;
    matrixHigh = imresize(getOccupancy(map), scaleFactor, "nearest");
    map = occupancyMap(matrixHigh, newResolution);
end

inflate(map,0.25); % Inflate the map for planning
fprintf('[INFO] Map loaded, scaled to %d cells/m, and inflated (0.25 m safety margin).\n', map.Resolution);

% State space selection based on algorithm
if contains(lower(planningAlgorithm), "rrt") && contains(lower(planningAlgorithm), "dubins")
    ss = stateSpaceDubins;
    ss.MinTurningRadius = 0.75;
    ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
else
    % Use highly efficient SE2 state space for graph-based planners (PRM & A*) and geometric RRT/RRT*
    ss = stateSpaceSE2;
    ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
end

% State validator
sv = validatorOccupancyMap(ss);
sv.Map = map;
sv.ValidationDistance = 0.1;

% Set RNG seed for absolute reproducibility of randomized components
rng(globalRngSeed);

% Path planner selection
tPlanStart = tic;
switch lower(planningAlgorithm)
    case {"plannerrrt", "plannerrrtdubins"}
        fprintf('[INFO] plannerRRT: creating planner and searching path...\n');
        planner = plannerRRT(ss,sv);
        planner.MaxConnectionDistance = 2.5;
        [plannedPath,solInfo] = plan(planner,startPose,goalPose);
        if plannedPath.NumStates > 0
            interpolate(plannedPath,round(2*plannedPath.pathLength)); % Approx. 2 waypoints per meter
            plannedStates = plannedPath.States;
            fprintf('[INFO] plannerRRT: path found with %d states.\n', plannedPath.NumStates);
        else
            plannedStates = zeros(0,3);
            fprintf('[WARN] plannerRRT: no valid path found.\n');
        end

    case "plannerrrtstar"
        fprintf('[INFO] plannerRRTStar: creating planner and searching path...\n');
        planner = plannerRRTStar(ss,sv);
        planner.MaxConnectionDistance = 1.5;
        planner.ContinueAfterGoalReached = false;
        planner.MaxIterations = 6000;
        planner.MaxNumTreeNodes = 6000;
        planner.GoalBias = 0.15;

        fprintf('[INFO] plannerRRTStar: running best-of-%d with shortcut post-processing...\n', rttStarNumRuns);
        [plannedPath,solInfo] = runRRTStarBestOfN( ...
            planner,startPose,goalPose,sv,rttStarNumRuns,rttStarSeedBase,rttStarShortcutTrials);

        if plannedPath.NumStates > 0
            numInterpStates = max(plannedPath.NumStates,round(2*plannedPath.pathLength));
            interpolate(plannedPath,numInterpStates); % Approx. 2 waypoints per meter
            plannedStates = plannedPath.States;
            fprintf('[INFO] plannerRRTStar: path found with %d states.\n', plannedPath.NumStates);
        else
            plannedStates = zeros(0,3);
            fprintf('[WARN] plannerRRTStar: no valid path found.\n');
        end

    case {"plannerprm", "plannerprmsmooth"}
        fprintf('[INFO] plannerPRM: building roadmap and searching graph path...\n');
        [planner,plannedPath,solInfo] = planWithPRMRetries( ...
            ss,sv,startPose,goalPose,prmNumNodes,prmMaxConnectionDistance);
        if plannedPath.NumStates > 0
            % Keep a copy of the planner object inside solInfo so the visualizer can access the graph of nodes and connections
            solInfo.Planner = planner;
            plannedStates = plannedPath.States;
            if enablePathSmoothing
                fprintf('[INFO] plannerPRM: applying shortcut smoothing on raw roadmap nodes...\n');
                plannedStates = shortcutPath(plannedStates, sv, 250);
            end
            
            % Save to temporary navPath structure to interpolate smoothly
            tempPath = navPath(ss, plannedStates);
            interpolate(tempPath, round(2*tempPath.pathLength)); % Approx. 2 waypoints per meter
            plannedStates = tempPath.States;
            
            if enablePathSmoothing
                % Quick final pass to tidy up after interpolation
                fprintf('[INFO] plannerPRM: applying final shortcut smoothing pass...\n');
                plannedStates = shortcutPath(plannedStates, sv, 150);
            end
            fprintf('[INFO] plannerPRM: path found with %d states.\n', size(plannedStates,1));
        else
            plannedStates = zeros(0,3);
            fprintf('[WARN] plannerPRM: no valid path found.\n');
        end

    case "plannerastar"
        % Build a roadmap first, then run A* on the resulting graph.
        fprintf('[INFO] plannerAStar: generating PRM roadmap backbone...\n');
        [prmPlanner,prmPath,prmSolInfo] = planWithPRMRetries( ...
            ss,sv,startPose,goalPose,astarPrmNodeTrials,astarPrmConnTrials);

        if prmPath.NumStates < 1
            fprintf('[WARN] plannerAStar: fast PRM trials failed, switching to robust PRM retries...\n');
            [prmPlanner,prmPath,prmSolInfo] = planWithPRMRetries( ...
                ss,sv,startPose,goalPose,prmNumNodes,prmMaxConnectionDistance);
        end

        graphObj = navGraph(graphData(prmPlanner));
        fprintf('[INFO] plannerAStar: roadmap graph size = %d nodes.\n', size(graphObj.States,1));

        graphStates = graphObj.States{:,1};
        startNodeCandidates = nearestPoseIndices(graphStates,startPose,6);
        goalNodeCandidates = nearestPoseIndices(graphStates,goalPose,6);
        fprintf('[INFO] plannerAStar: trying up to %d x %d start/goal node pairs.\n', ...
            numel(startNodeCandidates),numel(goalNodeCandidates));

        planner = plannerAStar(graphObj);
        planner.HeuristicCostFcn = @nav.algs.distanceEuclidean;
        planner.TieBreaker = true;
        fprintf('[INFO] plannerAStar: running shortest-path search...\n');
        astarPath = [];
        solInfo = struct();
        for si = 1:numel(startNodeCandidates)
            for gi = 1:numel(goalNodeCandidates)
                startNode = startNodeCandidates(si);
                goalNode = goalNodeCandidates(gi);
                [candidatePath,candidateInfo] = plan(planner,graphStates(startNode,:),graphStates(goalNode,:));
                if ~isempty(candidatePath)
                    astarPath = candidatePath;
                    solInfo = candidateInfo;
                    fprintf('[INFO] plannerAStar: path found using start node %d and goal node %d.\n', startNode, goalNode);
                    break;
                end
            end
            if ~isempty(astarPath)
                break;
            end
        end

        if ~isempty(astarPath)
            plannedStates = [startPose; astarPath; goalPose];
            if enablePathSmoothing
                fprintf('[INFO] plannerAStar: applying shortcut smoothing...\n');
                plannedStates = shortcutPath(plannedStates, sv, 150);
            end
            fprintf('[INFO] plannerAStar: path found with %d states (with start/goal anchors).\n', size(plannedStates,1));
        else
            if prmPath.NumStates > 0
                plannedStates = prmPath.States;
                if enablePathSmoothing
                    fprintf('[INFO] plannerAStar: applying shortcut smoothing to fallback path...\n');
                    plannedStates = shortcutPath(plannedStates, sv, 150);
                end
                solInfo = prmSolInfo;
                fprintf('[WARN] plannerAStar: no graph path found; using PRM fallback path with %d states.\n', size(plannedStates,1));
            else
                plannedStates = zeros(0,3);
                fprintf('[WARN] plannerAStar: no valid path found.\n');
            end
        end

    case {"plannerbug2", "bug2"}
        % Determine wall following direction parameter
        if exist('bug2Direction', 'var')
            wf_dir = lower(string(bug2Direction));
        else
            wf_dir = "left";
        end
        fprintf('[INFO] Bug2: calculating path via Bug2 (%s-handed) wall-following...\n', wf_dir);
        
        x_s = startPose(1:2);
        x_g = goalPose(1:2);
        
        % M-line: A*x + B*y + C = 0
        A = x_s(2) - x_g(2);
        B = x_g(1) - x_s(1);
        C = x_s(1)*x_g(2) - x_g(1)*x_s(2);
        m_line_denom = sqrt(A^2 + B^2);
        get_signed_dist = @(p) (A*p(1) + B*p(2) + C) / m_line_denom;
        
        step_size = 0.15;
        max_steps = 15000;
        path_pts = x_s;
        
        current_pos = x_s;
        mode = "GTG"; % "GTG" (Go To Goal) or "WF" (Wall Following)
        hit_point = [inf, inf];
        
        last_angle = atan2(x_g(2)-x_s(2), x_g(1)-x_s(1));
        solInfo = struct();
        
        for step = 1:max_steps
            dist_to_goal = norm(x_g - current_pos);
            if dist_to_goal <= step_size
                path_pts = [path_pts; x_g];
                break;
            end
            
            if mode == "GTG"
                dir_to_goal = (x_g - current_pos) / dist_to_goal;
                next_pos = current_pos + step_size * dir_to_goal;
                
                if isStateValid(sv, [next_pos, 0])
                    current_pos = next_pos;
                    path_pts = [path_pts; current_pos];
                    last_angle = atan2(dir_to_goal(2), dir_to_goal(1));
                else
                    mode = "WF";
                    hit_point = current_pos;
                    last_angle = atan2(dir_to_goal(2), dir_to_goal(1));
                    
                    % Find initial free direction to start wall following
                    found_step = false;
                    if wf_dir == "right"
                        % For right wall following, turn right (negative angles first)
                        scan_angles = (0:-5:-180) * (pi/180);
                    else
                        % For left wall following, turn left (positive angles first)
                        scan_angles = (0:5:180) * (pi/180);
                    end
                    
                    for sa = scan_angles
                        test_angle = last_angle + sa;
                        test_pos = current_pos + step_size * [cos(test_angle), sin(test_angle)];
                        if isStateValid(sv, [test_pos, 0])
                            current_pos = test_pos;
                            path_pts = [path_pts; current_pos];
                            last_angle = test_angle;
                            found_step = true;
                            break;
                        end
                    end
                    if ~found_step
                        break;
                    end
                end
            elseif mode == "WF"
                found_step = false;
                if wf_dir == "right"
                    % Scan from left (+90) to right (-180) relative to last_angle
                    scan_angles = (90:-5:-180) * (pi/180);
                else
                    % Scan from right (-90) to left (+180) relative to last_angle
                    scan_angles = (-90:5:180) * (pi/180);
                end
                
                for sa = scan_angles
                    test_angle = last_angle + sa;
                    test_pos = current_pos + step_size * [cos(test_angle), sin(test_angle)];
                    if isStateValid(sv, [test_pos, 0])
                        current_pos = test_pos;
                        path_pts = [path_pts; current_pos];
                        last_angle = test_angle;
                        found_step = true;
                        break;
                    end
                end
                if ~found_step
                    break;
                end
                
                curr_signed_dist = get_signed_dist(current_pos);
                prev_signed_dist = get_signed_dist(path_pts(end-1, :));
                crossed_m_line = (sign(curr_signed_dist) ~= sign(prev_signed_dist)) || (abs(curr_signed_dist) < 0.5 * step_size);
                
                if crossed_m_line
                    dist_to_goal = norm(x_g - current_pos);
                    hit_dist_to_goal = norm(x_g - hit_point);
                    dist_to_hit = norm(current_pos - hit_point);
                    
                    if (dist_to_goal < hit_dist_to_goal - 0.2) && (dist_to_hit > 0.3)
                        dir_to_goal = (x_g - current_pos) / dist_to_goal;
                        next_test = current_pos + step_size * dir_to_goal;
                        if isStateValid(sv, [next_test, 0])
                            mode = "GTG";
                            last_angle = atan2(dir_to_goal(2), dir_to_goal(1));
                        end
                    end
                end
            end
        end
        
        numPts = size(path_pts, 1);
        thetas = zeros(numPts, 1);
        for i = 1:numPts-1
            thetas(i) = atan2(path_pts(i+1,2)-path_pts(i,2), path_pts(i+1,1)-path_pts(i,1));
        end
        thetas(numPts) = goalPose(3);
        plannedStates = [path_pts, thetas];
        fprintf('[INFO] Bug2 planning finished. Steps: %d. Path contains %d states.\n', step, numPts);

    case "distancetransform"
        fprintf('[INFO] Distance Transform: calculating shortest path via geodesic distance transform in cell space...\n');
        goal_grid = world2grid(map, [goalPose(1) goalPose(2)]);
        start_grid = world2grid(map, [startPose(1) startPose(2)]);
        occ_matrix = getOccupancy(map);
        BW = (occ_matrix < 0.5); % Free space in planning/inflated map
        
        % Compute geodesic distance to goal in cells
        D = bwdistgeodesic(BW, goal_grid(2), goal_grid(1), 'quasi-euclidean');
        
        % Descent trace from start to goal
        current = start_grid;
        path_grid = current;
        max_steps = 15000;
        step_count = 0;
        while norm(double(current - goal_grid)) > 1.5 && step_count < max_steps
            step_count = step_count + 1;
            row = current(1);
            col = current(2);
            
            [dr, dc] = meshgrid(-1:1, -1:1);
            dr = dr(:); dc = dc(:);
            keep = (dr ~= 0) | (dc ~= 0);
            dr = dr(keep); dc = dc(keep);
            
            neighbor_rows = row + dr;
            neighbor_cols = col + dc;
            
            valid = neighbor_rows >= 1 & neighbor_rows <= size(D,1) & ...
                    neighbor_cols >= 1 & neighbor_cols <= size(D,2);
            neighbor_rows = neighbor_rows(valid);
            neighbor_cols = neighbor_cols(valid);
            
            ind = sub2ind(size(D), neighbor_rows, neighbor_cols);
            vals = D(ind);
            
            [min_val, min_idx] = min(vals);
            if isempty(min_val) || isnan(min_val) || isinf(min_val)
                break;
            end
            current = [neighbor_rows(min_idx), neighbor_cols(min_idx)];
            path_grid = [path_grid; current];
        end
        
        path_world = grid2world(map, path_grid);
        
        % Filter out duplicate/ultra-close successive points
        d_pts = vecnorm(diff(path_world,1,1),2,2);
        path_world = [path_world(1,:); path_world(find(d_pts > 1e-4) + 1, :)];
        
        % Prepend/append precise start and goal points
        if norm(path_world(1,:) - startPose(1:2)) > 1e-3
            path_world = [startPose(1:2); path_world];
        end
        if norm(path_world(end,:) - goalPose(1:2)) > 1e-3
            path_world = [path_world; goalPose(1:2)];
        end
        
        numPts = size(path_world, 1);
        thetas = zeros(numPts, 1);
        for i = 1:numPts-1
            thetas(i) = atan2(path_world(i+1,2)-path_world(i,2), path_world(i+1,1)-path_world(i,1));
        end
        thetas(numPts) = goalPose(3);
        plannedStates = [path_world, thetas];
        solInfo = struct();
        solInfo.D = D; % Save distance matrix for visualization
        fprintf('[INFO] Distance Transform path found with %d states.\n', numPts);

    otherwise
        error('Unsupported planning algorithm: %s. Use "plannerRRT", "plannerRRTStar", "plannerAStar", "plannerPRM", "distanceTransform", or "plannerBug2".',planningAlgorithm);
end
planningTime = toc(tPlanStart);
fprintf('[INFO] Planning time: %.3f s\n', planningTime);

function [bestPath,bestInfo] = runRRTStarBestOfN(planner,startPose,goalPose,sv,numRuns,seedBase,shortcutTrials)
bestPath = navPath(planner.StateSpace);
bestInfo = struct();
bestLen = inf;

for runIdx = 1:numRuns
    rng(seedBase + runIdx - 1);
    [candidatePath,candidateInfo] = plan(planner,startPose,goalPose);

    if candidatePath.NumStates < 1
        fprintf('[WARN] plannerRRTStar run %d/%d: no path found.\n', runIdx, numRuns);
        continue;
    end

    candidateStates = shortcutPath(candidatePath.States,sv,shortcutTrials);
    candidatePath = navPath(planner.StateSpace,candidateStates);
    candidateLen = candidatePath.pathLength;

    fprintf('[INFO] plannerRRTStar run %d/%d: path length %.2f m\n', runIdx, numRuns, candidateLen);

    if candidateLen < bestLen
        bestLen = candidateLen;
        bestPath = candidatePath;
        bestInfo = candidateInfo;
    end
end
end

function statesOut = shortcutPath(statesIn,sv,numTrials)
statesOut = statesIn;
if size(statesOut,1) < 3
    return;
end

for k = 1:numTrials
    n = size(statesOut,1);
    if n < 3
        break;
    end

    i = randi([1, n-2]);
    j = randi([i+2, n]);

    [isValid,~] = isMotionValid(sv,statesOut(i,:),statesOut(j,:));
    if isValid
        statesOut = [statesOut(1:i,:); statesOut(j:end,:)];
    end
end
end

%% Plan a path and visualize it
if isempty(plannedStates)
    error('No path found for selected algorithm: %s',planningAlgorithm);
end
fprintf('[INFO] Planning complete. Starting visualization and closed-loop path following...\n');

pathLength = sum(vecnorm(diff(plannedStates(:,1:2),1,1),2,2));

if contains(lower(planningAlgorithm), "distancetransform")
    % Special Distance Transform visualization matching the exact dimensions of standard map
    viz(startPose) % Initialize robot pose visualizer showing standard map to capture dimensions
    
    ax = gca;
    hold(ax, 'on');
    
    % Delete the original map image so we can replace its background with uninflated sharp black walls 
    h_img_orig = findobj(ax, 'Type', 'Image');
    delete(h_img_orig);
    
    % Load uninflated map with resolution 10 for drawing background
    mapTemp = load('complexMap');
    scaleFactor = map.Resolution / mapTemp.map.Resolution;
    occ_uninflated = imresize(getOccupancy(mapTemp.map), scaleFactor, "nearest");
    
    % Compute continuous geodesic distance transform on the uninflated map for pure display
    goal_grid_uninflated = world2grid(occupancyMap(occ_uninflated, map.Resolution), [goalPose(1) goalPose(2)]);
    D_display = bwdistgeodesic(occ_uninflated < 0.5, goal_grid_uninflated(2), goal_grid_uninflated(1), 'quasi-euclidean');
    
    % Normalize distance field for coloring [0, 1] range
    D_min = min(D_display(isfinite(D_display)));
    D_max = max(D_display(isfinite(D_display)));
    D_norm = (D_display - D_min) / (D_max - D_min);
    
    % Directly generate RGB CData to render:
    % - Sharp black walls [0 0 0] (no bloating or black inflate-extensions)
    % - Beautiful, continuous green gradient over uninflated free space (dark green near goal -> light/bright green far from goal)
    [rows, cols] = size(D_norm);
    CData = zeros(rows, cols, 3); % Pure black [0 0 0] for walls
    is_free = (occ_uninflated < 0.5) & isfinite(D_display);
    
    CData_R = zeros(rows, cols);
    CData_G = zeros(rows, cols);
    CData_B = zeros(rows, cols);
    
    CData_R(is_free) = 0.0 + 0.3 * D_norm(is_free);
    CData_G(is_free) = 0.2 + 0.8 * D_norm(is_free);
    CData_B(is_free) = 0.0 + 0.3 * D_norm(is_free);
    
    CData(:,:,1) = CData_R;
    CData(:,:,2) = CData_G;
    CData(:,:,3) = CData_B;
    
    x_centers = (1:cols)/map.Resolution - 0.5/map.Resolution;
    y_centers_inc = flipud(map.YWorldLimits(2) - ((1:rows)'/map.Resolution - 0.5/map.Resolution));
    
    % Correctly flip the direct RGB image vertically (flipud) so that the background and walls align 100% perfectly with world coordinates and robot path!
    h_img = imagesc(ax, x_centers, y_centers_inc, flipud(CData));
    uistack(h_img, 'bottom');
    
    % Set axis background of empty spaces outside the map limits to white [1 1 1], exactly like other planners!
    set(ax, 'Color', 'w');
    
    % Draw arrows indicating the gradient descent direction if showArrows is enabled
    if exist('showArrows', 'var') && showArrows
        try
            % We draw arrows pointing toward smaller distance values (which is gradient descent)
            % Compute the gradient of the display distance
            % Since distance transform grows OUTWARD from the goal, the negative gradient (-DX, DY) 
            % points toward the goal, which is the direction of the geodesic descent.
            % Note: Because matrix row index increases downwards but world Y-axis increases upwards,
            % the vertical component of the descent direction in world coordinates is DY.
            [DX, DY] = gradient(D_display);
            DX = -DX; 
            % DY remains positive (DY) because world Y and matrix row index are inversely oriented,
            % so a step towards smaller row indices (up) corresponds to positive world Y velocity.
            
            % Normalize the gradient vector fields to unit length for pretty uniform arrows
            grad_norm = sqrt(DX.^2 + DY.^2);
            grad_norm(grad_norm == 0) = 1; % Avoid division by zero
            U = DX ./ grad_norm;
            V = DY ./ grad_norm;
            
            % Downsample the grid to avoid cluttering the plot with too many arrows
            % With a resolution of 10 cells/m, a step of 4 cells (40cm/arrow) or 5 cells is gorgeous
            step_idx = 4;
            [XG, YG] = meshgrid(x_centers, flipud(y_centers_inc));
            
            X_sub = XG(1:step_idx:end, 1:step_idx:end);
            Y_sub = YG(1:step_idx:end, 1:step_idx:end);
            U_sub = U(1:step_idx:end, 1:step_idx:end);
            V_sub = V(1:step_idx:end, 1:step_idx:end);
            is_free_sub = is_free(1:step_idx:end, 1:step_idx:end);
            
            % Only show arrows on valid free space where distance is finite
            X_plot = X_sub(is_free_sub);
            Y_plot = Y_sub(is_free_sub);
            U_plot = U_sub(is_free_sub);
            V_plot = V_sub(is_free_sub);
            
            % Plot arrows in black
            h_q = quiver(ax, X_plot, Y_plot, U_plot, V_plot, 0.4, 'k', 'LineWidth', 0.5);
            if ~isempty(h_q)
                h_q.Annotation.LegendInformation.IconDisplayStyle = 'off';
            end
        catch ME
            fprintf('[WARN] Fehler beim Zeichnen der Gradienten-Pfeile: %s\n', ME.message);
        end
    end
    
    % Set the axes colormap and colorbar so that it shows the corresponding green gradient and correct distances in cells
    greenMap = [linspace(0.0, 0.3, 256)', linspace(0.2, 1.0, 256)', linspace(0.0, 0.3, 256)'];
    colormap(ax, greenMap);
    cb = colorbar(ax);
    ylabel(cb, 'Distance to goal (cells)');
    clim(ax, [D_min D_max]);
    
    % Draw Start and Goal markers (roter Kreis und rotes Kreuz, wie bei den anderen)
    plot(ax, startPose(1), startPose(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Start');
    plot(ax, goalPose(1), goalPose(2), 'rx', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Ziel');
    
    % Draw the planned path (rot gestrichelt, wie bei den anderen)
    plot(ax, plannedStates(:,1), plannedStates(:,2), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Distance Transform path');
    
    title(ax, sprintf('Path Planning Algorithm: %s | Path Length: %.2f m | Comp Time: %.3f s', ...
        planningAlgorithm, pathLength, planningTime));
    hold(ax, 'off');
else
    load complexMap % Reload map without inflation for display
    viz(startPose)
    visualizePlannedPath(plannedStates,solInfo,ss,startPose,goalPose,planningAlgorithm,pathLength,planningTime); % Helper function to visualize
end

%% Define Pure Pursuit controller for path following
waypoints = plannedStates(:,1:2);
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.25;
controller.DesiredLinearVelocity = 1;
controller.MaxAngularVelocity = 3;
rotateInPlaceThreshold = deg2rad(30);
headingGain = 2.5;
terminalControlDistance = 1.0;
goalLinearGain = 0.8;
goalPositionTolerance = 0.15;
goalHeadingTolerance = deg2rad(8);

%% Path following simulation loop
r = rateControl(5/sampleTime); % Run at 5x speed
pose = zeros(3,numel(tVec));
pose(:,1) = startPose;
idx = 2;
dist = Inf;
goalReached = false;
fprintf('[INFO] Simulation started.\n');
while (idx < numel(tVec)) && ~goalReached
    % Run the Pure Pursuit controller and apply v/w directly to the
    % differential-drive model.
    [vRef,wRef] = controller(pose(:,idx-1));

    goalDistNow = norm(pose(1:2,idx-1) - goalPose(1:2)');
    goalHeadingError = atan2(sin(goalPose(3)-pose(3,idx-1)),cos(goalPose(3)-pose(3,idx-1)));

    % Near goal, switch from path tracking to terminal goal regulation.
    if goalDistNow <= terminalControlDistance
        targetHeadingGoal = atan2(goalPose(2)-pose(2,idx-1),goalPose(1)-pose(1,idx-1));
        headingToGoalError = atan2(sin(targetHeadingGoal-pose(3,idx-1)),cos(targetHeadingGoal-pose(3,idx-1)));

        if goalDistNow <= goalPositionTolerance
            vCmd = 0;
            if abs(goalHeadingError) > goalHeadingTolerance
                wCmd = sign(goalHeadingError) * min(controller.MaxAngularVelocity,headingGain*abs(goalHeadingError));
            else
                wCmd = 0;
                goalReached = true;
            end
        elseif abs(headingToGoalError) > rotateInPlaceThreshold
            vCmd = 0;
            wCmd = sign(headingToGoalError) * min(controller.MaxAngularVelocity,headingGain*abs(headingToGoalError));
        else
            vCmd = min(controller.DesiredLinearVelocity,goalLinearGain*goalDistNow);
            wCmd = sign(headingToGoalError) * min(controller.MaxAngularVelocity,headingGain*abs(headingToGoalError));
        end
    else
        % Path following stage - apply selected overshoot reduction mode
        if ~exist('overshootReductionMode', 'var')
            overshootReductionMode = 0; % Default to pure pursuit
        end
        
        switch overshootReductionMode
            case 1
                % Mode 1: Rotate in Place. If heading error to path target is too large,
                % stop forward movement and spin on the spot.
                if vRef > 0
                    % Estimate path target heading from Pure Pursuit output:
                    % wRef = vRef * curvature => curvature = wRef / vRef
                    % heading_error = atan(curvature * lookahead)
                    targetHeadingPath = pose(3,idx-1) + atan(wRef * controller.LookaheadDistance / vRef);
                    headingToPathError = atan2(sin(targetHeadingPath-pose(3,idx-1)), cos(targetHeadingPath-pose(3,idx-1)));
                    
                    if abs(headingToPathError) > rotateInPlaceThreshold
                        vCmd = 0;
                        wCmd = sign(headingToPathError) * min(controller.MaxAngularVelocity, headingGain * abs(headingToPathError));
                    else
                        vCmd = vRef;
                        wCmd = wRef;
                    end
                else
                    vCmd = vRef;
                    wCmd = wRef;
                end
                
            case 2
                % Mode 2: Dynamic Speed Scaling. Scale down linear velocity
                % exponentially when making sharp turns (high steering rate wRef).
                vCmd = vRef * exp(-1.5 * abs(wRef)); 
                wCmd = wRef;
                
            otherwise
                % Mode 0: Standard Pure Pursuit
                vCmd = vRef;
                wCmd = wRef;
        end
    end

    vel = derivative(vehicle,pose(:,idx-1),[vCmd wCmd]);
    
    % Perform forward discrete integration step
    pose(:,idx) = pose(:,idx-1) + vel*sampleTime;
    
    % Update visualization
    viz(pose(:,idx))
    
    % Calculate distance to goal and update loop
    dist = norm( pose(1:2,idx) - goalPose(1:2)' );
    if dist <= goalPositionTolerance
        headingErrFinal = atan2(sin(goalPose(3)-pose(3,idx)),cos(goalPose(3)-pose(3,idx)));
        if abs(headingErrFinal) <= goalHeadingTolerance
            goalReached = true;
        end
    end
    idx = idx+1;
    waitfor(r);
end

travelTime = (idx-1)*sampleTime;
fprintf('[INFO] Simulation finished. Distance to goal: %.3f m, Travel Time: %.2f s\n', dist, travelTime);
title(sprintf('Path Planning Algorithm: %s | Path Length: %.2f m | Comp Time: %.2f s | Travel Time: %.2f s', ...
    planningAlgorithm,pathLength,planningTime,travelTime));

%% Helper function to visualize path
function visualizePlannedPath(plannedStates,solInfo,ss,startPose,goalPose,planningAlgorithm,pathLength,planningTime)
hold on
% Plot roadmap in the background (green nodes and connections) for plannerPRM / plannerPRMsmooth
if isfield(solInfo, 'Planner') && isa(solInfo.Planner, 'plannerPRM')
    try
        gd = graphData(solInfo.Planner);
        node_states = gd.Nodes.StateVector;
        edges_nodes = gd.Edges.EndNodes;
        
        % Draw connections/edges in green
        x_line = [node_states(edges_nodes(:,1), 1)'; node_states(edges_nodes(:,2), 1)'];
        y_line = [node_states(edges_nodes(:,1), 2)'; node_states(edges_nodes(:,2), 2)'];
        h_edges = plot(x_line, y_line, 'Color', [0.4 0.75 0.5], 'LineWidth', 0.5);
        
        % Draw nodes/vertices in dark green
        h_nodes = plot(node_states(:,1), node_states(:,2), '.', 'Color', [0.1 0.45 0.2], 'MarkerSize', 6);
        
        % Exclude from legend to keep clean
        if ~isempty(h_edges)
            h_edges(1).Annotation.LegendInformation.IconDisplayStyle = 'off';
        end
        if ~isempty(h_nodes)
            h_nodes(1).Annotation.LegendInformation.IconDisplayStyle = 'off';
        end
    catch ME
        fprintf('[WARN] Fehler beim Zeichnen des PRM Roadmaps: %s\n', ME.message);
    end
end

% Plot RRT tree in the background (green nodes and connections) for plannerRRT / plannerRRTStar (geometric/non-Dubins cases)
if (contains(lower(planningAlgorithm), "rrt") || contains(lower(planningAlgorithm), "rrtstar")) && ...
   ~contains(lower(planningAlgorithm), "dubins") && isfield(solInfo, 'TreeData') && ~isempty(solInfo.TreeData)
    try
        tData = solInfo.TreeData;
        % TreeData structured as: row 3N-2=parent, 3N-1=child, 3N=NaN spacer
        % Extract non-NaN rows as unique nodes
        nodes = tData(~isnan(tData(:,1)), :);
        % Since each connected segment is parent -> child, we plot them in green
        x_tree = [tData(1:3:end-2, 1)'; tData(2:3:end-1, 1)'];
        y_tree = [tData(1:3:end-2, 2)'; tData(2:3:end-1, 2)'];
        
        h_edges = plot(x_tree, y_tree, 'Color', [0.4 0.75 0.5], 'LineWidth', 0.5);
        h_nodes = plot(nodes(:,1), nodes(:,2), '.', 'Color', [0.1 0.45 0.2], 'MarkerSize', 6);
        
        if ~isempty(h_edges)
            h_edges(1).Annotation.LegendInformation.IconDisplayStyle = 'off';
        end
        if ~isempty(h_nodes)
            h_nodes(1).Annotation.LegendInformation.IconDisplayStyle = 'off';
        end
    catch ME
        fprintf('[WARN] Fehler beim Zeichnen des RRT-Baums: %s\n', ME.message);
    end
end

% Plot mline (magenta dashed) as a reference line from start to goal - only for Bug2
if contains(lower(planningAlgorithm), "bug2")
    plot([startPose(1); goalPose(1)], [startPose(2); goalPose(2)], 'm--', 'LineWidth', 1.2, 'DisplayName', 'M-Linie');
end
% Plot the path from start to goal
plot(plannedStates(:,1),plannedStates(:,2),'r--','LineWidth',1.5);
plot(startPose(1),startPose(2),'ro','LineWidth',2);
plot(goalPose(1),goalPose(2),'rx','LineWidth',2);
% Interpolate each path segment to be smoother and plot it
if isfield(solInfo,'TreeData') && ~isempty(solInfo.TreeData)
    tData = solInfo.TreeData;
    for idx = 3:3:size(tData,1)-2
        p = navPath(ss,tData(idx:idx+1,:));
        interpolate(p,10);
        plot(p.States(:,1),p.States(:,2),':','Color',[0 0.6 0.9]);
    end
end
title(sprintf('Path Planning Algorithm: %s | Path Length: %.2f m | Comp Time: %.2f s | Travel Time: n/a', ...
    planningAlgorithm,pathLength,planningTime));
hold off
end

function idx = nearestPoseIndex(stateList,queryPose)
% Select nearest node in (x,y,theta) space with wrapped heading error.
xyErr = stateList(:,1:2) - queryPose(1:2);
thErr = atan2(sin(stateList(:,3)-queryPose(3)),cos(stateList(:,3)-queryPose(3)));
metric = sum(xyErr.^2,2) + 0.1*(thErr.^2);
[~,idx] = min(metric);
end

function idxList = nearestPoseIndices(stateList,queryPose,k)
% Return k nearest node indices in (x,y,theta) space.
xyErr = stateList(:,1:2) - queryPose(1:2);
thErr = atan2(sin(stateList(:,3)-queryPose(3)),cos(stateList(:,3)-queryPose(3)));
metric = sum(xyErr.^2,2) + 0.1*(thErr.^2);
[~,sortedIdx] = sort(metric,'ascend');
kUse = min(k,numel(sortedIdx));
idxList = sortedIdx(1:kUse);
end

function [planner,plannedPath,solInfo] = planWithPRMRetries(ss,sv,startPose,goalPose,baseNumNodes,baseConnDistance)
% Retry PRM planning with larger roadmaps if a path is not found.
if isscalar(baseNumNodes) && isscalar(baseConnDistance)
    numNodeTrials = [baseNumNodes, max(baseNumNodes,2000), max(baseNumNodes,3500)];
    connTrials = [baseConnDistance, max(baseConnDistance,4.0), max(baseConnDistance,6.0)];
else
    numNodeTrials = baseNumNodes(:)';
    connTrials = baseConnDistance(:)';
    if numel(numNodeTrials) ~= numel(connTrials)
        error('PRM trial configuration mismatch: node and connection trial arrays must have equal length.');
    end
end

for trialIdx = 1:numel(numNodeTrials)
    fprintf('[INFO] PRM trial %d/%d: MaxNumNodes=%d, MaxConnectionDistance=%.2f\n', ...
        trialIdx,numel(numNodeTrials),numNodeTrials(trialIdx),connTrials(trialIdx));
    planner = plannerPRM(ss,sv, ...
        MaxNumNodes=numNodeTrials(trialIdx), ...
        MaxConnectionDistance=connTrials(trialIdx));

    [plannedPath,solInfo] = plan(planner,startPose,goalPose);
    if plannedPath.NumStates > 0
        fprintf('[INFO] PRM trial %d succeeded with %d states.\n', trialIdx, plannedPath.NumStates);
        return;
    end
    fprintf('[WARN] PRM trial %d failed to find a path.\n', trialIdx);
end
end