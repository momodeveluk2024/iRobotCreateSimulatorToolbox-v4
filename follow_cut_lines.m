function [waypoints, sim] = follow_cut_lines(mapFile)
% FOLLOW_CUT_LINES  Drive a bot along the cut lines, not the walls.
%
% Usage:
%   follow_cut_lines
%   [waypoints, sim] = follow_cut_lines
%   [waypoints, sim] = follow_cut_lines('my_map.txt')
%
% What this file does:
%   1. Reads only map entries that start with: line x1 y1 x2 y2
%   2. Ignores all map entries that start with: wall x1 y1 x2 y2
%   3. Turns the cut-line segments into ordered waypoints.
%   4. Simulates a simple differential-drive bot following those waypoints.
%
% To run on a real bot, replace sendVelocityCommand(v,w) at the bottom of
% this file with your robot's velocity-command function and set
% SIMULATE_ONLY = false below.

SIMULATE_ONLY = true;     % Keep true for plotting/simulation only.
SHOW_PLOT     = true;

% Controller parameters. Tune these for your bot if needed.
params.dt          = 0.05;   % seconds
params.maxV        = 0.35;   % forward speed in map units/second
params.maxW        = 2.20;   % angular speed in rad/second
params.kHeading    = 2.00;   % heading gain
params.goalTol     = 0.08;   % waypoint hit radius
params.maxRunTime  = 600;    % seconds
params.connectTol  = 0.15;   % gap tolerance between cut-line segments

% If no external map file is supplied, use the map from the prompt.
if nargin < 1 || isempty(mapFile)
    mapText = defaultMapText();
else
    mapText = fileread(mapFile);
end

[walls, cutSegments] = readMapText(mapText);

if isempty(cutSegments)
    error('No cut lines were found. The map needs entries like: line x1 y1 x2 y2');
end

% This creates a path from the line entries only. Wall entries are not used.
[waypoints, connectionInfo] = cutSegmentsToWaypoints(cutSegments, params.connectTol);

% Start at the first cut-line waypoint and face toward the second waypoint.
if size(waypoints, 1) >= 2
    theta0 = atan2(waypoints(2,2) - waypoints(1,2), waypoints(2,1) - waypoints(1,1));
else
    theta0 = 0;
end
startPose = [waypoints(1,1), waypoints(1,2), theta0];

sim = followWaypoints(waypoints, startPose, params, SIMULATE_ONLY);
sim.walls = walls;
sim.cutSegments = cutSegments;
sim.connectionInfo = connectionInfo;
sim.params = params;

if SHOW_PLOT
    plotCutLineRun(walls, cutSegments, waypoints, sim, connectionInfo);
end

fprintf('Loaded %d wall segments, ignored them for driving.\n', size(walls,1));
fprintf('Loaded %d cut-line segments and generated %d waypoints.\n', size(cutSegments,1), size(waypoints,1));
fprintf('Final target reached: %d\n', sim.reachedGoal);

end

function mapText = defaultMapText()
% The map supplied in the prompt. The controller uses only the "line" rows.
mapText = sprintf([ ...
'wall -4.888 -0.990 3.216 -1.010\n' ...
'wall -1.974 -1.010 -2.009 2.970\n' ...
'wall 1.026 -0.990 1.009 2.990\n' ...
'wall 3.216 -1.010 3.181 2.970\n' ...
'line -4.821 0.017 -3.191 -0.009\n' ...
'line -3.168 4.037 -1.287 3.931\n' ...
'line -1.287 3.931 -1.195 -0.089\n' ...
'line -1.195 -0.089 0.205 -0.089\n' ...
'line 0.205 -0.089 0.297 3.957\n' ...
'line 0.297 3.957 1.674 3.877\n' ...
'line 1.674 3.877 1.674 0.017\n' ...
'line 1.674 0.017 2.775 0.017\n' ...
'line 2.775 0.017 2.683 3.931\n' ...
'line 2.683 3.931 4.060 3.851\n' ...
'line 4.060 3.851 4.060 -2.032\n' ...
'line 4.060 -2.032 -4.867 -2.059\n']);
end

function [walls, cutSegments] = readMapText(mapText)
% Parse a map file and separate real walls from cut lines.
rows = regexp(mapText, '\r\n|\n|\r', 'split');
walls = zeros(0,4);
cutSegments = zeros(0,4);

for i = 1:numel(rows)
    row = strtrim(rows{i});
    if isempty(row)
        continue;
    end
    if row(1) == '%'
        continue;
    end

    parts = regexp(row, '\s+', 'split');
    key = lower(parts{1});
    values = sscanf(row(numel(parts{1})+1:end), '%f');

    if numel(values) < 4
        continue;
    end

    if strcmp(key, 'wall')
        walls(end+1,:) = values(1:4).'; %#ok<AGROW>
    elseif strcmp(key, 'line')
        cutSegments(end+1,:) = values(1:4).'; %#ok<AGROW>
    end
end
end

function [waypoints, connectionInfo] = cutSegmentsToWaypoints(cutSegments, connectTol)
% Convert cut-line segments to waypoints while preserving the file order.
% If two consecutive line segments have a gap, a connecting waypoint is kept
% so the bot drives continuously to the next cut line. This is useful for the
% supplied map because the left vertical cut-line connection is implied by
% the line order but not present as its own "line" row.
waypoints = zeros(0,2);
connectionInfo = struct('from', {}, 'to', {}, 'distance', {});

for i = 1:size(cutSegments,1)
    p1 = cutSegments(i,1:2);
    p2 = cutSegments(i,3:4);

    if isempty(waypoints)
        waypoints(end+1,:) = p1; %#ok<AGROW>
        waypoints(end+1,:) = p2; %#ok<AGROW>
        continue;
    end

    last = waypoints(end,:);
    dToP1 = norm(last - p1);
    dToP2 = norm(last - p2);

    % Pick the direction that makes this segment connect best to the
    % previous one.
    if dToP1 <= dToP2
        startPoint = p1;
        endPoint = p2;
        gap = dToP1;
    else
        startPoint = p2;
        endPoint = p1;
        gap = dToP2;
    end

    if gap > connectTol
        connectionInfo(end+1).from = last; %#ok<AGROW>
        connectionInfo(end).to = startPoint;
        connectionInfo(end).distance = gap;
        waypoints(end+1,:) = startPoint; %#ok<AGROW>
    end

    if norm(waypoints(end,:) - endPoint) > 1e-9
        waypoints(end+1,:) = endPoint; %#ok<AGROW>
    end
end
end

function sim = followWaypoints(waypoints, pose0, params, simulateOnly)
% Simple unicycle/differential-drive waypoint follower.
% pose = [x, y, theta]
pose = pose0;
targetIndex = 2;
maxSteps = ceil(params.maxRunTime / params.dt);
poseHistory = zeros(maxSteps, 3);
cmdHistory = zeros(maxSteps, 2);
reachedGoal = false;

for k = 1:maxSteps
    poseHistory(k,:) = pose;

    if targetIndex > size(waypoints,1)
        reachedGoal = true;
        cmdHistory(k,:) = [0, 0];
        break;
    end

    target = waypoints(targetIndex,:);
    dx = target(1) - pose(1);
    dy = target(2) - pose(2);
    distanceToTarget = hypot(dx, dy);

    if distanceToTarget < params.goalTol
        targetIndex = targetIndex + 1;
        cmdHistory(k,:) = [0, 0];
        continue;
    end

    desiredHeading = atan2(dy, dx);
    headingError = wrapAngle(desiredHeading - pose(3));

    % Slow down when not pointing toward the target, so the bot turns first.
    v = params.maxV * max(0, cos(headingError));
    w = clamp(params.kHeading * headingError, -params.maxW, params.maxW);

    cmdHistory(k,:) = [v, w];

    if ~simulateOnly
        sendVelocityCommand(v, w);
    end

    pose(1) = pose(1) + v * cos(pose(3)) * params.dt;
    pose(2) = pose(2) + v * sin(pose(3)) * params.dt;
    pose(3) = wrapAngle(pose(3) + w * params.dt);
end

% Trim unused preallocated rows.
lastRow = find(any(poseHistory ~= 0, 2), 1, 'last');
if isempty(lastRow)
    lastRow = 1;
end
poseHistory = poseHistory(1:lastRow,:);
cmdHistory = cmdHistory(1:lastRow,:);

sim.pose = poseHistory;
sim.commands = cmdHistory;
sim.reachedGoal = reachedGoal;
sim.lastTargetIndex = targetIndex;
end

function plotCutLineRun(walls, cutSegments, waypoints, sim, connectionInfo)
figure('Name', 'Bot following cut lines only');
hold on;
axis equal;
grid on;
xlabel('x');
ylabel('y');
title('Bot path follows line/cut entries; wall entries are ignored for path');

% Plot walls for reference only.
for i = 1:size(walls,1)
    plot(walls(i,[1 3]), walls(i,[2 4]), '-', 'LineWidth', 1.5, 'Color', [0.25 0.25 0.25]);
end

% Plot the cut-line segments from the map.
for i = 1:size(cutSegments,1)
    plot(cutSegments(i,[1 3]), cutSegments(i,[2 4]), 'k--', 'LineWidth', 1.0);
end

% Plot inserted connections, if any.
for i = 1:numel(connectionInfo)
    from = connectionInfo(i).from;
    to = connectionInfo(i).to;
    plot([from(1) to(1)], [from(2) to(2)], 'm:', 'LineWidth', 1.0);
end

plot(waypoints(:,1), waypoints(:,2), 'bo-', 'LineWidth', 1.0, 'MarkerSize', 4);
plot(sim.pose(:,1), sim.pose(:,2), 'r-', 'LineWidth', 1.5);
plot(waypoints(1,1), waypoints(1,2), 'go', 'MarkerSize', 8, 'LineWidth', 1.5);
plot(waypoints(end,1), waypoints(end,2), 'rx', 'MarkerSize', 8, 'LineWidth', 1.5);

legend('walls/reference only', 'cut lines', 'inserted cut-line connection', ...
       'waypoints', 'bot path', 'start', 'finish', 'Location', 'bestoutside');
hold off;
end

function a = wrapAngle(a)
% Wrap angle to [-pi, pi] without relying on toolboxes.
a = mod(a + pi, 2*pi) - pi;
end

function y = clamp(x, lo, hi)
y = min(max(x, lo), hi);
end

function sendVelocityCommand(v, w)
% Replace this no-op with your robot's command API.
% Example patterns:
%   bot.setVelocity(v, w)
%   robot.sendVelocity(v, w)
%   sim.setvel(v, w)
%
% v = forward velocity, w = angular velocity.
% This function is intentionally empty so the file runs as a simulation.
% Remove the next line after connecting it to your bot.
%#ok<INUSD>
end
