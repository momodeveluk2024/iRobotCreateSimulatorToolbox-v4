function result = simulateTask1Trial(mapFile, startPose, duration, dt)
% simulateTask1Trial Fast non-GUI trial for task1ControlStep on one map.

if nargin < 2 || isempty(startPose)
    startPose = [0 0 0];
end
if nargin < 3 || isempty(duration)
    duration = 45;
end
if nargin < 4 || isempty(dt)
    dt = 0.12;
end

[walls, lines, beacons, virtualWalls] = parseSimulatorMapFile(mapFile);
robot = CreateRobot();
setMap(robot, walls, lines, beacons, virtualWalls);
setMapStart(robot, startPose);
setAutoEnable(robot, true);
startTimeElap(robot);
updateOutput(robot);

state = struct;
steps = max(1, round(duration / dt));
collisionSteps = 0;
bumpSteps = 0;
distanceTravelled = 0;
previousState = getState(robot);
minFront = inf;

for k = 1:steps
    sonar = genSonar(robot); % [front left rear right]
    lidarScan = genLidar(robot);
    bump = genBump(robot);   % [right front left]
    minFront = min(minFront, sonar(1));

    if any(bump)
        bumpSteps = bumpSteps + 1;
    end

    [forwardVel, angularVel, state] = task1ControlStep( ...
        lidarScan, sonar(1), sonar(4), sonar(2), bump, state, dt);
    SetFwdVelAngVelCreate(robot, forwardVel, angularVel);

    collPts = findCollisions(robot);
    if isempty(collPts)
        driveNormal(robot, dt);
    elseif size(collPts, 1) == 1
        if ~collPts(4)
            drive1Wall(robot, dt, collPts);
        else
            driveCorner(robot, dt, collPts);
        end
        collisionSteps = collisionSteps + 1;
    else
        if ~any(collPts(:,4))
            drive2Wall(robot, dt, collPts);
        elseif xor(collPts(1,4), collPts(2,4))
            collPts = collPts(~collPts(:,4), :);
            drive1Wall(robot, dt, collPts);
        else
            collPts = collPts(1, :);
            driveCorner(robot, dt, collPts);
        end
        collisionSteps = collisionSteps + 1;
    end

    currentState = getState(robot);
    distanceTravelled = distanceTravelled + hypot( ...
        currentState(1) - previousState(1), currentState(2) - previousState(2));
    previousState = currentState;
end

finalState = getState(robot);
result = struct( ...
    'mapFile', mapFile, ...
    'startPose', startPose, ...
    'duration', duration, ...
    'dt', dt, ...
    'distanceTravelled', distanceTravelled, ...
    'collisionSteps', collisionSteps, ...
    'bumpSteps', bumpSteps, ...
    'minFrontSonar', minFront, ...
    'finalState', finalState);
end
