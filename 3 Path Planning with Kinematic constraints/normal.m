clear;
close all;

% Map Setup
mapSize = [50 50]; resolution = 10; binaryMap = zeros(mapSize);
start = [0.5 0.5 0];  % [x, y, theta]
goal = [4.5 4.5 0];
shape1 = polyshape([15 20 25 20 15], [10 15 20 25 20]);
shape2 = polyshape([30 40 35], [30 30 40]);
shape3 = polyshape([25 30 35 30 25 20], [35 40 35 30 25 30]);
[X, Y] = meshgrid(1:mapSize(1), 1:mapSize(2)); points = [X(:) Y(:)];
binaryMap(isinterior(shape1, points)) = 1;
binaryMap(isinterior(shape2, points)) = 1;
binaryMap(isinterior(shape3, points)) = 1;
map = occupancyMap(binaryMap, resolution);

% Kinematic RRT with plannerControlRRT
propagator = mobileRobotPropagator('Environment', map); % Bicycle model by default
propagator.SystemParameters.KinematicModel.WheelBase = 1;
propagator.StateSpace.StateBounds(1:2,:) = [map.XWorldLimits; map.YWorldLimits];

% MaxNumTreeNodes: Maximum number of nodes in the search tree, default = 1e4
plannerKin = plannerControlRRT(propagator,'MaxNumIteration', 10000);

% Plan Kinematic Path
rng("default");
[pthObjKin, solnInfoKin] = plan(plannerKin, start, goal);

% Standard RRT with plannerRRT
ssRRT = stateSpaceSE2;
ssRRT.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
svRRT = validatorOccupancyMap(ssRRT, 'Map', map);
svRRT.ValidationDistance = 0.01;
plannerStd = plannerRRT(ssRRT, svRRT, ...
    'MaxIterations', 10000, ...
    'MaxConnectionDistance', 0.2);

% Plan Standard Path
rng("default");
[pthObjStd, solnInfoStd] = plan(plannerStd, start, goal);

% Compute Metrics
% Kinematic RRT
if solnInfoKin.IsPathFound
    kinCost = sum(vecnorm(diff(pthObjKin.States(:,1:2)), 2, 2));
    kinNodes = solnInfoKin.NumTreeNode; % Correct field name
    kinIterations = solnInfoKin.NumIteration; % Correct field name
else
    kinCost = NaN;
    kinNodes = solnInfoKin.NumTreeNode;
    kinIterations = solnInfoKin.NumIteration;
end

% Standard RRT
if solnInfoStd.IsPathFound
    stdCost = sum(vecnorm(diff(pthObjStd.States(:,1:2)), 2, 2));
    stdNodes = solnInfoStd.NumNodes; % plannerRRT uses NumNodes
    stdIterations = solnInfoStd.NumIterations; % plannerRRT uses NumIterations
else
    stdCost = NaN;
    stdNodes = solnInfoStd.NumNodes;
    stdIterations = solnInfoStd.NumIterations;
end

% Plotting
figure('Position', [100 100 1000 400]);

% Standard RRT
subplot(1, 2, 1);
show(map); hold on;
plot(start(1), start(2), 'rx', 'LineWidth', 3);
plot(goal(1), goal(2), 'go', 'LineWidth', 3);
if solnInfoStd.IsPathFound
    plot(solnInfoStd.TreeData(:,1), solnInfoStd.TreeData(:,2), '.-', 'Color', [0.5 0.5 0.5]);
    plot(pthObjStd.States(:,1), pthObjStd.States(:,2), 'r-', 'LineWidth', 2);
    title(sprintf('Standard RRT\nCost: %.2f m, Nodes: %d, Iterations: %d', ...
        stdCost, stdNodes, stdIterations));
else
    title(sprintf('Standard RRT - Path not found\nNodes: %d, Iterations: %d', ...
        stdNodes, stdIterations));
end
hold off;

% Kinematic RRT
subplot(1, 2, 2);
show(map); hold on;
plot(start(1), start(2), 'rx', 'LineWidth', 3);
plot(goal(1), goal(2), 'go', 'LineWidth', 3);
if solnInfoKin.IsPathFound
    TreeInfo = solnInfoKin.TreeInfo';
    plot(TreeInfo(:,1), TreeInfo(:,2), '.-', 'Color', [0.5 0.5 0.5]);
    plot(pthObjKin.States(:,1), pthObjKin.States(:,2), 'r-', 'LineWidth', 2);
    title(sprintf('Kinematic RRT\nCost: %.2f m, Nodes: %d, Iterations: %d', ...
        kinCost, kinNodes, kinIterations));
else
    title(sprintf('Kinematic RRT - Path not found\nNodes: %d, Iterations: %d', ...
        kinNodes, kinIterations));
end
hold off;