clear

% Map setup
mapSize = [50 50]; % Grid size (50x50 cells)
resolution = 10;   % 10 cells per meter
binaryMap = zeros(mapSize);

start = [0.5 0.5 0];
goal = [4.9 4.9 0];

% Create numerous polygonal obstacles for maximum clutter
shape1x = [5 10 12 7];          % Trapezoid near start
shape1y = [2 2 7 7];
shape1 = polyshape(shape1x, shape1y);

shape2x = [15 20 20 15];        % Square in lower-middle
shape2y = [5 5 10 10];
shape2 = polyshape(shape2x, shape2y);

shape3x = [25 30 32 27];        % Trapezoid in lower-middle
shape3y = [2 2 8 8];
shape3 = polyshape(shape3x, shape3y);

shape4x = [35 40 42 37];        % Trapezoid in lower-right
shape4y = [5 5 10 10];
shape4 = polyshape(shape4x, shape4y);

shape5x = [45 48 48 45];        % Square near goal (lower-right)
shape5y = [2 2 7 7];
shape5 = polyshape(shape5x, shape5y);

shape6x = [10 15 17 12];        % Trapezoid in lower-middle
shape6y = [10 10 15 15];
shape6 = polyshape(shape6x, shape6y);

shape7x = [20 25 27 22];        % Trapezoid in lower-middle
shape7y = [12 12 17 17];
shape7 = polyshape(shape7x, shape7y);

shape8x = [30 35 37 32];        % Trapezoid in lower-right
shape8y = [10 10 15 15];
shape8 = polyshape(shape8x, shape8y);

shape9x = [40 45 47 42];        % Trapezoid in lower-right
shape9y = [12 12 17 17];
shape9 = polyshape(shape9x, shape9y);

shape10x = [5 10 10 5];         % Square in upper-left
shape10y = [35 35 40 40];
shape10 = polyshape(shape10x, shape10y);

shape11x = [15 20 22 17];       % Trapezoid in upper-middle
shape11y = [30 30 35 35];
shape11 = polyshape(shape11x, shape11y);

shape12x = [25 30 32 27];       % Trapezoid in upper-middle
shape12y = [35 35 40 40];
shape12 = polyshape(shape12x, shape12y);

shape13x = [35 40 42 37];       % Trapezoid in upper-right
shape13y = [30 30 35 35];
shape13 = polyshape(shape13x, shape13y);

shape14x = [45 48 48 45];       % Square near goal (upper-right)
shape14y = [35 35 40 40];
shape14 = polyshape(shape14x, shape14y);

shape15x = [10 15 17 12];       % Trapezoid in middle-left
shape15y = [20 20 25 25];
shape15 = polyshape(shape15x, shape15y);

shape16x = [20 25 27 22];       % Trapezoid in center
shape16y = [22 22 27 27];
shape16 = polyshape(shape16x, shape16y);

shape17x = [30 35 37 32];       % Trapezoid in middle-right
shape17y = [20 20 25 25];
shape17 = polyshape(shape17x, shape17y);

shape18x = [40 45 47 42];       % Trapezoid in middle-right
shape18y = [22 22 27 27];
shape18 = polyshape(shape18x, shape18y);

shape19x = [5 10 12 7];         % Trapezoid in upper-left
shape19y = [42 42 47 47];
shape19 = polyshape(shape19x, shape19y);

shape20x = [15 20 22 17];       % Trapezoid in upper-middle
shape20y = [40 40 45 45];
shape20 = polyshape(shape20x, shape20y);

shape21x = [25 30 32 27];       % Trapezoid in upper-middle
shape21y = [42 42 47 47];
shape21 = polyshape(shape21x, shape21y);

shape22x = [35 40 42 37];       % Trapezoid in upper-right
shape22y = [40 40 45 45];
shape22 = polyshape(shape22x, shape22y);

shape23x = [45 48 48 45];       % Square near goal (upper-right)
shape23y = [42 42 47 47];
shape23 = polyshape(shape23x, shape23y);

% Convert polygons to binary map
[X, Y] = meshgrid(1:mapSize(1), 1:mapSize(2));
points = [X(:) Y(:)];

% Check interior points for all shapes
inShape1 = isinterior(shape1, points);
inShape2 = isinterior(shape2, points);
inShape3 = isinterior(shape3, points);
inShape4 = isinterior(shape4, points);
inShape5 = isinterior(shape5, points);
inShape6 = isinterior(shape6, points);
inShape7 = isinterior(shape7, points);
inShape8 = isinterior(shape8, points);
inShape9 = isinterior(shape9, points);
inShape10 = isinterior(shape10, points);
inShape11 = isinterior(shape11, points);
inShape12 = isinterior(shape12, points);
inShape13 = isinterior(shape13, points);
inShape14 = isinterior(shape14, points);
inShape15 = isinterior(shape15, points);
inShape16 = isinterior(shape16, points);
inShape17 = isinterior(shape17, points);
inShape18 = isinterior(shape18, points);
inShape19 = isinterior(shape19, points);
inShape20 = isinterior(shape20, points);
inShape21 = isinterior(shape21, points);
inShape22 = isinterior(shape22, points);
inShape23 = isinterior(shape23, points);

% Set occupied cells
binaryMap(inShape1) = 1;
binaryMap(inShape2) = 1;
binaryMap(inShape3) = 1;
binaryMap(inShape4) = 1;
binaryMap(inShape5) = 1;
binaryMap(inShape6) = 1;
binaryMap(inShape7) = 1;
binaryMap(inShape8) = 1;
binaryMap(inShape9) = 1;
binaryMap(inShape10) = 1;
binaryMap(inShape11) = 1;
binaryMap(inShape12) = 1;
binaryMap(inShape13) = 1;
binaryMap(inShape14) = 1;
binaryMap(inShape15) = 1;
binaryMap(inShape16) = 1;
binaryMap(inShape17) = 1;
binaryMap(inShape18) = 1;
binaryMap(inShape19) = 1;
binaryMap(inShape20) = 1;
binaryMap(inShape21) = 1;
binaryMap(inShape22) = 1;
binaryMap(inShape23) = 1;

% Create the occupancy map object
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
    kinNodes = solnInfoKin.NumTreeNode;
    kinIterations = solnInfoKin.NumIteration;
else
    kinCost = NaN;
    kinNodes = solnInfoKin.NumTreeNode;
    kinIterations = solnInfoKin.NumIteration;
end

% Standard RRT
if solnInfoStd.IsPathFound
    stdCost = sum(vecnorm(diff(pthObjStd.States(:,1:2)), 2, 2));
    stdNodes = solnInfoStd.NumNodes;
    stdIterations = solnInfoStd.NumIterations; 
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