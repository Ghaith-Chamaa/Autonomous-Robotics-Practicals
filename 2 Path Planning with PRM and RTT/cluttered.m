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

% Create figure with two subplots side by side
figure('Position', [100 100 1000 400]);

% PRM Planner (Left subplot)
subplot(1,2,1);
% Defines a 2D euclidean environment
ssPRM = stateSpaceSE2;
ssPRM.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
% validates discretized states and movements based on the value in a 2D occupancy map. 
% An occupied location on the map is interpreted as an invalid state, this
% insures the path being built by PRM doesn't cross obstacles (obstacle detection)
svPRM = validatorOccupancyMap(ssPRM, Map=map);
% Sampling interval between states and checking state validity (not being on an obstacle)
svPRM.ValidationDistance = 0.01;

% if you want to use a sampler (other than the default stateSamplerUniform)
% in the SE(2) state space, the standard deviation must be a three-element row vector of the form [σx σy σθ] 
% MaxAttempts : the maximum number of attempts the sampler can make to find valid samples
% sampler = stateSamplerGaussian(svPRM,StandardDeviation=[5 5 0.05], MaxAttempts=20);
% plannerPRM = plannerPRM(ssPRM, svPRM, StateSampler=sampler, MaxNumNodes=200);

% the plannerPRM takes : 
% MaxConnectionDistance : Maximum connection distance between two states 
% (which is to say the radius around the current state to connect with)
% MaxNumNodes : Maximum number of nodes in the graph
plannerPRM = plannerPRM(ssPRM, svPRM, MaxNumNodes=200);

graph = graphData(plannerPRM);
edges = table2array(graph.Edges);
nodes = table2array(graph.Nodes);

% Plotting the grid with obstacles and the PRM graph (without the path)
show(svPRM.Map);
hold on;
plot(nodes(:,1), nodes(:,2), "*", "Color", "b", "LineWidth", 2);
for i = 1:size(edges,1)
    states = interpolate(ssPRM, nodes(edges(i,1),:), ...
                         nodes(edges(i,2),:), 0:0.02:1);
    plot(states(:,1), states(:,2), "Color", "b");
end
plot(start(1), start(2), "*", "Color", "g", "LineWidth", 3);
plot(goal(1), goal(2), "*", "Color", "r", "LineWidth", 3);

% Set the seed value of the random number to ensure repeatability.
rng(100, "twister");
[pthObjPRM, solnInfoPRM] = plan(plannerPRM, start, goal);


% Plotting the PRM path if found
if solnInfoPRM.IsPathFound
    interpolate(pthObjPRM, 1000);
    plot(pthObjPRM.States(:,1), pthObjPRM.States(:,2), ...
         "Color", [0.85 0.325 0.098], "LineWidth", 2);

    % Calculate PRM path cost (total length), using Euclidean Distance
    prmPath = pthObjPRM.States;
    prmCost = 0;
    for i = 1:size(prmPath,1)-1
        prmCost = prmCost + norm(prmPath(i+1,1:2) - prmPath(i,1:2));
    end
    prmNodes = size(nodes,1);
    title(sprintf('PRM\nCost: %.2f m, Nodes: %d', prmCost, prmNodes));
else
    disp("PRM: Path not found");
    title('PRM - Path not found');
end
hold off;


% RRT Planner (Right subplot)
subplot(1,2,2);
ssRRT = stateSpaceSE2;
ssRRT.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
svRRT = validatorOccupancyMap(ssRRT, Map=map);
svRRT.ValidationDistance = 0.01;
plannerRRT = plannerRRT(ssRRT, svRRT, MaxConnectionDistance=0.3);

show(map);
hold on;
rng(100, 'twister');
[pthObjRRT, solnInfoRRT] = plan(plannerRRT, start, goal);

if solnInfoRRT.IsPathFound
    % Tree expansion
    plot(solnInfoRRT.TreeData(:,1), solnInfoRRT.TreeData(:,2), '.-', ...
         'Color', [0.5 0.5 0.5]);
    % Path
    plot(pthObjRRT.States(:,1), pthObjRRT.States(:,2), 'r-', ...
         'LineWidth', 2);
    plot(start(1), start(2), "*", "Color", "g", "LineWidth", 3);
    plot(goal(1), goal(2), "*", "Color", "r", "LineWidth", 3);

    % Calculate RRT path cost (total length)
    rrtPath = pthObjRRT.States;
    rrtCost = 0;
    for i = 1:size(rrtPath,1)-1
        rrtCost = rrtCost + norm(rrtPath(i+1,1:2) - rrtPath(i,1:2));
    end
    rrtNodes = size(solnInfoRRT.TreeData,1);
    title(sprintf('RRT\nCost: %.2f m, Nodes: %d', rrtCost, rrtNodes));
else
    disp("RRT: Path not found");
    title('RRT - Path not found');
end
hold off;

% Adjust subplot spacing
subplot(1,2,1);
set(gca, 'Position', [0.05 0.11 0.43 0.815]);
subplot(1,2,2);
set(gca, 'Position', [0.52 0.11 0.43 0.815]);