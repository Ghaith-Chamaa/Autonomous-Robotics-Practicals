clear

% Map setup
mapSize = [50 50]; % Grid size (50x50 cells)
resolution = 10;   % 10 cells per meter
binaryMap = zeros(mapSize);

start = [0.5 0.5 0];
goal = [4.5 4.5 0];

% Create polygonal obstacles
shape1x = [15 20 25 20 15];
shape1y = [10 15 20 25 20];
shape1 = polyshape(shape1x, shape1y);

shape2x = [30 40 35];
shape2y = [30 30 40];
shape2 = polyshape(shape2x, shape2y);

shape3x = [25 30 35 30 25 20];
shape3y = [35 40 35 30 25 30];
shape3 = polyshape(shape3x, shape3y);

% Convert polygons to binary map
[X, Y] = meshgrid(1:mapSize(1), 1:mapSize(2));
points = [X(:) Y(:)];

inShape1 = isinterior(shape1, points);
inShape2 = isinterior(shape2, points);
inShape3 = isinterior(shape3, points);

binaryMap(inShape1) = 1;
binaryMap(inShape2) = 1;
binaryMap(inShape3) = 1;

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
% Capable of using a sampler like PRM, and has MaxConnectionDistance ann 
% MaxNumTreeNodes similar to MaxNumNodes in PRM
% But given that RRT is iterative then we have MaxIterations, so it stops
% either when it hits iterations whether or not the end is reached , or the
% number of nodes has been exhausted, default val for MaxIterations = 1e4

% NOTE : lowering the MaxConnectionDistance has an effect on the palength
plannerRRT = plannerRRT(ssRRT, svRRT, MaxConnectionDistance=0.2);

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