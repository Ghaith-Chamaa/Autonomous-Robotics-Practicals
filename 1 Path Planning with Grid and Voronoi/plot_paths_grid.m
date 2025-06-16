clc; clear; close all;

% Define grid map (0 = free, 1 = obstacle) - used only to generate obstacles
% grd = [
%     0 0 0 0 0;
%     0 1 1 1 0;
%     0 0 0 1 0;
%     1 1 0 0 0;
%     0 0 0 0 0
% ];

% grd = [
%     0 0 0 0 0;
%     0 1 1 1 0;
%     0 1 1 1 0;
%     0 1 1 1 0;
%     0 0 0 0 0];

grd = [
    0 0 0 0 0 0 0;
    0 1 1 0 1 1 0;
    0 0 0 0 0 1 0;
    0 1 1 1 0 0 0;
    0 1 0 0 1 0 0;
    0 1 0 1 1 0 0;
    0 0 0 0 0 0 0
    ];

% Start and goal positions (in matrix coordinates: row, col)

% start_pos = [1, 1]; % [row, col]
% goal_pos = [5, 5];  % [row, col]

% start_pos = [1, 1]; % [row, col]
% goal_pos = [5, 5];  % [row, col]

start_pos = [1, 1];  % [row, col]
goal_pos = [7,7];    % [row, col]

% Run A* algorithm
connectivity = 8; % Set 4 or 8 connectivity
[path, path_cost, nodes_explored] = astar_grid(grd, start_pos, goal_pos, connectivity);

% Display results
if isempty(path)
    disp('No path found');
else
    disp('Path found:');
    disp(path);
    disp(['Path Cost: ' num2str(path_cost)]);
    disp(['Nodes Explored: ' num2str(nodes_explored)]);
end

% Plot the grid and path
figure; hold on; axis equal;
colormap([1 1 1; 0 0 0]); % White for free space, black for obstacles
imagesc(grd); % Ensures correct visualization orientation

% Adjust axes to align with the grid definition
set(gca, 'XTick', 1:size(grd,2), 'YTick', 1:size(grd,1));
set(gca, 'XLim', [0.5, size(grd,2) + 0.5], 'YLim', [0.5, size(grd,1) + 0.5]);
set(gca, 'YDir', 'reverse'); % Prevents flipping

% Plot start and goal positions
plot(start_pos(2), start_pos(1), 'go', 'MarkerSize', 12, 'LineWidth', 3); % Green: Start
plot(goal_pos(2), goal_pos(1), 'ro', 'MarkerSize', 12, 'LineWidth', 3); % Red: Goal

% Plot the path
if ~isempty(path)
    plot(path(:,2), path(:,1), 'b-', 'LineWidth', 2.5); % Blue line for path
    scatter(path(:,2), path(:,1), 50, 'b', 'filled'); % Blue dots for waypoints
end

% Add cost and nodes explored to title
title(['A* Grid Path Planning' sprintf('\nCost: %.2f, Nodes Explored: %d', path_cost, nodes_explored)]);
xlabel('X-axis (Columns)'); ylabel('Y-axis (Rows)');
grid on;
hold off;