clc; clear; close all;

% Define grid map (0 = free, 1 = obstacle) - used only to generate obstacles
% grd = [
%     0 0 0 0 0;
%     0 1 1 1 0;
%     0 0 0 1 0;
%     1 1 0 0 0;
%     0 0 0 0 0
% ];

grd = [
    0 0 0 0 0;
    0 1 1 1 0;
    0 1 1 1 0;
    0 1 1 1 0;
    0 0 0 0 0];

% grd = [
%     0 0 0 0 0 0 0;
%     0 1 1 0 1 1 0;
%     0 0 0 0 0 1 0;
%     0 1 1 1 0 0 0;
%     0 1 0 0 1 0 0;
%     0 1 0 1 1 0 0;
%     0 0 0 0 0 0 0
%     ];

% Start and goal positions (in matrix coordinates: row, col)

% start_pos = [1, 1]; % [row, col]
% goal_pos = [5, 5];  % [row, col]

start_pos = [1, 1]; % [row, col]
goal_pos = [5, 5];  % [row, col]

% start_pos = [1, 1];  % [row, col]
% goal_pos = [7,7];    % [row, col]

% Extract obstacle points for Voronoi (in matrix coordinates)
[obs_row, obs_col] = find(grd == 1);
obstacles_matrix = [obs_col, obs_row]; % [col, row] format

% Convert to plot coordinates (x, y) where y is flipped
grid_height = size(grd, 1);
obstacles = obstacles_matrix;
obstacles(:,2) = grid_height - obstacles(:,2) + 1; % Flip y-coordinate

start_pos_plot = [start_pos(2), grid_height - start_pos(1) + 1]; % [x, y]
goal_pos_plot = [goal_pos(2), grid_height - goal_pos(1) + 1];    % [x, y]

% Run A* Voronoi algorithm with flipped coordinates
[path_voronoi, path_cost, nodes_explored] = astar_voronoi(obstacles, start_pos_plot, goal_pos_plot);

% Create figure for Voronoi plot
figure('Position', [100 100 400 400]);

% Plot Voronoi-based A*
hold on; axis equal;
voronoi(obstacles(:,1), obstacles(:,2));
scatter(obstacles(:,1), obstacles(:,2), 100, 'k', 's', 'filled');
plot(start_pos_plot(1), start_pos_plot(2), 'go', 'MarkerSize', 12, 'LineWidth', 3);
plot(goal_pos_plot(1), goal_pos_plot(2), 'ro', 'MarkerSize', 12, 'LineWidth', 3);

if ~isempty(path_voronoi)
    plot(path_voronoi(:,1), path_voronoi(:,2), 'b-', 'LineWidth', 2.5);
    scatter(path_voronoi(:,1), path_voronoi(:,2), 50, 'b', 'filled');
end

title(['A* Voronoi Path Planning' sprintf('\nCost: %.2f, Nodes Explored: %d', path_cost, nodes_explored)]);
xlabel('X'); ylabel('Y');
xlim([0.5 size(grd,2)+0.5]); ylim([0.5 size(grd,1)+0.5]);
grid on;
hold off;

% Convert path back to matrix coordinates for display
if ~isempty(path_voronoi)
    path_voronoi_display = path_voronoi;
    path_voronoi_display(:,2) = grid_height - path_voronoi(:,2) + 1; % Convert back to row numbers
else
    path_voronoi_display = [];
end

% Display results
disp('Voronoi-based A* path (in matrix coordinates [col, row]):');
disp(path_voronoi_display);
disp(['Path Cost: ' num2str(path_cost)]);
disp(['Nodes Explored: ' num2str(nodes_explored)]);