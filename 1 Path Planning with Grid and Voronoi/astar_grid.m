function [path, path_cost, nodes_explored] = astar_grid(grid, start_pos, goal_pos, connectivity)
    % A* Algorithm for Grid-based Path Planning in MATLAB
    % Inputs:
    %   grid - 2D matrix (0 = free, 1 = obstacle)
    %   start_pos - [row, col] start position
    %   goal_pos - [row, col] goal position
    %   connectivity - 4 or 8 (movement connectivity)
    % Outputs:
    %   path - List of [row, col] coordinates from start to goal
    %   path_cost - Total cost of the path
    %   nodes_explored - Number of nodes explored

    % Define movement directions and costs
    if connectivity == 4
        moves = [0 1; 1 0; 0 -1; -1 0]; % N, E, S, W
        costs = [1, 1, 1, 1];
    elseif connectivity == 8
        moves = [0 1; 1 0; 0 -1; -1 0; -1 -1; -1 1; 1 -1; 1 1]; % Include diagonals
        costs = [1, 1, 1, 1, sqrt(2), sqrt(2), sqrt(2), sqrt(2)];
    else
        error('Connectivity must be 4 or 8');
    end

    % Grid size
    [rows, cols] = size(grid);
    
    % Heuristic function (Euclidean distance)
    h = @(pos) norm(pos - goal_pos);
    
    % Open list: [row, col, g, f, parent_row, parent_col]
    open_list = [start_pos, 0, h(start_pos), -1, -1];
    closed_list = zeros(rows, cols); % Closed list (tracks explored nodes)
    parent_map = zeros(rows, cols, 2) - 1; % Store parent nodes
    path_found = false;
    
    while ~isempty(open_list)
        % Select node with lowest f-cost
        [~, idx] = min(open_list(:, 4));
        current = open_list(idx, :);
        open_list(idx, :) = []; % Remove from open list
        
        % Mark as visited
        closed_list(current(1), current(2)) = 1;
        
        % Check if goal reached
        if isequal(current(1:2), goal_pos)
            path_found = true;
            break;
        end
        
        % Explore neighbors
        for i = 1:size(moves, 1)
            neighbor = current(1:2) + moves(i, :);
            
            % Check bounds 
            if neighbor(1) < 1 || neighbor(1) > rows || neighbor(2) < 1 || neighbor(2) > cols
                continue;
            end
            % Check for visited nodes and obstacles
            if grid(neighbor(1), neighbor(2)) == 1 || closed_list(neighbor(1), neighbor(2))
                continue;
            end
            
            % Compute cost
            g_new = current(3) + costs(i);
            f_new = g_new + h(neighbor);
            
            % Check if neighbor is already in open list
            existing_idx = find(ismember(open_list(:, 1:2), neighbor, 'rows'));
            if isempty(existing_idx) || open_list(existing_idx, 3) > g_new
                open_list = [open_list; neighbor, g_new, f_new, current(1:2)];
                parent_map(neighbor(1), neighbor(2), :) = current(1:2);
            end
        end
    end
    
    % Backtrack to get path and calculate cost
    if path_found
        path = goal_pos;
        path_cost = 0;
        current = goal_pos;
        while ~isequal(current, start_pos)
            parent = squeeze(parent_map(current(1), current(2), :))';
            if any(parent == -1)
                break;
            end
            % Calculate cost between current and parent
            move_diff = current - parent;
            move_idx = find(ismember(moves, move_diff, 'rows'));
            path_cost = path_cost + costs(move_idx);
            path = [parent; path];
            current = parent;
        end
        nodes_explored = sum(closed_list(:)); % Count nodes in closed list
    else
        path = [];
        path_cost = Inf;
        nodes_explored = sum(closed_list(:)); % Still count explored nodes
        disp('No path found');
    end
end