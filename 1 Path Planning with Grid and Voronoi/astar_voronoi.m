function [path, path_cost, nodes_explored] = astar_voronoi(obstacles, start_pos, goal_pos)
    % A* path planning using Voronoi diagram with start/goal connection
    % obstacles: matrix of obstacle points [x, y]
    % start_pos: [x, y] starting position
    % goal_pos: [x, y] goal position
    % Outputs: path, total path cost, number of nodes explored
    
    % Include start and goal as points for Voronoi diagram
    points = [obstacles; start_pos; goal_pos];
    
    % Generate Voronoi diagram
    [vx, vy] = voronoi(points(:,1), points(:,2));
    
    % Create edges from Voronoi diagram
    edges_start = [vx(1,:)' vy(1,:)'];
    edges_end = [vx(2,:)' vy(2,:)'];
    
    % Remove infinite edges
    valid = ~(isinf(edges_start(:,1)) | isinf(edges_start(:,2)) | ...
             isinf(edges_end(:,1)) | isinf(edges_end(:,2)));
    edges_start = edges_start(valid,:);
    edges_end = edges_end(valid,:);
    
    % Get unique vertices, ensuring start and goal are included
    vertices = unique([edges_start; edges_end; start_pos; goal_pos], 'rows');
    
    % Create adjacency list
    n = size(vertices, 1);
    adj = cell(n, 1);
    costs = cell(n, 1);
    
    % Build graph from Voronoi edges
    for i = 1:size(edges_start, 1)
        v1_idx = find(ismember(vertices, edges_start(i,:), 'rows'));
        v2_idx = find(ismember(vertices, edges_end(i,:), 'rows'));
        
        if v1_idx == v2_idx
            continue;
        end
        
        if ~check_collision(vertices(v1_idx,:), vertices(v2_idx,:), obstacles)
            dist = sqrt(sum((vertices(v1_idx,:) - vertices(v2_idx,:)).^2));
            adj{v1_idx} = [adj{v1_idx} v2_idx];
            adj{v2_idx} = [adj{v2_idx} v1_idx];
            costs{v1_idx} = [costs{v1_idx} dist];
            costs{v2_idx} = [costs{v2_idx} dist];
        end
    end
    
    % Connect start and goal to the graph
    start_idx = find(ismember(vertices, start_pos, 'rows'));
    goal_idx = find(ismember(vertices, goal_pos, 'rows'));
    
    % Connect start to nearest valid vertices
    [start_connections, start_costs] = connect_to_graph(start_pos, vertices, obstacles, start_idx);
    adj{start_idx} = [adj{start_idx} start_connections];
    costs{start_idx} = [costs{start_idx} start_costs];
    for i = 1:length(start_connections)
        adj{start_connections(i)} = [adj{start_connections(i)} start_idx];
        costs{start_connections(i)} = [costs{start_connections(i)} start_costs(i)];
    end
    
    % Connect goal to nearest valid vertices
    [goal_connections, goal_costs] = connect_to_graph(goal_pos, vertices, obstacles, goal_idx);
    adj{goal_idx} = [adj{goal_idx} goal_connections];
    costs{goal_idx} = [costs{goal_idx} goal_costs];
    for i = 1:length(goal_connections)
        adj{goal_connections(i)} = [adj{goal_connections(i)} goal_idx];
        costs{goal_connections(i)} = [costs{goal_connections(i)} goal_costs(i)];
    end
    
    % A* implementation
    if isempty(start_idx) || isempty(goal_idx)
        path = [];
        path_cost = Inf;
        nodes_explored = 0;
        return;
    end
    
    g_score = inf(n, 1);
    f_score = inf(n, 1);
    came_from = zeros(n, 1);
    explored = false(n, 1); % Track explored nodes
    
    g_score(start_idx) = 0;
    f_score(start_idx) = heuristic(start_pos, goal_pos);
    
    open_set = start_idx;
    
    while ~isempty(open_set)
        [~, current_idx] = min(f_score(open_set));
        current = open_set(current_idx);
        
        % Mark node as explored
        explored(current) = true;
        
        if current == goal_idx
            [path, path_cost] = reconstruct_path(came_from, current, vertices, adj, costs);
            nodes_explored = sum(explored);
            return;
        end
        
        open_set(current_idx) = [];
        
        for i = 1:length(adj{current})
            neighbor = adj{current}(i);
            tentative_g_score = g_score(current) + costs{current}(i);
            
            if tentative_g_score < g_score(neighbor)
                came_from(neighbor) = current;
                g_score(neighbor) = tentative_g_score;
                f_score(neighbor) = g_score(neighbor) + ...
                    heuristic(vertices(neighbor,:), goal_pos);
                
                if ~ismember(neighbor, open_set)
                    open_set = [open_set neighbor];
                end
            end
        end
    end
    
    path = []; % No path found
    path_cost = Inf;
    nodes_explored = sum(explored);
end

function [connections, connection_costs] = connect_to_graph(point, vertices, obstacles, point_idx)
    % Connect point to nearest valid vertices
    connections = [];
    connection_costs = [];
    distances = sqrt(sum((vertices - point).^2, 2));
    [sorted_dist, idx] = sort(distances);
    
    % Connect to up to 3 nearest vertices that don't collide
    connection_count = 0;
    i = 2; % Skip self (idx(1) is the point itself)
    while i <= length(idx) && connection_count < 3
        if idx(i) ~= point_idx && ~check_collision(point, vertices(idx(i),:), obstacles)
            connections = [connections idx(i)];
            connection_costs = [connection_costs sorted_dist(i)];
            connection_count = connection_count + 1;
        end
        i = i + 1;
    end
end

function h = heuristic(pos1, pos2)
    h = sqrt(sum((pos1 - pos2).^2));
end

function [path, path_cost] = reconstruct_path(came_from, current, vertices, adj, costs)
    path = vertices(current,:);
    path_cost = 0;
    current_idx = current;
    while came_from(current_idx) ~= 0
        prev_idx = came_from(current_idx);
        % Find the cost between current and previous node
        neighbors = adj{prev_idx};
        cost_idx = find(neighbors == current_idx);
        if ~isempty(cost_idx)
            path_cost = path_cost + costs{prev_idx}(cost_idx);
        end
        current_idx = prev_idx;
        path = [vertices(current_idx,:); path];
    end
end

function collision = check_collision(p1, p2, obstacles)
    collision = false;
    for i = 1:size(obstacles, 1)
        obs = obstacles(i,:);
        [dist, ~] = point_to_line_distance(obs, p1, p2);
        if dist < 0.5
            collision = true;
            return;
        end
    end
end

function [dist, t] = point_to_line_distance(p, l1, l2)
    v = l2 - l1;
    w = p - l1;
    c1 = dot(w, v);
    c2 = dot(v, v);
    t = max(0, min(1, c1 / c2));
    closest = l1 + t * v;
    dist = sqrt(sum((p - closest).^2));
end