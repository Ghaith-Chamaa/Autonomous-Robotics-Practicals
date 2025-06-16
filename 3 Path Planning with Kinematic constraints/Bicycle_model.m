% Clear workspace and figures
clear;
close all;

% Define bicycle model parameters
dt = 0.1;          % Time step (s)
t_max = 5.0;       % Total simulation time (s)
n_steps = floor(t_max/dt); % Number of time steps

% Function to simulate bicycle model motion
function [new_state] = bicycle_model(state, v, omega, dt, L)
    % state = [x, y, theta, delta]
    % v: linear velocity
    % omega: steering rate
    % dt: time step
    % L: wheelbase
    
    x = state(1);
    y = state(2);
    theta = state(3);
    delta = state(4);
    
    % Update state using Euler integration
    x_dot = v * cos(theta);
    y_dot = v * sin(theta);
    theta_dot = (v/L) * tan(delta);
    delta_dot = omega;
    
    new_state = [
        x + x_dot * dt
        y + y_dot * dt
        theta + theta_dot * dt
        delta + delta_dot * dt
    ];
end

% Initial state [x, y, theta, delta]
initial_state = [0, 0, 0, 0];

% Test cases
test_cases = {
    struct('v', 1.0, 'omega', 0.0, 'L', 1.0, 'name', 'Straight Line', 'gradual', false),           % Constant velocity, no steering
    struct('v', 1.0, 'omega', 0.2, 'L', 1.0, 'name', 'Gradual Steering', 'gradual', true),      % Positive steering rate (gradual)
    struct('v', 2.0, 'omega', 0.2, 'L', 1.0, 'name', 'Faster Right Turn', 'gradual', false),      % Higher velocity
    struct('v', 1.0, 'omega', 0.2, 'L', 0.5, 'name', 'Small Wheelbase', 'gradual', false),        % Smaller L (0.5m)
    struct('v', 1.0, 'omega', 0.2, 'L', 2.0, 'name', 'Large Wheelbase', 'gradual', false)         % Larger L (2.0m)
};

% Simulate and plot each test case separately
for i = 1:length(test_cases)
    % Reset state
    state = initial_state;
    trajectory = zeros(n_steps+1, 4);
    trajectory(1,:) = state;
    
    % Check if this case should have gradual steering
    if test_cases{i}.gradual
        % Calculate omega increment for gradual cases
        omega_max = test_cases{i}.omega;
        omega_increment = omega_max / n_steps;
        current_omega = 0;
        
        % Simulate motion with gradually increasing omega
        for t = 1:n_steps
            state = bicycle_model(state, test_cases{i}.v, current_omega, ...
                               dt, test_cases{i}.L);
            trajectory(t+1,:) = state;
            current_omega = min(current_omega + omega_increment, omega_max);
        end
    else
        % Simulate motion with constant omega
        for t = 1:n_steps
            state = bicycle_model(state, test_cases{i}.v, test_cases{i}.omega, ...
                               dt, test_cases{i}.L);
            trajectory(t+1,:) = state;
        end
    end
    
    % Create new figure for each test case
    figure('Position', [100 100 600 500]);
    hold on;
    
    % Plot trajectory
    plot(trajectory(:,1), trajectory(:,2), 'b-', 'LineWidth', 2);
    
    % Plot initial orientation
    arrow_length = 0.5;
    quiver(0, 0, arrow_length*cos(0), arrow_length*sin(0), 'k', ...
           'LineWidth', 1.5, 'MaxHeadSize', 0.5);
    
    % Figure formatting
    title(['Bicycle Model: ' test_cases{i}.name]);
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    grid on;
    axis equal;
    
    % Add parameters text (show max omega for gradual cases)
    if test_cases{i}.gradual
        text_str = sprintf('v = %.1f m/s\nω_max = %.1f rad/s\nL = %.1f m', ...
                          test_cases{i}.v, test_cases{i}.omega, test_cases{i}.L);
    else
        text_str = sprintf('v = %.1f m/s\nω = %.1f rad/s\nL = %.1f m', ...
                          test_cases{i}.v, test_cases{i}.omega, test_cases{i}.L);
    end
    text(max(trajectory(:,1))*0.1, max(trajectory(:,2))*0.9, text_str);
    
    hold off;
end