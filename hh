clc; clear; close all;

%% Define Receiver Path
num_steps = 50; % Number of receiver positions
t = linspace(0, 2*pi, num_steps); % Parameter for curved path

% Receiver moves in a 3D spiral path
receiver_path = [5*cos(t); 5*sin(t); linspace(-5, 5, num_steps)]'; % [x, y, z]

%% Define Anchor Configurations
num_anchors_list = [4, 6, 8, 10]; % Testing different anchor counts
geometry_types = {'Tetrahedral', 'Cuboid', 'Irregular'};

% Define function for anchor placement
function anchors = generate_anchors(type, num_anchors)
    a = 5; h = 7; % Scaling parameters
    switch type
        case 'Tetrahedral'
            anchors = [
                a, 0, -h;
                -a, 0, -h;
                0, a, h;
                0, -a, h
            ];
        case 'Cuboid'
            anchors = [
                -a, -a, -h;
                 a, -a, -h;
                -a,  a, -h;
                 a,  a, -h;
                -a, -a,  h;
                 a, -a,  h;
                -a,  a,  h;
                 a,  a,  h
            ];
        case 'Irregular'
            rng(num_anchors); % Seed for reproducibility
            anchors = (rand(num_anchors, 3) - 0.5) * 20; % Randomized in a 20x20x20 space
    end
end

%% Function to Compute DOP Values
function [GDOP, HDOP, VDOP] = compute_dop(anchors, receiver)
    num_anchors = size(anchors, 1);
    ranges = vecnorm(anchors - receiver, 2, 2);  % Compute distances
    
    % Compute Jacobian Matrix H
    H = zeros(num_anchors, 3);
    for i = 1:num_anchors
        H(i, 1) = (receiver(1) - anchors(i, 1)) / ranges(i);  % dR/dx
        H(i, 2) = (receiver(2) - anchors(i, 2)) / ranges(i);  % dR/dy
        H(i, 3) = (receiver(3) - anchors(i, 3)) / ranges(i);  % dR/dz
    end
    
    % Compute Covariance Matrix P = (H^T * H)^-1
    P = inv(H' * H);
    
    % Compute GDOP, HDOP, VDOP
    GDOP = sqrt(trace(P));
    HDOP = sqrt(P(1,1) + P(2,2));
    VDOP = sqrt(P(3,3));
end

%% Simulate Receiver Path with Different Configurations
results = [];

for num_anchors = num_anchors_list
    for geom_type = geometry_types
        % Generate anchor placement
        anchors = generate_anchors(geom_type{1}, num_anchors);
        
        % Store DOP values for the path
        GDOP_values = zeros(num_steps, 1);
        HDOP_values = zeros(num_steps, 1);
        VDOP_values = zeros(num_steps, 1);

        for i = 1:num_steps
            receiver = receiver_path(i, :);
            [GDOP, HDOP, VDOP] = compute_dop(anchors, receiver);
            GDOP_values(i) = GDOP;
            HDOP_values(i) = HDOP;
            VDOP_values(i) = VDOP;
        end

        % Store results
        results = [results; {num_anchors, geom_type{1}, GDOP_values, HDOP_values, VDOP_values}];
    end
end

%% Plot Results
figure;

for i = 1:length(results)
    num_anchors = results{i, 1};
    geom_type = results{i, 2};
    GDOP_values = results{i, 3};
    
    subplot(2,2,i);
    plot(1:num_steps, GDOP_values, 'LineWidth', 1.5);
    title(sprintf('%s - %d Anchors', geom_type, num_anchors));
    xlabel('Receiver Path Step');
    ylabel('GDOP');
    grid on;
end

sgtitle('GDOP Variation Along Receiver Path');

%% 3D Visualization of Best & Worst Configurations
figure;

% Best Configuration (Lowest GDOP)
[~, best_idx] = min(cellfun(@(x) mean(x), results(:,3)));
best_anchors = generate_anchors(results{best_idx,2}, results{best_idx,1});

subplot(1,2,1); hold on; grid on;
scatter3(best_anchors(:,1), best_anchors(:,2), best_anchors(:,3), 'bo', 'filled');
plot3(receiver_path(:,1), receiver_path(:,2), receiver_path(:,3), 'r-', 'LineWidth', 1.5);
title(sprintf('Best Geometry: %s (%d Anchors)', results{best_idx,2}, results{best_idx,1}));
xlabel('X'); ylabel('Y'); zlabel('Z'); axis equal;

% Worst Configuration (Highest GDOP)
[~, worst_idx] = max(cellfun(@(x) mean(x), results(:,3)));
worst_anchors = generate_anchors(results{worst_idx,2}, results{worst_idx,1});

subplot(1,2,2); hold on; grid on;
scatter3(worst_anchors(:,1), worst_anchors(:,2), worst_anchors(:,3), 'ro', 'filled');
plot3(receiver_path(:,1), receiver_path(:,2), receiver_path(:,3), 'r-', 'LineWidth', 1.5);
title(sprintf('Worst Geometry: %s (%d Anchors)', results{worst_idx,2}, results{worst_idx,1}));
xlabel('X'); ylabel('Y'); zlabel('Z'); axis equal;

sgtitle('Comparison of Best and Worst Anchor Geometries');