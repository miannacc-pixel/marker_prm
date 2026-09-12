clear all
close all
clc

% Paired-roadmap evaluation

number_of_trials = 30;
random_seeds = 1:number_of_trials;

N = 2000; % Total number of roadmap vertices, including the initial vertex

%% Definition of the roadmap vertex structure

% Just setting initial values for unsampled vertices 
ini_st = -100; % initial value for unsampled nodes
ini_P = diag([-100 5]); % initial covariance for unsampled nodes
ini_value = 5000; % initial cost for unsampled nodes

% Initilize node with the value which will not be outputted by algorithm
node(1:N) = struct('x', ini_st*ones(1,2), 'P', ini_P, 't', ini_st, ...
    'parent', 0, 'value', ini_value, 'ra', ini_st, 'rb', ini_st, ...
    'ang', ini_st, 'ellipse_rect', ini_st*ones(1,4));

% node.x: The position (2-D) of the roadmap vertex   (1*2 vector)
% node.P: The covariance (2-D) of the roadmap vertex (2*2 matrix)
% node.t: The elapsed travel time from the initial vertex (scalar)
% node.parent: The predecessor on the final shortest path tree (scalar value)
% node.value: The shortest path cost from the initial vertex (scalar value)
% node.ra: The length of major axis of ellipse  (scalar value)
% node.rb: The length of minor axis of ellipse  (scalar value)
% node.ang: The rotation angle of the ellipse (range is from 0 to 2*pi) (scalar value)
% node.ellipse_rect: A bounding box which surrouds the ellipse 
%        [bottom-left-x bottom-left-y width height]  ([1, 4] matrix)

%% PRM parameters

% Two spatial PRM vertices are considered for a connection when their
% Euclidean distance is less than this value.
connection_radius = 0.5;

% Weight on normalized terminal uncertainty in Dtotal = Dtravel + lambda*trace(Pgoal)/trace(Pstart).
lambda = 0.2;

% EKF prediction model. P is propagated along each accepted PRM edge rather
% than independently sampled at every roadmap vertex.
F = eye(2);

% Fixed-speed time model. Time is propagated along an edge as
% t_next = t_current + Dtravel/robot_speed.
robot_speed = 1.0; % [m/s]

% The gain of process noise per traveled meter (W in the original paper).
R = (1/10000)*eye(2);

% Confidence bound used for collision checking
safety_probability = 0.8;
chi = chi2inv(safety_probability,2);

%% Optional stigmergy marker

% In main.m, marker_enabled toggles one planner run. This evaluation instead
% runs both settings on every shared roadmap, so no toggle is needed here.
marker.x = [0.80, 0.60];
marker.sensing_radius = 0.10;
marker.H = eye(2);
marker.R = 1e-5 * eye(2);

%% Environment definition and Properties

% current enviroment is  " multiple obstacle enviroment" 

        environment_name = 'baseline';

        % define obstacle as a set of edges 
        % each edge is defined by: start point, end point, slope, and Y_axis
        % intercept
        obstacle_edge = obstacle_multi(environment_name);
        obs_polyshape= obstacle_polyshape(environment_name); %definition of obstacles to use polyshape functionalities of Matlab

        % Target(final) area [xmin, xmax; ymin, ymax]. 
        target = [0.8, 0.9; 0.1, 0.2];

        % Path planning area
        bound(1).x = [0,1];
        bound(2).x = [0,1];

        % The position of the initial node
        node(1).x = [0.1, 0.1];

%% The setting for initial node
node(1).P = 1e-4 * eye(2);
node(1).t = 0;
initial_uncertainty = trace(node(1).P);
node(1).value = lambda * trace(node(1).P) / initial_uncertainty;

[node(1).ra,node(1).rb,node(1).ang,node(1).ellipse_rect] = error_ellipse(node(1).x, node(1).P, chi);
% [ra=major axis, rb=minor axis, ang= rotation angle , rect=bounding box] 
% = error_ellipse(x= 2D position of the ellipse, P = covariance , chi= confidence level)

%% Parameters for collision checking along a roadmap edge

% How many intermediate ellipses are used to check one directed edge
num_props = 10;

% Definition of intermediate ellipses 
prop(1:num_props) = struct('x', ini_st*ones(1,2), 'P', ini_st*eye(2), 'ra', ini_st,...
    'rb', ini_st, 'ang', ini_st, 'ellipse_rect', zeros(1,4));

% Initialize "prop", which have following structure
% prop.x: The position (2-D) of the node  (1*2 vector)
% prop.P: The covariance (2-D) of the node  (2*2 matrix)
% prop.ra: The length of major axis of ellipse  (scalar)
% prop.rb: The length of minor axis of ellipse  (scalar)
% prop.ang: The rotation angle of the ellipse   (range is from 0 to 2*pi)
% prop.ellipse_rect: A bounding box which surrouds the ellipse
%        [bottom-left-x bottom-left-y width height]
% prop.ellipse_rect: Used for collision checking (Boolean)

%% Variables used to save paired-roadmap results

seed = random_seeds(:);
marker_off_success = false(number_of_trials,1);
marker_on_success = false(number_of_trials,1);
marker_on_visited = false(number_of_trials,1);

marker_off_travel = nan(number_of_trials,1);
marker_on_travel = nan(number_of_trials,1);
marker_off_terminal_uncertainty = nan(number_of_trials,1);
marker_on_terminal_uncertainty = nan(number_of_trials,1);
marker_off_total_cost = nan(number_of_trials,1);
marker_on_total_cost = nan(number_of_trials,1);
marker_off_planning_time = nan(number_of_trials,1);
marker_on_planning_time = nan(number_of_trials,1);
roadmap_generation_time = nan(number_of_trials,1);

marker_off_path = cell(number_of_trials,1);
marker_on_path = cell(number_of_trials,1);
roadmap_positions = cell(number_of_trials,1);
marker_off_P_goal = cell(number_of_trials,1);
marker_on_P_goal = cell(number_of_trials,1);

completed_trials = 0;
experiment_complete = false;

%% PRM algorithm

tic

for trial = 1:number_of_trials
    rng(random_seeds(trial), 'twister')

    % Reset the roadmap vertex structure before sampling a new paired roadmap.
    shared_node = node;

    % Step 1: Sample collision-free spatial states to create the roadmap vertices.
    % Their covariances are assigned later by EKF propagation during the search.
    roadmap_tic = tic;
    for ii = 2:N
        shared_node(ii).x = sample_free_position(bound, obs_polyshape);
    end

    % Step 2: Build spatial PRM neighborhoods. Edge covariances cannot be fixed
    % here because they depend on the belief propagated to the source vertex.
    % The paired evaluation reuses this exact roadmap for both planner conditions.
    x_all = reshape([shared_node.x], 2, N).';
    neighbor_ID = rangesearch(x_all, x_all, connection_radius);
    roadmap_generation_time(trial) = toc(roadmap_tic);
    roadmap_positions{trial} = x_all;

    % Step 3: Dijkstra search with EKF covariance and time propagation. When the
    % marker is present, use one layer before sensing and one after sensing.
    % The paired evaluation runs both versions instead of selecting one with a toggle.

    %% Marker-absent search

    node_off = shared_node;
    planner_tic = tic;

    [node_off, distance_off, predecessor_off] = dijkstra_ekf_prm( ...
        node_off, neighbor_ID, F, R, robot_speed, lambda, obstacle_edge, ...
        chi, bound, num_props, prop);

    marker_off_planning_time(trial) = toc(planner_tic);

    all_rec = reshape([node_off.ellipse_rect], [4, N]).';
    in_target = all_rec(:,1) >= target(1,1) & ...
        all_rec(:,1) + all_rec(:,3) <= target(1,2) & ...
        all_rec(:,2) >= target(2,1) & ...
        all_rec(:,2) + all_rec(:,4) <= target(2,2);
    target_ID = find(in_target & isfinite(distance_off));

    path_off = [];
    if ~isempty(target_ID)
        [~, target_index] = min(distance_off(target_ID));
        path_off = reconstruct_prm_path( ...
            predecessor_off, target_ID(target_index), 1);
    end

    if ~isempty(path_off)
        marker_off_success(trial) = true;
        marker_off_path{trial} = path_off;
        positions_off = reshape([node_off(path_off).x], 2, []).';
        marker_off_travel(trial) = sum( ...
            vecnorm(diff(positions_off,1,1), 2, 2));
        marker_off_P_goal{trial} = node_off(path_off(end)).P;
        marker_off_terminal_uncertainty(trial) = ...
            trace(node_off(path_off(end)).P);
        marker_off_total_cost(trial) = marker_off_travel(trial) + ...
            lambda * marker_off_terminal_uncertainty(trial) / initial_uncertainty;
    end

    %% Marker-present search

    node_on = shared_node;
    planner_tic = tic;

    [node_on, ~, ~, path_on] = dijkstra_ekf_marker_prm( ...
        node_on, neighbor_ID, F, R, robot_speed, lambda, marker, ...
        obstacle_edge, chi, bound, num_props, prop, target);

    marker_on_planning_time(trial) = toc(planner_tic);

    if ~isempty(path_on)
        marker_on_success(trial) = true;
        marker_on_path{trial} = path_on;
        positions_on = reshape([node_on(path_on).x], 2, []).';
        marker_on_travel(trial) = sum( ...
            vecnorm(diff(positions_on,1,1), 2, 2));
        marker_on_P_goal{trial} = node_on(path_on(end)).P;
        marker_on_terminal_uncertainty(trial) = ...
            trace(node_on(path_on(end)).P);
        marker_on_total_cost(trial) = marker_on_travel(trial) + ...
            lambda * marker_on_terminal_uncertainty(trial) / initial_uncertainty;

        for ii = 1:size(positions_on,1)-1
            if marker_encounter(positions_on(ii,:), positions_on(ii+1,:), marker)
                marker_on_visited(trial) = true;
                break
            end
        end
    end

    %% Output Data
    %%%%%%%%%%%%%% All data should be saved here %%%%%%%%%%%%%%%
    % The file name used for save the data
    % Data is saved in "data" folder
    % Name includes the paired experiment, environment, N, lambda value, 
    % safety probability, and number of trials.

    completed_trials = trial;
    timeElapsed = toc;

    paired_results = table(seed, marker_off_success, marker_on_success, ...
        marker_on_visited, marker_off_travel, marker_on_travel, ...
        marker_off_terminal_uncertainty, marker_on_terminal_uncertainty, ...
        marker_off_total_cost, marker_on_total_cost, ...
        marker_off_planning_time, marker_on_planning_time, ...
        roadmap_generation_time);

    savename = ['data/paired_', environment_name, ...
        '_N', num2str(N), ...
        '_lambda_', num2str(lambda), ...
        '_safety_', num2str(safety_probability),'_trials_', num2str(number_of_trials)];
    savename(savename=='.') = [];

    save(savename, 'paired_results', 'marker_off_path', 'marker_on_path', ...
        'roadmap_positions', 'marker_off_P_goal', 'marker_on_P_goal', ...
        'completed_trials', 'experiment_complete', 'timeElapsed', ...
        'number_of_trials', 'random_seeds', 'N', 'connection_radius', ...
        'lambda', 'F', 'R', 'robot_speed', 'chi', 'marker', ...
        'obstacle_edge', 'obs_polyshape', 'target', 'bound', '-v7.3')

    fprintf(['Trial %d/%d: marker off success = %d, ', ...
        'marker on success = %d\n'], trial, number_of_trials, ...
        marker_off_success(trial), marker_on_success(trial))
end

%% Paired statistical summary

complete_pair = marker_off_success & marker_on_success;

paired_difference_travel = marker_on_travel - marker_off_travel;
paired_difference_terminal_uncertainty = ...
    marker_on_terminal_uncertainty - marker_off_terminal_uncertainty;
paired_difference_total_cost = marker_on_total_cost - marker_off_total_cost;
paired_difference_planning_time = ...
    marker_on_planning_time - marker_off_planning_time;

summary.number_of_trials = number_of_trials;
summary.number_of_complete_pairs = sum(complete_pair);
summary.marker_off_success_rate = mean(marker_off_success);
summary.marker_on_success_rate = mean(marker_on_success);
summary.marker_on_visit_rate = mean(marker_on_visited(marker_on_success));

summary.travel = paired_statistics(marker_off_travel, ...
    marker_on_travel, complete_pair);
summary.terminal_uncertainty = paired_statistics( ...
    marker_off_terminal_uncertainty, marker_on_terminal_uncertainty, ...
    complete_pair);
summary.total_cost = paired_statistics(marker_off_total_cost, ...
    marker_on_total_cost, complete_pair);
summary.planning_time = paired_statistics(marker_off_planning_time, ...
    marker_on_planning_time, complete_pair);

experiment_complete = true;
timeElapsed = toc;

save(savename, 'paired_results', 'summary', ...
    'paired_difference_travel', ...
    'paired_difference_terminal_uncertainty', ...
    'paired_difference_total_cost', 'paired_difference_planning_time', ...
    'marker_off_path', 'marker_on_path', 'roadmap_positions', ...
    'marker_off_P_goal', 'marker_on_P_goal', 'completed_trials', ...
    'experiment_complete', 'timeElapsed', 'number_of_trials', ...
    'random_seeds', 'N', 'connection_radius', 'lambda', 'F', 'R', ...
    'robot_speed', 'chi', 'marker', 'obstacle_edge', 'obs_polyshape', ...
    'target', 'bound', '-v7.3')

fprintf('\nPaired-roadmap evaluation complete. Data saved to %s.mat\n', savename)
fprintf('Complete pairs: %d/%d\n', sum(complete_pair), number_of_trials)

function statistics = paired_statistics(marker_off, marker_on, valid_pair)
valid = valid_pair & isfinite(marker_off) & isfinite(marker_on);
difference = marker_on(valid) - marker_off(valid);

statistics.number_of_pairs = sum(valid);
statistics.marker_off_mean = mean(marker_off(valid));
statistics.marker_off_std = std(marker_off(valid));
statistics.marker_on_mean = mean(marker_on(valid));
statistics.marker_on_std = std(marker_on(valid));
statistics.paired_difference_mean = mean(difference);
statistics.paired_difference_std = std(difference);
end
