clear all
close all
clc

% Sensitivity analysis

number_of_trials = 5;
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

        % Target(final) area [xmin, xmax; ymin, ymax]
        target = [0.8, 0.9; 0.1, 0.2];

        % Path planning area
        bound(1).x = [0,1];
        bound(2).x = [0,1];

        % The position of the initial node
        node(1).x = [0.1, 0.1];

%% The setting for initial node
node(1).P = 1e-4 * eye(2);
node(1).t = 0;
node(1).value = lambda * trace(node(1).P) / trace(node(1).P);

[node(1).ra,node(1).rb,node(1).ang,node(1).ellipse_rect] = error_ellipse(node(1).x, node(1).P, chi);
% [ra=major axis, rb=minor axis, ang= rotation angle , rect=bounding box] 
% = error_ellipse(x= 2D position of the ellipse, P = covariance , chi= confidence level)

initial_x = node(1).x;
initial_P = node(1).P;

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

%% Sensitivity values

% The lambda values span both sides of the approximate point at which the
% marker detour becomes worthwhile in the paired-roadmap evaluation.
lambda_values = [0, 0.05, 0.1, 0.2, 0.8];

% Each scalar below defines an isotropic covariance R_m = value*I.
marker_covariance_values = [1e-6, 1e-5, 1e-4, 5e-4, 1e-3];

% Marker locations progress from near the direct route toward locations
% that require a larger detour.
marker_location_values = [0.65, 0.20; ...
                          0.65, 0.40; ...
                          0.65, 0.60; ...
                          0.80, 0.60; ...
                          0.80, 0.80];

factor_names = {'lambda', 'marker_covariance', 'marker_location'};
number_of_levels = [numel(lambda_values), ...
    numel(marker_covariance_values), ...
    size(marker_location_values,1)];
maximum_rows = 2 * number_of_trials * sum(number_of_levels);

%% Variables used to save sensitivity-analysis results

factor = cell(maximum_rows,1);
level = nan(maximum_rows,1);
level_value = nan(maximum_rows,1);
marker_x = nan(maximum_rows,1);
marker_y = nan(maximum_rows,1);
seed = nan(maximum_rows,1);
marker_enabled = false(maximum_rows,1);
success = false(maximum_rows,1);
marker_visited = false(maximum_rows,1);
travel_distance = nan(maximum_rows,1);
terminal_uncertainty = nan(maximum_rows,1);
total_cost = nan(maximum_rows,1);
planning_time = nan(maximum_rows,1);
roadmap_generation_time = nan(maximum_rows,1);
path = cell(maximum_rows,1);
P_goal = cell(maximum_rows,1);

row = 0;
completed_pairs = 0;
total_pairs = number_of_trials * sum(number_of_levels);
experiment_complete = false;

savename = ['data/sensitivity_', environment_name, ...
        '_N', num2str(N), ...
        '_lambda_', num2str(lambda), ...
        '_safety_', num2str(safety_probability),'_trials_', num2str(number_of_trials)];
savename(savename=='.') = [];

%% PRM algorithm

tic

for factor_index = 1:numel(factor_names)
    current_factor = factor_names{factor_index};

    for level_index = 1:number_of_levels(factor_index)
        for trial = 1:number_of_trials

            % Begin with the baseline configuration for every paired run.
            current_N = N;
            current_lambda = lambda;
            current_marker = marker;

            switch current_factor
                case 'lambda'
                    current_value = lambda_values(level_index);
                    current_lambda = current_value;
                case 'marker_covariance'
                    current_value = marker_covariance_values(level_index);
                    current_marker.R = current_value * eye(2);
                case 'marker_location'
                    current_value = level_index;
                    current_marker.x = marker_location_values(level_index,:);
            end

            rng(random_seeds(trial), 'twister')

            %% Definition of the roadmap vertex structure

            node = repmat(struct('x', ini_st*ones(1,2), ...
                'P', ini_P, 't', ini_st, 'parent', 0, ...
                'value', ini_value, 'ra', ini_st, 'rb', ini_st, ...
                'ang', ini_st, 'ellipse_rect', ini_st*ones(1,4)), ...
                1, current_N);

            node(1).x = initial_x;
            node(1).P = initial_P;
            node(1).t = 0;
            node(1).value = current_lambda * trace(node(1).P) / ...
                trace(node(1).P);
            [node(1).ra,node(1).rb,node(1).ang,node(1).ellipse_rect] = ...
                error_ellipse(node(1).x, node(1).P, chi);

            % Step 1: Sample collision-free spatial states to create the roadmap vertices.
            % Their covariances are assigned later by EKF propagation during the search.
            roadmap_tic = tic;
            for ii = 2:current_N
                node(ii).x = sample_free_position(bound, obs_polyshape);
            end

            % Step 2: Build spatial PRM neighborhoods. Edge covariances cannot be fixed
            % here because they depend on the belief propagated to the source vertex.
            % The sensitivity analysis reuses this exact roadmap for both planner conditions..
            x_all = reshape([node.x], 2, current_N).';
            neighbor_ID = rangesearch(x_all, x_all, connection_radius);
            current_roadmap_time = toc(roadmap_tic);

            % Step 3: Dijkstra search with EKF covariance and time propagation. When the
            % marker is present, use one layer before sensing and one after sensing.
            % The sensitivity analysis runs both versions instead of selecting one with a toggle.

            %% Marker-absent search

            node_off = node;
            planner_tic = tic;
            [node_off, distance_off, predecessor_off] = dijkstra_ekf_prm( ...
                node_off, neighbor_ID, F, R, robot_speed, current_lambda, ...
                obstacle_edge, chi, bound, num_props, prop);
            off_time = toc(planner_tic);

            path_off = find_target_path(node_off, distance_off, ...
                predecessor_off, target);
            off_result = calculate_result(node_off, path_off, ...
                current_lambda, current_marker);

            %% Marker-present search

            node_on = node;
            planner_tic = tic;
            [node_on, ~, ~, path_on] = dijkstra_ekf_marker_prm( ...
                node_on, neighbor_ID, F, R, robot_speed, current_lambda, ...
                current_marker, obstacle_edge, chi, bound, num_props, ...
                prop, target);
            on_time = toc(planner_tic);

            on_result = calculate_result(node_on, path_on, ...
                current_lambda, current_marker);

            %% Store both members of the paired result

            for mode = 1:2
                row = row + 1;
                factor{row} = current_factor;
                level(row) = level_index;
                level_value(row) = current_value;
                marker_x(row) = current_marker.x(1);
                marker_y(row) = current_marker.x(2);
                seed(row) = random_seeds(trial);
                roadmap_generation_time(row) = current_roadmap_time;

                if mode == 1
                    marker_enabled(row) = false;
                    result = off_result;
                    planning_time(row) = off_time;
                else
                    marker_enabled(row) = true;
                    result = on_result;
                    planning_time(row) = on_time;
                end

                success(row) = result.success;
                marker_visited(row) = result.marker_visited;
                travel_distance(row) = result.travel_distance;
                terminal_uncertainty(row) = result.terminal_uncertainty;
                total_cost(row) = result.total_cost;
                path{row} = result.path;
                P_goal{row} = result.P_goal;
            end

            completed_pairs = completed_pairs + 1;
            timeElapsed = toc;

            %% Output Data
            %%%%%%%%%%%%%% All data should be saved here %%%%%%%%%%%%%%%
            % The file name used for save the data
            % Data is saved in "data" folder
            % Name includes the experiment, environment, baseline N, lambda value, 
            % safety probability, and number of trials.
            sensitivity_results = make_results_table(row, factor, level, ...
                level_value, marker_x, marker_y, seed, marker_enabled, ...
                success, marker_visited, travel_distance, ...
                terminal_uncertainty, total_cost, planning_time, ...
                roadmap_generation_time);

            save(savename, 'sensitivity_results', 'path', 'P_goal', ...
                'completed_pairs', 'total_pairs', 'experiment_complete', ...
                'timeElapsed', 'number_of_trials', 'random_seeds', ...
                'lambda_values', 'marker_covariance_values', ...
                'marker_location_values', 'N', 'connection_radius', ...
                'lambda', 'F', 'R', 'robot_speed', 'chi', 'marker', ...
                'initial_P', ...
                'obstacle_edge', 'obs_polyshape', 'target', 'bound', '-v7.3')

            fprintf(['%s level %d/%d, trial %d/%d: ', ...
                'marker absent = %d, marker present = %d\n'], ...
                current_factor, level_index, number_of_levels(factor_index), ...
                trial, number_of_trials, off_result.success, on_result.success)
        end
    end
end

%% Sensitivity summary

sensitivity_summary = summarize_sensitivity(sensitivity_results);
experiment_complete = true;
timeElapsed = toc;

save(savename, 'sensitivity_results', 'sensitivity_summary', ...
    'path', 'P_goal', 'completed_pairs', 'total_pairs', ...
    'experiment_complete', 'timeElapsed', 'number_of_trials', ...
    'random_seeds', 'lambda_values', 'marker_covariance_values', ...
    'marker_location_values', ...
    'N', 'connection_radius', 'lambda', 'F', 'R', 'robot_speed', ...
    'initial_P', ...
    'chi', 'marker', 'obstacle_edge', 'obs_polyshape', 'target', ...
    'bound', '-v7.3')

fprintf('\nSensitivity analysis complete. Data saved to %s.mat\n', savename)

function path = find_target_path(node, distance, predecessor, target)
number_of_nodes = numel(node);
all_rec = reshape([node.ellipse_rect], [4, number_of_nodes]).';
in_target = all_rec(:,1) >= target(1,1) & ...
    all_rec(:,1) + all_rec(:,3) <= target(1,2) & ...
    all_rec(:,2) >= target(2,1) & ...
    all_rec(:,2) + all_rec(:,4) <= target(2,2);
target_ID = find(in_target & isfinite(distance));

path = [];
if ~isempty(target_ID)
    [~, target_index] = min(distance(target_ID));
    path = reconstruct_prm_path(predecessor, target_ID(target_index), 1);
end
end

function result = calculate_result(node, path, lambda, marker)
result.success = ~isempty(path);
result.path = path;
result.travel_distance = nan;
result.terminal_uncertainty = nan;
result.total_cost = nan;
result.P_goal = nan(2);
result.marker_visited = false;

if isempty(path)
    return
end

positions = reshape([node(path).x], 2, []).';
result.travel_distance = sum(vecnorm(diff(positions,1,1), 2, 2));
result.P_goal = node(path(end)).P;
result.terminal_uncertainty = trace(result.P_goal);
result.total_cost = result.travel_distance + lambda * ...
    result.terminal_uncertainty / trace(node(1).P);

for ii = 1:size(positions,1)-1
    if marker_encounter(positions(ii,:), positions(ii+1,:), marker)
        result.marker_visited = true;
        break
    end
end
end

function results = make_results_table(row, factor, level, level_value, ...
    marker_x, marker_y, seed, marker_enabled, success, marker_visited, ...
    travel_distance, terminal_uncertainty, total_cost, planning_time, ...
    roadmap_generation_time)

results = table(factor(1:row), level(1:row), level_value(1:row), ...
    marker_x(1:row), marker_y(1:row), seed(1:row), ...
    marker_enabled(1:row), success(1:row), marker_visited(1:row), ...
    travel_distance(1:row), terminal_uncertainty(1:row), ...
    total_cost(1:row), planning_time(1:row), ...
    roadmap_generation_time(1:row), ...
    'VariableNames', {'factor', 'level', 'level_value', 'marker_x', ...
    'marker_y', 'seed', 'marker_enabled', 'success', ...
    'marker_visited', 'travel_distance', 'terminal_uncertainty', ...
    'total_cost', 'planning_time', 'roadmap_generation_time'});
end

function summary = summarize_sensitivity(results)
factors = unique(results.factor, 'stable');
summary = table();

for ii = 1:numel(factors)
    factor_rows = strcmp(results.factor, factors{ii});
    levels = unique(results.level(factor_rows), 'stable');

    for jj = 1:numel(levels)
        rows = factor_rows & results.level == levels(jj);
        off = rows & ~results.marker_enabled & results.success;
        on = rows & results.marker_enabled & results.success;

        new_row = table(factors(ii), levels(jj), ...
            results.level_value(find(rows,1)), ...
            results.marker_x(find(rows,1)), results.marker_y(find(rows,1)), ...
            sum(off), sum(on), mean(results.travel_distance(off)), ...
            std(results.travel_distance(off)), ...
            mean(results.travel_distance(on)), ...
            std(results.travel_distance(on)), ...
            mean(results.terminal_uncertainty(off)), ...
            std(results.terminal_uncertainty(off)), ...
            mean(results.terminal_uncertainty(on)), ...
            std(results.terminal_uncertainty(on)), ...
            mean(results.total_cost(off)), std(results.total_cost(off)), ...
            mean(results.total_cost(on)), std(results.total_cost(on)), ...
            'VariableNames', {'factor', 'level', 'level_value', ...
            'marker_x', 'marker_y', 'off_successes', 'on_successes', ...
            'off_travel_mean', 'off_travel_std', 'on_travel_mean', ...
            'on_travel_std', 'off_uncertainty_mean', ...
            'off_uncertainty_std', 'on_uncertainty_mean', ...
            'on_uncertainty_std', 'off_cost_mean', 'off_cost_std', ...
            'on_cost_mean', 'on_cost_std'});
        summary = [summary; new_row];
    end
end
end
