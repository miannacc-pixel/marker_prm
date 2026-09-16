% PLOT_MARKER_OVERLAY
% Plots without marker and with marker paths from two saved main.m runs
% on the same axis. Run main.m once with marker_enabled = false and once
% with marker_enabled = true before running this script.

clear
close all
clc

%% Select the two main.m data files

% Change filenames when plotting a different environment.
marker_off_file = ...
    'data/main_cavern_without_marker_N2000_lambda_02_safety_08';
marker_on_file = ...
    'data/main_cavern_with_marker_N2000_lambda_02_safety_08';

off_run = load(marker_off_file);
on_run = load(marker_on_file);

%% Check that both files can be plotted together

required_variables = {'node', 'saver', 'bound', 'target', ...
    'obstacle_edge', 'R', 'chi', 'num_props', 'lambda'};

for ii = 1:numel(required_variables)
    if ~isfield(off_run, required_variables{ii}) || ...
            ~isfield(on_run, required_variables{ii})
        error('Both data files must contain %s.', required_variables{ii})
    end
end

path_off = off_run.saver(end).path;
path_on = on_run.saver(end).path;

if isempty(path_off) || isempty(path_on)
    error(['Both saved runs must contain a feasible path. ', ...
        'Without-marker path empty: %d; with-marker path empty: %d.'], ...
        isempty(path_off), isempty(path_on))
end

off_obstacles = edges_to_polygons(off_run.obstacle_edge);
on_obstacles = edges_to_polygons(on_run.obstacle_edge);

if ~isequal(off_run.bound, on_run.bound) || ...
        ~isequal(off_run.target, on_run.target) || ...
        ~same_obstacles(off_obstacles, on_obstacles)
    error(['The marker-disabled and marker-enabled files use different ', ...
        'environment definitions.'])
end

if ~isfield(on_run, 'marker')
    error('The marker-enabled data file does not contain marker settings.')
end

off_positions = reshape([off_run.node.x], 2, []).';
on_positions = reshape([on_run.node.x], 2, []).';
if ~isequal(off_positions, on_positions)
    warning(['The two runs used different sampled roadmaps. The paths can ', ...
        'still be plotted together, but a paired comparison should use ', ...
        'the same rng seed before roadmap sampling in both main.m runs.'])
end

%% Plot both planner outputs on one axis

path_off_color = [0.80, 0.16, 0.14];
path_on_color = [0.00, 0.35, 0.80];
marker_color = [0.4940, 0.1840, 0.5560];
obstacle_color = [187/255, 139/255, 172/255];
goal_fill_color = [0.80, 1.00, 0.80];
goal_edge_color = [0.00, 0.50, 0.00];

fig_f = figure('Color', 'w');
fig_f.Position = [150, 90, 760, 710];
ax = axes(fig_f);
hold(ax, 'on')
axis(ax, 'equal')
xlim(ax, off_run.bound(1).x)
ylim(ax, off_run.bound(2).x)
grid(ax, 'on')
box(ax, 'on')
set(ax, 'FontName', 'Arial', 'FontSize', 12, 'LineWidth', 1.2)

% Plot the selected environment.
for ii = 1:numel(off_obstacles)
    fill(ax, off_obstacles(ii).x, off_obstacles(ii).y, ...
        obstacle_color, 'EdgeColor', [0.15, 0.15, 0.15], ...
        'LineWidth', 1)
end

% Plot the target region.
goal_x = [off_run.target(1,1), off_run.target(1,2), ...
    off_run.target(1,2), off_run.target(1,1)];
goal_y = [off_run.target(2,1), off_run.target(2,1), ...
    off_run.target(2,2), off_run.target(2,2)];
goal_handle = patch(ax, goal_x, goal_y, goal_fill_color, ...
    'EdgeColor', goal_edge_color, 'LineWidth', 1.5);

% Plot the marker and its sensing region.
theta = linspace(0, 2*pi, 160);
range_handle = plot(ax, ...
    on_run.marker.x(1) + on_run.marker.sensing_radius*cos(theta), ...
    on_run.marker.x(2) + on_run.marker.sensing_radius*sin(theta), '--', ...
    'Color', marker_color, 'LineWidth', 1.5);
marker_handle = plot(ax, on_run.marker.x(1), on_run.marker.x(2), 'p', ...
    'Color', marker_color, 'MarkerFaceColor', marker_color, ...
    'MarkerSize', 13);

% Plot the results without and with the marker.
path_off_handle = plot_path_and_covariance(ax, off_run.node, path_off, ...
    off_run.R, off_run.chi, off_run.num_props, [], false, path_off_color);
path_on_handle = plot_path_and_covariance(ax, on_run.node, path_on, ...
    on_run.R, on_run.chi, on_run.num_props, on_run.marker, true, ...
    path_on_color);

start = off_run.node(1).x;
start_handle = plot(ax, start(1), start(2), 'o', 'Color', [0.85, 0, 0], ...
    'MarkerFaceColor', [0.85, 0, 0], 'MarkerSize', 8);

xlabel(ax, 'Location X [m]')
ylabel(ax, 'Location Y [m]')

% Change for different plots
title('Cavern environment')

legend(ax, [path_off_handle, path_on_handle, range_handle, ...
    marker_handle, start_handle, goal_handle], ...
    {'Without marker', 'With marker', 'Marker sensing range', ...
    'Marker', 'Start', 'Goal'}, 'Location', 'best', 'FontSize', 9)

%% Save the overlay figure and display path metrics

[result_folder, result_name] = fileparts(marker_on_file);
overlay_name = strrep(result_name, 'main_', 'overlay_');
overlay_name = strrep(overlay_name, '_with_marker', '');
exportgraphics(fig_f, fullfile(result_folder, [overlay_name, '.png']), ...
    'Resolution', 300)

off_metrics = calculate_path_metrics(off_run.node, path_off, off_run.lambda);
on_metrics = calculate_path_metrics(on_run.node, path_on, on_run.lambda);

fprintf(['Without marker: Dtravel = %.6f m, tr(Pgoal) = %.6e, ', ...
    'Dtotal = %.6f\n'], off_metrics.travel, ...
    off_metrics.trace_goal, off_metrics.total)
fprintf(['With marker   : Dtravel = %.6f m, tr(Pgoal) = %.6e, ', ...
    'Dtotal = %.6f\n'], on_metrics.travel, ...
    on_metrics.trace_goal, on_metrics.total)

function trajectory_handle = plot_path_and_covariance(ax, node, path, ...
    R, chi, num_props, marker, marker_enabled, color)

marker_has_been_sensed = false;

for ii = 1:numel(path)-1
    x0 = node(path(ii)).x;
    x1 = node(path(ii+1)).x;
    P0 = node(path(ii)).P;
    edge_distance = norm(x1-x0);
    edge_senses_marker = false;

    if marker_enabled && ~marker_has_been_sensed
        [edge_senses_marker, marker_fraction] = ...
            marker_encounter(x0, x1, marker);
        if edge_senses_marker
            P_before_marker = P0 + R*(marker_fraction*edge_distance);
            P_after_marker = ekf_update_covariance(P_before_marker, ...
                marker.H, marker.R);
        end
    end

    for jj = 0:num_props
        edge_fraction = jj/num_props;
        position = x0 + edge_fraction*(x1-x0);

        if edge_senses_marker && edge_fraction > marker_fraction
            P = P_after_marker + ...
                R*((edge_fraction-marker_fraction)*edge_distance);
        else
            P = P0 + R*(edge_fraction*edge_distance);
        end

        draw_covariance_ellipse(ax, position, P, chi, color)
    end

    if edge_senses_marker
        marker_has_been_sensed = true;
    end
end

positions = reshape([node(path).x], 2, []).';
trajectory_handle = plot(ax, positions(:,1), positions(:,2), '-', ...
    'Color', color, 'LineWidth', 2.2);
plot(ax, positions(:,1), positions(:,2), 'o', 'Color', color, ...
    'MarkerFaceColor', color, 'MarkerSize', 3.5)
end

function draw_covariance_ellipse(ax, position, P, chi, color)
[ra, rb, angle] = error_ellipse(position, P, chi);
theta = linspace(0, 2*pi, 80);
ellipse = [ra*cos(theta); rb*sin(theta)];
rotation = [cos(angle), sin(angle); -sin(angle), cos(angle)];
ellipse = rotation*ellipse;
plot(ax, ellipse(1,:)+position(1), ellipse(2,:)+position(2), '-', ...
    'Color', color, 'LineWidth', 0.65)
end

function metrics = calculate_path_metrics(node, path, lambda)
positions = reshape([node(path).x], 2, []).';
metrics.travel = sum(vecnorm(diff(positions,1,1), 2, 2));
metrics.trace_goal = trace(node(path(end)).P);
metrics.total = metrics.travel + lambda*metrics.trace_goal/trace(node(path(1)).P);
end

function match = same_obstacles(first, second)
match = numel(first) == numel(second);
if ~match
    return
end

for ii = 1:numel(first)
    if ~isequal(first(ii).x, second(ii).x) || ...
            ~isequal(first(ii).y, second(ii).y)
        match = false;
        return
    end
end
end

function obstacles = edges_to_polygons(obstacle_edge)
obstacles = struct('x', {}, 'y', {});
current_vertices = zeros(0,2);

for ii = 1:numel(obstacle_edge)
    if isempty(current_vertices)
        first_vertex = obstacle_edge(ii).start;
    end

    current_vertices(end+1,:) = obstacle_edge(ii).start; %#ok<AGROW>

    if isequal(obstacle_edge(ii).end, first_vertex)
        obstacle_index = numel(obstacles)+1;
        obstacles(obstacle_index).x = current_vertices(:,1).';
        obstacles(obstacle_index).y = current_vertices(:,2).';
        current_vertices = zeros(0,2);
    end
end

if ~isempty(current_vertices)
    error('The saved obstacle edges do not form closed polygons.')
end
end
