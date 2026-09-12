% PLOT_MARKER_COMPARISON Compare marker-disabled and marker-enabled runs.
%
% This script loads one PRM result without the marker and a 
% matching result with the marker, then compares terminal 
% uncertainty, path length, and the exact planning objective 
% Dtotal = Dtravel + lambda*trace(Pgoal)/trace(Pstart).
%
% Run main.m once with marker_enabled = false and once with
% marker_enabled = true before running this script. Change the two
% filenames below when comparing a different pair of runs.

clear
close all

marker_off_file = ...
    'data/main_baseline_without_marker_N2000_lambda_02_safety_08.mat';
marker_on_file = ...
    'data/main_baseline_with_marker_N2000_lambda_02_safety_08.mat';

off_run = load(marker_off_file);
on_run = load(marker_on_file);

off_metrics = path_metrics(off_run, marker_off_file);
on_metrics = path_metrics(on_run, marker_on_file);

labels = {'Without marker', 'With marker'};
bar_colors = [0.35 0.35 0.35; 0.4940 0.1840 0.5560];

fig_f = figure('Color', 'w');
fig_f.Position = [650 100 900 280];
tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile
make_comparison_bar([off_metrics.normalized_uncertainty, on_metrics.normalized_uncertainty], ...
    labels, bar_colors, 'Terminal uncertainty', ...
    'tr(P_{goal})/tr(P_{start})', '%.3g');
ylim([0, 3])
yticks(0:1:3)

nexttile
make_comparison_bar([off_metrics.travel_distance, on_metrics.travel_distance], ...
    labels, bar_colors, 'Travel distance', 'D_{travel} [m]', '%.3g');
ylim([0, 3])
yticks(0:1:3)

nexttile
make_comparison_bar([off_metrics.total_cost, on_metrics.total_cost], ...
    labels, bar_colors, 'Planning objective', 'D_{total}', '%.3g');
ylim([0, 3])
yticks(0:1:3)

[result_folder, result_name] = fileparts(marker_on_file);
comparison_name = strrep(result_name, 'main_', 'comparison_');
comparison_name = strrep(comparison_name, '_with_marker', '');
exportgraphics(fig_f, fullfile(result_folder, [comparison_name, '.png']), ...
    'Resolution', 300)

fprintf('Without marker: tr(Pgoal) = %.6g, Dtravel = %.6g m, Dtotal = %.6g\n', ...
    off_metrics.trace_P_goal, off_metrics.travel_distance, off_metrics.total_cost);
fprintf('With marker   : tr(Pgoal) = %.6g, Dtravel = %.6g m, Dtotal = %.6g\n', ...
    on_metrics.trace_P_goal, on_metrics.travel_distance, on_metrics.total_cost);

function metrics = path_metrics(run, filename)

if ~isfield(run, 'saver') || isempty(run.saver) || isempty(run.saver(end).path)
    error('plot_marker_comparison:NoPath', ...
        'No feasible path was saved in %s.', filename)
end
if ~isfield(run, 'lambda')
    error('plot_marker_comparison:MissingLambda', ...
        'The saved result %s does not contain lambda.', filename)
end

path = run.saver(end).path;
if numel(path) < 2
    error('plot_marker_comparison:ShortPath', ...
        'The saved path in %s must contain at least two vertices.', filename)
end

positions = reshape([run.node(path).x], 2, []).';
edge_vectors = diff(positions, 1, 1);
metrics.travel_distance = sum(vecnorm(edge_vectors, 2, 2));

P_goal = run.node(path(end)).P;
metrics.trace_P_goal = trace(P_goal);
metrics.normalized_uncertainty = metrics.trace_P_goal / ...
    trace(run.node(path(1)).P);
metrics.total_cost = metrics.travel_distance + ...
    run.lambda * metrics.normalized_uncertainty;
end

function make_comparison_bar(values, labels, colors, plot_title, y_label, value_format)

x_positions = [1, 2.5];
bar_handle = bar(x_positions, values, 0.65, 'FaceColor', 'flat');
bar_handle.CData = colors;
set(gca, 'XTick', x_positions, 'XTickLabel', labels, 'FontName', 'Arial', ...
    'FontSize', 10, 'LineWidth', 1)
xlim([0.4, 3.1])
ylabel(y_label)
grid on
box off

upper_limit = max(values);
if upper_limit <= 0
    upper_limit = 1;
end
ylim([0, 1.15 * upper_limit])

for ii = 1:numel(values)
    text(x_positions(ii), values(ii), sprintf(value_format, values(ii)), ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
        'FontSize', 9)
end
end
