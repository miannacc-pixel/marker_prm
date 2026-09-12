clear
close all
clc

% PLOT_SENSITIVITY_ANALYSIS
% Loads the completed sensitivity-analysis data and generates figures

result_file = ...
    'data/sensitivity_baseline_N2000_lambda_02_safety_08_trials_5.mat';
load(result_file, 'sensitivity_summary', 'initial_P')

% Each figure shows mean +/- standard deviation for the marker-disabled
% and marker-enabled planners. Figures are saved in the "data" folder.
plot_sensitivity_figure(sensitivity_summary, initial_P, 'lambda', ...
    'Uncertainty weight, \lambda', false, ...
    'data/sensitivity_lambda.png');

plot_sensitivity_figure(sensitivity_summary, initial_P, 'sensing_radius', ...
    'Marker sensing radius, r_m [m]', false, ...
    'data/sensitivity_sensing_radius.png');

plot_sensitivity_figure(sensitivity_summary, initial_P, 'marker_covariance', ...
    'Marker measurement covariance scalar', true, ...
    'data/sensitivity_marker_covariance.png');

plot_sensitivity_figure(sensitivity_summary, initial_P, 'roadmap_size', ...
    'Roadmap size, N', false, ...
    'data/sensitivity_roadmap_size.png');

plot_sensitivity_figure(sensitivity_summary, initial_P, 'marker_location', ...
    'Marker location', false, ...
    'data/sensitivity_marker_location.png');

function plot_sensitivity_figure(summary, initial_P, factor_name, x_label, ...
    use_log_scale, output_name)

rows = strcmp(summary.factor, factor_name);
data = summary(rows,:);

if isempty(data)
    return
end

x = data.level_value;
x_tick_labels = {};
if strcmp(factor_name, 'marker_location')
    x = (1:height(data)).';
    x_tick_labels = arrayfun(@(ii) sprintf('(%.2f, %.2f)', ...
        data.marker_x(ii), data.marker_y(ii)), 1:height(data), ...
        'UniformOutput', false);
end

marker_off_color = [0.35, 0.35, 0.35];
marker_on_color = [0.4940, 0.1840, 0.5560];

figure_handle = figure('Color', 'w');
figure_handle.Position = [150, 150, 1050, 320];
figure_handle.ToolBar = 'none';
tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact')

nexttile
hold on
errorbar(x, data.off_uncertainty_mean/trace(initial_P), ...
    data.off_uncertainty_std/trace(initial_P), '-o', ...
    'Color', marker_off_color, 'MarkerFaceColor', marker_off_color, ...
    'LineWidth', 1.5, 'MarkerSize', 6)
errorbar(x, data.on_uncertainty_mean/trace(initial_P), ...
    data.on_uncertainty_std/trace(initial_P), '-s', ...
    'Color', marker_on_color, 'MarkerFaceColor', marker_on_color, ...
    'LineWidth', 1.5, 'MarkerSize', 6)
xlabel(x_label)
ylabel('tr(P_{goal})/tr(P_{start})')
title('Terminal uncertainty')
legend({'Without marker', 'With marker'}, ...
    'Location', 'best', 'FontSize', 9)
format_sensitivity_axis(use_log_scale, x, x_tick_labels)
ylim([0, 5])

nexttile
hold on
errorbar(x, data.off_travel_mean, data.off_travel_std, '-o', ...
    'Color', marker_off_color, 'MarkerFaceColor', marker_off_color, ...
    'LineWidth', 1.5, 'MarkerSize', 6)
errorbar(x, data.on_travel_mean, data.on_travel_std, '-s', ...
    'Color', marker_on_color, 'MarkerFaceColor', marker_on_color, ...
    'LineWidth', 1.5, 'MarkerSize', 6)
xlabel(x_label)
ylabel('D_{travel} [m]')
title('Travel distance')
format_sensitivity_axis(use_log_scale, x, x_tick_labels)
ylim([0, 3])

nexttile
hold on
errorbar(x, data.off_cost_mean, data.off_cost_std, '-o', ...
    'Color', marker_off_color, 'MarkerFaceColor', marker_off_color, ...
    'LineWidth', 1.5, 'MarkerSize', 6)
errorbar(x, data.on_cost_mean, data.on_cost_std, '-s', ...
    'Color', marker_on_color, 'MarkerFaceColor', marker_on_color, ...
    'LineWidth', 1.5, 'MarkerSize', 6)
xlabel(x_label)
ylabel('D_{total}')
title('Total objective')
format_sensitivity_axis(use_log_scale, x, x_tick_labels)
ylim([0, 3])

exportgraphics(figure_handle, output_name, 'Resolution', 300)
end

function format_sensitivity_axis(use_log_scale, x, x_tick_labels)
axis_handle = gca;
set(axis_handle, 'FontName', 'Arial', 'FontSize', 10, 'LineWidth', 1)
axis_handle.Toolbar.Visible = 'off';
grid on
box on

if use_log_scale
    set(gca, 'XScale', 'log')
end

if ~isempty(x_tick_labels)
    xticks(x)
    xticklabels(x_tick_labels)
    xtickangle(25)
end
end
