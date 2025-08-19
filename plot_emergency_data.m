function plot_emergency_data(filename)
    % PLOT_EMERGENCY_DATA - Load and plot emergency exoskeleton data in MATLAB
    % 
    % Usage: plot_emergency_data('emergency_plot_data_YYYYMMDD_HHMMSS.mat')
    %        plot_emergency_data() - opens file dialog to select file
    
    if nargin < 1
        [file, path] = uigetfile('*.mat', 'Select Emergency Data File');
        if isequal(file, 0)
            return;
        end
        filename = fullfile(path, file);
    end
    
    % Load the data
    try
        data = load(filename);
        fprintf('Loaded emergency data from: %s\n', filename);
        fprintf('Timestamp: %s\n', data.timestamp);
        fprintf('Buffer size: %.0f samples\n', data.buffer_size);
    catch ME
        error('Failed to load data file: %s', ME.message);
    end
    
    % Create figure with same layout as Python node (2x3 grid)
    figure('Position', [100, 100, 1400, 800], 'Color', 'k');
    set(gcf, 'Name', 'Emergency Exoskeleton Data Visualization', 'NumberTitle', 'off');
    
    % Define subplot positions (2 rows, 3 columns)
    % Top row: Hip Position, Hip Velocity, Hip Torques
    % Bottom row: Knee Position, Knee Velocity, Knee Torques
    
    % Plot 1: Right Hip Position (top-left)
    subplot(2, 3, 1);
    hold on;
    if ~isempty(data.trajectory_time) && ~isempty(data.rhip_pos_ref)
        plot(data.trajectory_time, data.rhip_pos_ref, 'b-', 'LineWidth', 1, 'DisplayName', 'Right Hip Position Ref');
    end
    if ~isempty(data.state_time) && ~isempty(data.rhip_pos_current)
        plot(data.state_time, data.rhip_pos_current, 'b--', 'LineWidth', 1, 'DisplayName', 'Right Hip Position Current');
    end
    title('Right Hip Position (deg)', 'Color', 'w');
    xlabel('Time (s)', 'Color', 'w');
    ylabel('Position (deg)', 'Color', 'w');
    legend('Location', 'best', 'TextColor', 'w');
    grid on;
    set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.3]);
    
    % Plot 2: Right Hip Velocity (top-center)
    subplot(2, 3, 2);
    hold on;
    if ~isempty(data.trajectory_time) && ~isempty(data.rhip_vel_ref)
        plot(data.trajectory_time, data.rhip_vel_ref, 'g-', 'LineWidth', 1, 'DisplayName', 'Right Hip Velocity Ref');
    end
    if ~isempty(data.state_time) && ~isempty(data.rhip_vel_current)
        plot(data.state_time, data.rhip_vel_current, 'g--', 'LineWidth', 1, 'DisplayName', 'Right Hip Velocity Current');
    end
    title('Right Hip Velocity (deg/s)', 'Color', 'w');
    xlabel('Time (s)', 'Color', 'w');
    ylabel('Velocity (deg/s)', 'Color', 'w');
    legend('Location', 'best', 'TextColor', 'w');
    grid on;
    set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.3]);
    
    % Plot 3: Right Hip Torques (top-right)
    subplot(2, 3, 3);
    hold on;
    if ~isempty(data.torque_time) && ~isempty(data.rhip_ff_torque)
        plot(data.torque_time, data.rhip_ff_torque, 'c-', 'LineWidth', 1, 'DisplayName', 'Hip FF Torque');
    end
    if ~isempty(data.torque_time) && ~isempty(data.rhip_motor_torque)
        plot(data.torque_time, data.rhip_motor_torque, 'Color', [1 0.5 0], 'LineStyle', '--', 'LineWidth', 1, 'DisplayName', 'Hip Motor Torque');
    end
    title('Right Hip Torques (N⋅m)', 'Color', 'w');
    xlabel('Time (s)', 'Color', 'w');
    ylabel('Torque (N⋅m)', 'Color', 'w');
    legend('Location', 'best', 'TextColor', 'w');
    grid on;
    set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.3]);
    
    % Plot 4: Right Knee Position (bottom-left)
    subplot(2, 3, 4);
    hold on;
    if ~isempty(data.trajectory_time) && ~isempty(data.rknee_pos_ref)
        plot(data.trajectory_time, data.rknee_pos_ref, 'r-', 'LineWidth', 1, 'DisplayName', 'Right Knee Position Ref');
    end
    if ~isempty(data.state_time) && ~isempty(data.rknee_pos_current)
        plot(data.state_time, data.rknee_pos_current, 'r--', 'LineWidth', 1, 'DisplayName', 'Right Knee Position Current');
    end
    title('Right Knee Position (deg)', 'Color', 'w');
    xlabel('Time (s)', 'Color', 'w');
    ylabel('Position (deg)', 'Color', 'w');
    legend('Location', 'best', 'TextColor', 'w');
    grid on;
    set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.3]);
    
    % Plot 5: Right Knee Velocity (bottom-center)
    subplot(2, 3, 5);
    hold on;
    if ~isempty(data.trajectory_time) && ~isempty(data.rknee_vel_ref)
        plot(data.trajectory_time, data.rknee_vel_ref, 'm-', 'LineWidth', 1, 'DisplayName', 'Right Knee Velocity Ref');
    end
    if ~isempty(data.state_time) && ~isempty(data.rknee_vel_current)
        plot(data.state_time, data.rknee_vel_current, 'm--', 'LineWidth', 1, 'DisplayName', 'Right Knee Velocity Current');
    end
    title('Right Knee Velocity (deg/s)', 'Color', 'w');
    xlabel('Time (s)', 'Color', 'w');
    ylabel('Velocity (deg/s)', 'Color', 'w');
    legend('Location', 'best', 'TextColor', 'w');
    grid on;
    set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.3]);
    
    % Plot 6: Right Knee Torques (bottom-right)
    subplot(2, 3, 6);
    hold on;
    if ~isempty(data.torque_time) && ~isempty(data.rknee_ff_torque)
        plot(data.torque_time, data.rknee_ff_torque, 'y-', 'LineWidth', 1, 'DisplayName', 'Knee FF Torque');
    end
    if ~isempty(data.torque_time) && ~isempty(data.rknee_motor_torque)
        plot(data.torque_time, data.rknee_motor_torque, 'Color', [0.5 0 0.5], 'LineStyle', '--', 'LineWidth', 1, 'DisplayName', 'Knee Motor Torque');
    end
    title('Right Knee Torques (N⋅m)', 'Color', 'w');
    xlabel('Time (s)', 'Color', 'w');
    ylabel('Torque (N⋅m)', 'Color', 'w');
    legend('Location', 'best', 'TextColor', 'w');
    grid on;
    set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.3]);
    
    % Add main title
    sgtitle('Real-time Exoskeleton System Visualization - Emergency Data', 'Color', 'w', 'FontSize', 16);
    
    % Print data summary
    fprintf('\n=== Data Summary ===\n');
    fprintf('Trajectory samples: %d\n', length(data.trajectory_time));
    fprintf('State samples: %d\n', length(data.state_time));
    fprintf('Torque samples: %d\n', length(data.torque_time));
    
    if ~isempty(data.trajectory_time)
        fprintf('Time range: %.2f to %.2f seconds (%.2f duration)\n', ...
            min(data.trajectory_time), max(data.trajectory_time), ...
            max(data.trajectory_time) - min(data.trajectory_time));
    end
end