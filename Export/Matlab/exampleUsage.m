load('experiment_velocity_segments.mat');

figure('Color', 'w'); hold on;
colors = lines(length(all_velocity_segments));

for i = 1:length(all_velocity_segments)
    seg = all_velocity_segments{i};
    data = seg.data;

    % data format [Timestamp,Theta_rad,Gamma_rad,Phi_rad,Gamma_deg]
    
    t = data(:, 1); 
    t = t - t(1);
    x_c = data(:, 4) * 0.2527; % phi times R
    
    plot(t, x_c, 'Color', colors(i,:), 'LineWidth', 1.2, ...
        'DisplayName', sprintf('Seg %d (Target: %.1f)', i, seg.target_value));
end

xlabel('Time (s)'); ylabel('x_c'); 
title('Velocity Mode Segments from Multiple Files');
legend('Location', 'northeastoutside'); grid on;