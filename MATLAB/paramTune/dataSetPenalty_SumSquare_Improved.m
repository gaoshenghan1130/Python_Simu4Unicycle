function penalty = dataSetPenalty_SumSquare_Improved(targetData, simData)
targetT = targetData(:,1);
targetZ = targetData(:,2:end);
simT = simData(:,1);
simZ = simData(:,2:end);

ts = [linspace(1, 7.5, 10), linspace(5, 7.5, 100)];

weights = [10, 1/0.5 * 10, 1/50 * pi/180,1/170 * pi/180]; % x, dx, gamma, dgamma

delay = 0.2; % 20ms for i2c

% use interpolation to get values at the same time points
zTarget_interp = interp1(targetT, targetZ, ts, 'linear', 'extrap');
zSim_interp = interp1(simT + delay, simZ, ts, 'linear', 'extrap');

penalty = 0;
for i = 1:numel(ts)
    err = zTarget_interp(i, :) - zSim_interp(i, :);
    
    weighted_err = (err.^2) .* weights;
    penalty = penalty + sum(weighted_err);
end

penalty = penalty / numel(ts);
end