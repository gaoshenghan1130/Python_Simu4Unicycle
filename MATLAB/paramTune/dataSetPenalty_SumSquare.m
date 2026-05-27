function penalty = dataSetPenalty_SumSquare(targetData, simData)

targetT = targetData(:,1);
targetZ = targetData(:,2:end);

simT = simData(:,1);
simZ = simData(:,2:end);

ts = linspace(1, 15, 100);

penalty = 0;

for i = 1:numel(ts)
    tnow = ts(i);

    [~, idxTarget] = min(abs(targetT - tnow));
    [~, idxSim] = min(abs(simT - tnow));

    zTarget = targetZ(idxTarget, :);
    zSim = simZ(idxSim, :);

    err = zTarget - zSim;

    penalty = penalty + sum(err.^2);

end

penalty = penalty / numel(ts);

end