function newPar = randomTuneGenerator( ...
    par, ...
    fields, ...
    stepsize, ...
    loopNum)

newPar = par;

loopNum;

dir = randn(1, numel(fields));

dir = dir / norm(dir);

for i = 1:numel(fields)

    f = fields{i};

    currentVal = par.(f);

    scale = exp(stepsize * dir(i));

    newVal = currentVal * scale;

    newPar.(f) = newVal;

end

end