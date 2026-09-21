function results = run_experiment(e)
% Apply one scalar sweep value, generate controller, then simulate each case.
if e.sweep.enabled
    validateattributes(e.sweep.values,{'numeric'},{'real','finite','vector','nonempty'});
    values=e.sweep.values(:).';
else
    values=NaN;
end
runs=cell(1,numel(values));
for k=1:numel(values)
    run=e;
    if e.sweep.enabled
        run=apply_sweep(run,values(k));
        label=sprintf('%s = %.6g',e.sweep.parameter,values(k));
        if ~isempty(e.sweep.index)
            label=sprintf('%s(%d) = %.6g',e.sweep.parameter,e.sweep.index,values(k));
        end
    else
        label=char(run.controller.mode);
    end
    fprintf('Run %d/%d: %s [%s]\n',k,numel(values),label,run.controller.mode);
    c=generate_controller(run.parameters,run.controller,run.design);
    r=simulate_unicycle(run.parameters,c,run.settings);
    r.label=label;
    r.sweep_parameter=e.sweep.parameter; r.sweep_value=values(k);
    r.design=run.design;
    runs{k}=r;
end
results=[runs{:}];
end

function e=apply_sweep(e,value)
parts=strsplit(char(e.sweep.parameter),'.');
if numel(parts)~=2 || ~ismember(parts{1},{'parameters','controller','settings','design'}) ...
        || ~isfield(e.(parts{1}),parts{2})
    error('unicycle:SweepField','Use an existing parameters/controller/settings/design.field.');
end
if strcmp(parts{1},'design') && isfield(e.design,'lateral_states')
    rolling=strcmpi(e.design.lateral_states,'balance_chi');
    if (rolling && strcmp(parts{2},'lateral')) || ...
       (~rolling && ismember(parts{2},{'chi_poles','forward_speed'}))
        error('unicycle:InactiveSweep','Selected design field is unused by this lateral state selection.');
    end
end
field=e.(parts{1}).(parts{2});
if ~isnumeric(field) || isempty(field)
    error('unicycle:SweepType','Sweep target must be a nonempty numeric field.');
end
mode=lower(char(e.controller.mode));
if strcmp(parts{1},'design') && ~strcmp(mode,'pole_placement')
    error('unicycle:InactiveSweep','design sweeps require pole_placement mode.');
end
if strcmp(parts{1},'controller')
    if strcmp(mode,'pole_placement') && ismember(parts{2},{'K','x_ref'})
        error('unicycle:GeneratedSweep','Pole placement regenerates K and x_ref; sweep design poles instead.');
    end
    if (~strcmp(mode,'pd') && ~ismember(parts{2},{'K','x_ref','force_limit','torque_limit'})) ...
       || (strcmp(mode,'pd') && ismember(parts{2},{'K','x_ref'}))
        error('unicycle:InactiveSweep','Selected controller field is unused in this mode.');
    end
end
if isempty(e.sweep.index)
    if ~isscalar(field), error('unicycle:SweepIndex','Set sweep.index for a nonscalar field.'); end
    field=value;
else
    validateattributes(e.sweep.index,{'numeric'},{'scalar','integer','positive','<=',numel(field)});
    field(e.sweep.index)=value;
end
e.(parts{1}).(parts{2})=field;
end
