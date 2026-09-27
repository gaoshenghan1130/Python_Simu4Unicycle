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
        if strcmp(e.sweep.parameter,'speed')
            label=sprintf('initial = target speed = %.6g m/s',values(k));
        end
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
% A single physical speed shared by the initial state and rolling reference.
if strcmp(e.sweep.parameter,'speed')
    if ~isempty(e.sweep.index)
        error('unicycle:SweepIndex','The matched speed sweep is scalar; use index=[].');
    end
    if isfield(e.controller,'hold_gamma_zero') && e.controller.hold_gamma_zero
        error('unicycle:InactiveSweep','Matched target speed requires the longitudinal feedback, not gamma hold.');
    end
    validateattributes(value,{'numeric'},{'scalar','real','finite','nonzero'});
    % Reuse design-field validation: rolling pole placement must be active.
    trial=e; trial.sweep.parameter='design.forward_speed';
    trial=apply_sweep(trial,value);
    e.design=trial.design;
    e.settings.forward_speed0=value;
    return;
end
parts=strsplit(char(e.sweep.parameter),'.');
if numel(parts)~=2 || ~ismember(parts{1},{'parameters','controller','settings','design'}) ...
        || ~isfield(e.(parts{1}),parts{2})
    error('unicycle:SweepField','Use an existing parameters/controller/settings/design.field.');
end
if strcmp(parts{1},'design') && isfield(e.design,'lateral_states')
    selection=lower(char(e.design.lateral_states));
    rolling=ismember(selection,{'balance_rolling','balance_chi','balance_chi_epsilon'});
    unused=(rolling && strcmp(parts{2},'lateral')) || ...
        (~rolling && strcmp(parts{2},'forward_speed')) || ...
        (strcmp(parts{2},'rolling_poles') && ~strcmp(selection,'balance_rolling')) || ...
        (strcmp(parts{2},'chi_poles') && ~strcmp(selection,'balance_chi')) || ...
        (strcmp(parts{2},'chi_epsilon_poles') && ~strcmp(selection,'balance_chi_epsilon'));
    if unused
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
    if strcmp(mode,'lateral_open_loop') && ...
            ~ismember(parts{2},{'kp_gamma','kd_gamma','gamma_ref','gamma_dot_ref','torque_limit'})
        error('unicycle:InactiveSweep','Lateral-open-loop mode uses only gamma PD and torque_limit.');
    end
    if strcmp(mode,'open_loop')
        error('unicycle:InactiveSweep','Open loop has zero input; controller parameter sweeps have no effect.');
    end
    if strcmp(mode,'pole_placement') && ismember(parts{2},{'K','x_ref'})
        error('unicycle:GeneratedSweep','Pole placement regenerates K and x_ref; sweep design poles instead.');
    end
    if (~ismember(mode,{'pd','lateral_open_loop'}) && ~ismember(parts{2},{'K','x_ref','force_limit','torque_limit'})) ...
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
