function result = run_study(name,varargin)
% MAIN 3: Run a focused study. run_study('list') lists available names.
% Example: report=run_study('manifold_hardware',-1.5,false);
% Script-based studies keep editable parameters in studies/<name>.m.
if nargin<1, name='manifold_hardware'; end
root=fileparts(mfilename('fullpath'));
addpath(fullfile(root,'studies'));
names={'equilibrium_balance','mate_speed_relinearization', ...
    'allowable_theta_vs_speed','manifold_hardware', ...
    'manifold_hardware_tuning','manifold_overshoot','zero_yaw_manifold', ...
    'direct_upright_turning'};
name=validatestring(name,[names {'list'}],mfilename,'name');
if strcmp(name,'list')
    result=names;
    disp(string(names(:)));
    return;
end
switch name
    case 'manifold_hardware'
        result=run_manifold_hardware(varargin{:});
    case 'manifold_hardware_tuning'
        result=run_manifold_hardware_tuning(varargin{:});
    case 'manifold_overshoot'
        result=run_manifold_overshoot_study(varargin{:});
    case 'direct_upright_turning'
        assert(isempty(varargin),'This study takes no arguments.');
        result=verify_direct_upright_turning();
    otherwise
        assert(isempty(varargin), ...
            'Edit parameters in the studies/ script for this study.');
        switch name
            case 'equilibrium_balance'
                run_equilibrium_balance_study;
                result=report;
            case 'mate_speed_relinearization'
                run_mate_speed_relinearization;
                result=struct('runs',{runs},'summary',summary);
            case 'allowable_theta_vs_speed'
                run_allowable_theta_vs_speed;
                result=struct('summary',summary,'boundary_runs',{boundary_runs});
            case 'zero_yaw_manifold'
                run_zero_yaw_manifold_study;
                result=report;
        end
end
end
