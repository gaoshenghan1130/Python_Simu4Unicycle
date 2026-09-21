function test_damping_study()
% Run run_damping_speed_study first to create the numerical reference data.
root=fileparts(fileparts(mfilename('fullpath'))); addpath(genpath(root));
audit=load(fullfile(root,'results','damping_speed_study','study.mat'));
p=audit.p; d=audit.d; d.forward_speed=0.2;
[K,info]=pole_placement(p,d); p0=p; p0.BR=0;
[K0,~]=pole_placement(p0,d); expected=zeros(2,12); expected(1,[1 4])=p.BR*[p.R 1];
damping_gain_error=norm(K0-K-expected,inf); assert(damping_gain_error<1e-8);
refinement=struct();
for scenario=1:2
    base=audit.runs{scenario,3}; s=base.settings;
    s.max_step=s.max_step/2; s.output_dt=s.output_dt/2;
    s.rel_tol=s.rel_tol/10; s.abs_tol=s.abs_tol/10;
    refined=simulate_unicycle(p,base.controller,s);
    if scenario==1
        refinement.tilt_time_error=abs(refined.event_time(end)-base.event_time(end));
        assert(refinement.tilt_time_error<1e-3);
    else
        refinement.final_state_error=norm(refined.X(end,:)-base.X(end,:),inf);
        assert(refinement.final_state_error<1e-5);
    end
end
fid=fopen(fullfile(root,'results','damping_speed_study','verification.json'),'w');
fprintf(fid,'%s',jsonencode(struct('damping_gain_error',damping_gain_error,'refinement',refinement))); fclose(fid);
fprintf('Damping audit and regression checks passed.\n');

end
