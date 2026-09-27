function e = experiment_settings()
% Base configuration used by simulation_preset and lower-level experiments.
% Daily entry points: run_simulation.m and run_parameter_sweep.m.
e.parameters=model_parameters();
e.controller=controller_parameters();
e.settings=simulation_settings();
e.design=pole_placement_settings();

% Select 'open_loop', 'lateral_open_loop', 'pd', 'direct', or 'pole_placement'.
% open_loop: F=M2=0; no pole design, no speed control, no gamma hold.
e.controller.mode='pole_placement';
% For direct mode set e.controller.K to 2-by-12 gain matrix.
% e.controller.K=zeros(2,12);
% e.controller.K(1,[7 9 1 4]) = [Ktheta Kr Ksigma1 Ksigma_r];
% e.controller.K(2,[8 10 2 5]) = [Kphi Kgamma Ksigma2 Ksigma_g];

% Low-level sweep defaults; the two main scripts explicitly set enabled.
% Each run starts from the same base configuration;
%    the selected value is applied BEFORE generating its controller.
e.sweep.enabled=false;
e.sweep.parameter='parameters.mr';
e.sweep.values=[1.5 2.3 3.0];
e.sweep.index=[]; % []: scalar field; integer: element of a vector/matrix
% Examples:
% parameters.BR       [0 0.02 0.05]    index=[]
% controller.kp_gamma [2 3 4]          index=[]  (pd only)
% settings.theta0     [1 2.8 5]*pi/180 index=[]
% design.lateral      [-1 -2 -3]       index=1   (pole_placement only)
% controller.K        [-700 -800 -900] index=13 (direct; K(1,7), MATLAB linear index)
% Conjugate pairs must remain paired: edit a full vector in the design file.
end
