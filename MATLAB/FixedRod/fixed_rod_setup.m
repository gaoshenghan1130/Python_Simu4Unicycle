function p = fixed_rod_setup()
% Reuse the full model's parameters and nonlinear expressions.
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'Full','model'),fullfile(root,'Full','config'));
p = model_parameters();
end
