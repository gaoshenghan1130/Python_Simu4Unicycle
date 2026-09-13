function u = unicycle_controller(~, ~, ~)
%UNICYCLE_CONTROLLER Default controller matching Python's empty controller.
%
% u = [T_W; F_L], where T_W is wheel torque and F_L is linear motor force.

T_W = 0.0;
F_L = 0.0;

u = [T_W; F_L];
end
