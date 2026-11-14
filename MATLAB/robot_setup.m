% ROBOT_SETUP  loads the parameters + state space into the base workspace.
% Called automatically by the Simulink models (InitFcn) - you can also run it by hand.
P = robot_params(false);
S = robot_ss(P);
