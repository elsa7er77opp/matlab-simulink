function sm_prepare_data(scenario)
% SM_PREPARE_DATA  runs our state-space model (simulate_robot) and puts the motion
% of the robot in the base workspace for the Simscape Multibody model.
%   body : axle-centre position X Y Z + rotations (rx, ry, rz)
%   wheels : left / right wheel angle
if nargin < 1, scenario = 'ramp'; end
P = robot_params(false);  S = robot_ss(P);
R = simulate_robot(P, S, scenario);
t = R.t(:);  r = P.r;
if strcmp(scenario, 'ramp')
    Xa = R.X - r*sin(R.pitch);   Za = R.Z + r*cos(R.pitch);   Ya = zeros(size(t));
    ry = -R.pitch;               rz = zeros(size(t));
else
    Xa = R.X;  Ya = R.Y;  Za = r*ones(size(t));
    ry = zeros(size(t));         rz = R.psi;
end
% start at t = 0 with the robot standing still
t = [0; t];  Xa = [Xa(1); Xa]; Ya = [Ya(1); Ya]; Za = [Za(1); Za]; ry = [ry(1); ry]; rz = [rz(1); rz];
thL = [0; R.thL(:)];  thR = [0; R.thR(:)];

assignin('base', 'sm_X',   [t Xa]);
assignin('base', 'sm_Y',   [t Ya]);
assignin('base', 'sm_Z',   [t Za]);
assignin('base', 'sm_ry',  [t ry]);
assignin('base', 'sm_rz',  [t rz]);
assignin('base', 'sm_thL', [t thL]);
assignin('base', 'sm_thR', [t thR]);
assignin('base', 'sm_v',   [R.t(:) R.v(:) R.vref(:)]);
assignin('base', 'sm_Tend', t(end));
fprintf('Simscape data ready (%s, %.1f s)\n', scenario, t(end));
end
