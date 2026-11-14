%% IDENTIFY_PARAMS - get the real J and b of the robot from a test on the real robot
% 1) Robot ON THE GROUND (so the mass of the robot is included in J).
% 2) Run Robot_Wireless_RT, set mode = 0, then change V_test from 0 to 5 V,
%    wait ~3 s, set V_test back to 0, stop the simulation.
% 3) Run this script. It fits  w(s)/V(s) = K/(tau s + 1)  to the real data
%    and calculates b, Jeq and the rotor inertia -> update robot_params.m
clc; close all;
P = robot_params(false);

% ---- get the logged data (works with or without "single simulation output")
lg = [];
if exist('out', 'var'), try, lg = out.log_real; catch, end, end
if isempty(lg) && exist('log_real', 'var'), lg = log_real; end
if isempty(lg)
    error('Run Robot_Wireless_RT first (mode 0, V_test step) so log_real exists.');
end
t  = lg.time(:);
Y  = squeeze(lg.signals.values);  if size(Y,1) ~= numel(t), Y = Y.'; end
% columns: 1 wL_ref 2 wR_ref 3 wL 4 wR 5 VL 6 VR 7 mode 8 V_test
use = Y(:,7) < 0.5;                       % open loop part only
t = t(use); w = Y(use,3:4); V = Y(use,5:6);
if numel(t) < 20, error('Not enough open-loop data (mode = 0).'); end
dt = median(diff(t));

res = zeros(2,2);
figure('Name','Parameter identification');
for k = 1:2
    cost = @(p) sum((w(:,k) - sim_first_order(p, V(:,k), dt)).^2);
    p0 = [1, 0.15];
    p  = fminsearch(cost, p0, optimset('Display','off'));
    res(k,:) = p;
    subplot(2,1,k); plot(t, w(:,k), '.', t, sim_first_order(p, V(:,k), dt), 'LineWidth', 1.5); grid on;
    legend('real robot', sprintf('fit K=%.3f  tau=%.3f s', p(1), p(2)));
    ylabel('\omega [rad/s]'); names = {'LEFT','RIGHT'}; title([names{k} ' wheel']);
end
xlabel('t [s]');

K = mean(res(:,1));  tau = mean(res(:,2));
b_new    = (P.Kt/K - P.Kt*P.Ke) / P.R;
Jeq_new  = tau * (P.R*b_new + P.Kt*P.Ke) / P.R;
Jrot_new = (Jeq_new - P.Jwheel - P.m*P.r^2/2) / P.N^2;

fprintf('================ IDENTIFIED (real robot) ================\n');
fprintf('K   = %.4f  (model %.4f)  rad/s per V\n', K, P.Kt/(P.R*P.b + P.Kt*P.Ke));
fprintf('tau = %.4f  (model %.4f)  s\n', tau, P.R*P.Jeq/(P.R*P.b + P.Kt*P.Ke));
fprintf('b      = %.5f  N m s/rad   (was %.5f)\n', b_new, P.b);
fprintf('Jeq    = %.5f  kg m^2      (was %.5f)\n', Jeq_new, P.Jeq);
fprintf('Jrotor = %.3g  kg m^2      (was %.3g)\n', Jrot_new, P.Jrotor);
disp('-> put b and Jrotor in robot_params.m, run robot_model.m again (new Kp, Ki).');
