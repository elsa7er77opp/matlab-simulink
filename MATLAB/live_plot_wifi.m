%% LIVE_PLOT_WIFI - backup for the Simulink model (plain MATLAB, no Simulink)
% Sends a step sequence of speed references to the robot over WiFi and
% draws REFERENCE vs REAL vs MODEL live. Use it if Simulink gives problems.
%   laptop WiFi -> "Robot_ESP32", then  >> live_plot_wifi
clc; close all;
P = robot_params(false);  S = robot_ss(P);

% reference: v [m/s] steps (edit as you like)
seq_t = [0   1    5    9    13];          % time the step happens [s]
seq_v = [0   0.20 0.35 0.10 0  ];         % speed after that time  [m/s]
Tend  = 15;  dt = P.Ts_link;

esp32_link('open');
cleanup = onCleanup(@() esp32_link('close'));

fig = figure('Name','LIVE: robot response');
h1 = animatedline('Color','k','LineStyle','--','LineWidth',1.2);
h2 = animatedline('Color',[0 0.45 0.74],'LineWidth',1.5);
h3 = animatedline('Color',[0.85 0.33 0.1],'LineWidth',1.2);
grid on; xlabel('t [s]'); ylabel('wheel speed [rad/s]');
legend('reference','real robot (avg L/R)','model'); xlim([0 Tend]);

% model: first order + PI (same as the ESP32) simulated next to the robot
wm = 0; Im = 0; a = exp(-dt/S.tau);
t0 = tic; k = 0; data = [];
while toc(t0) < Tend && isvalid(fig)
    k = k + 1;  tk = (k-1)*dt;
    v = seq_v(find(tk >= seq_t, 1, 'last'));
    wref = v / P.r;
    y = esp32_link([wref wref 1 0 S.Kp S.Ki]);           % y = [wL wR VL VR]
    % model step
    e = wref - wm;  u = max(-P.Vmax, min(P.Vmax, S.Kp*e + S.Ki*Im));
    Im = Im + e*dt;  wm = a*wm + (1-a)*S.K*u;
    addpoints(h1, tk, wref); addpoints(h2, tk, mean(y(1:2))); addpoints(h3, tk, wm);
    drawnow limitrate
    data(end+1,:) = [tk wref y(:).' wm]; %#ok<SAGROW>
    while toc(t0) < k*dt, end                              % keep real time
end
save('live_test_data.mat', 'data');
disp('saved live_test_data.mat  [t wref wL wR VL VR w_model]');
