%% TEST_ROBOT_CONNECTION - run this ONCE before the Simulink wireless model
% Checks: WiFi link, data rate, wheel direction, encoder sign, encoder counts
% per turn (gear ratio) and the PI speed loop.
% >>> LIFT THE ROBOT (wheels in the air) before running. <<<
%   laptop WiFi -> "Robot_ESP32" (use your device-specific passphrase), then  >> test_robot_connection
clc; close all;
P = robot_params(false);  S = robot_ss(P);
dt = P.Ts_link;

fprintf('=== 1) WiFi link to the robot ===\n');
esp32_link('open');
c = onCleanup(@() esp32_link('close'));
n0 = esp32_link('count');
run_cmd([0 0 1 0 S.Kp S.Ki], 1.0, dt);                        % 1 s of "stop"
nGood = esp32_link('count') - n0;
if nGood < 20
    error(['Only %d packets in 1 s from the ESP32. Check: ESP32 powered, ESP32_Robot.ino uploaded, ' ...
           'laptop connected to WiFi "Robot_ESP32", Windows firewall allows MATLAB (UDP).'], nGood);
end
fprintf('   received %d packets in 1 s (expected ~50)  -> link OK\n\n', nGood);

fprintf('=== 2) Open loop 3 V on both motors (1.5 s) ===\n');
y = run_cmd([0 0 0 3 S.Kp S.Ki], 1.5, dt);
w = mean(y(round(end/2):end, 2:3));                            % steady part
w_model = S.K*3;
fprintf('   measured  wL = %+.2f  wR = %+.2f rad/s   (model: %.2f rad/s)\n', w(1), w(2), w_model);
side = {'LEFT', 'RIGHT'};  flagD = {'DIR_L', 'DIR_R'};  flagE = {'ENC_SIGN_L', 'ENC_SIGN_R'};
ok = true;
for k = 1:2
    if abs(w(k)) < 0.3
        fprintf('   !! %s wheel: no speed measured -> check the motor wires / encoder C1 C2 / 3.3V\n', side{k}); ok = false;
    elseif w(k) < 0
        fprintf(['   !! %s wheel speed is NEGATIVE:\n' ...
                 '      - if the wheel turned BACKWARD -> set %s = -1 in the ESP32 code\n' ...
                 '      - if it turned FORWARD          -> set %s = -1 in the ESP32 code\n'], side{k}, flagD{k}, flagE{k});
        ok = false;
    end
end
ratio = mean(abs(w)) / w_model;
if ok
    fprintf('   directions OK\n');
    if ratio > 1.35 || ratio < 0.7
        fprintf(['   !! speed is %.2f x the model -> encoder counts per turn may be wrong.\n' ...
                 '      Turn a wheel ONE turn by hand, send "C" from the phone / Serial Monitor,\n' ...
                 '      and put that number in CPR (ESP32 code) and P.N = CPR/44 (robot_params.m)\n'], ratio);
    else
        fprintf('   speed matches the model (%.2f x) -> encoder settings OK\n', ratio);
    end
end
run_cmd([0 0 1 0 S.Kp S.Ki], 0.8, dt);                         % stop

fprintf('\n=== 3) PI speed control: 4 rad/s on both wheels (2 s) ===\n');
y = run_cmd([4 4 1 0 S.Kp S.Ki], 2.0, dt);
e = 4 - mean(y(round(end/2):end, 2:3));
fprintf('   steady-state error  L = %.2f  R = %.2f rad/s\n', e(1), e(2));
if all(abs(e) < 0.4), fprintf('   PI loop OK\n'); else, fprintf('   !! large error - fix step 2 first\n'); end
run_cmd([0 0 1 0 S.Kp S.Ki], 0.8, dt);

figure('Name', 'Connection test', 'Color', 'w');
plot(y(:,1), y(:,2), y(:,1), y(:,3), '--', 'LineWidth', 1.5); hold on; yline(4, 'k:');
grid on; xlabel('t [s]'); ylabel('\omega [rad/s]'); legend('left', 'right', 'reference');
title('PI test on the real robot (wheels lifted)');
esp32_link('close');
fprintf('\nDone. If everything is OK -> build_robot_wireless and Run.\n');

function y = run_cmd(u, T, dt)
% sends the same command for T seconds at 50 Hz, returns [t wL wR VL VR]
n = round(T/dt);  y = zeros(n, 5);  t0 = tic;
for k = 1:n
    r = esp32_link(u);
    y(k,:) = [toc(t0), r(:).'];
    while toc(t0) < k*dt, end
end
end
