%% =====================================================================
%  MILESTONE 2 - Complete mathematical model of the 2WD mobile robot
%  Electrical model + mechanical model -> one state space for the whole robot
%  Needs: Control System Toolbox (ss, tf, step, pid ...)
%  Run this file section by section (Ctrl+Enter) or all at once (F5)
% ======================================================================
clc; clear; close all;

%% 1) Parameters ---------------------------------------------------------
P = robot_params(true);          % prints the parameter table + weight distribution

%% 2) Complete state-space model ----------------------------------------
S = robot_ss(P);                 % equations are written inside robot_ss.m

sys = ss(S.A, S.B, S.C, S.D, ...
        'StateName', S.states, 'InputName', S.inputs, 'OutputName', S.outputs);

disp('================ STATE SPACE  x_dot = A x + B u ================');
disp('x = [s  psi  wL  wR  iL  iR]''     u = [VL  VR  F_load]''');
A = S.A, B = S.B, C = S.C, D = S.D          %#ok<NOPTS>
disp('Eigenvalues of A (system poles):');
disp(eig(A));
% two poles at 0  -> s and psi are integrals of v and psi_dot (position, heading)
% two slow poles  -> mechanical (straight motion and rotation)
% two fast poles  -> electrical (L/R), very fast

% PBH (Hautus) test - better conditioned than rank(ctrb) because A has very
% fast electrical poles (-2000) next to zero poles
lam = eig(A);  n = size(A,1);  Bv = B(:,1:2);  Cm = C([1 2 5 6],:);
ctrb_ok = all(arrayfun(@(l) rank([l*eye(n)-A, Bv]) == n, lam));
obsv_ok = all(arrayfun(@(l) rank([l*eye(n)-A; Cm]) == n, lam));
fprintf('Controllable from (VL,VR): %d   |   Observable from (s, psi, wL, wR): %d\n\n', ctrb_ok, obsv_ok);

%% 3) Transfer function from the state space (no hand calculation) ---------
% Straight line: VL = VR = V   ->  wheel speed / voltage
G_from_ss = tf( minreal( sys(5,1:2)*[1;1] ) );   % output wL, input VL = VR = V
G_hand    = tf(S.num, S.den);                    % J*theta_dd = T  (one wheel, Jeq = Jw + m r^2/2)
G1        = tf(S.K, [S.tau 1]);                  % 1st order approximation

disp('---------- Wheel speed / Voltage (from the state space) ----------');
G_from_ss
disp('---------- Same transfer function written by hand  J*theta_dd = T ----------');
G_hand
fprintf('DC gain  : %.4f (rad/s)/V   time constant tau = %.3f s\n\n', dcgain(G_hand), S.tau);

plot_poles(pole(G_hand), [], [], 'Open loop poles');       % open loop: -7.1 (mechanical), -1993 (electrical)
fprintf('Open loop poles: %s\n', mat2str(pole(G_hand).', 4));

%% 4) Steady state (no TF needed:  0 = A x + B u  ->  x = -A\B u) -------
% s and psi keep growing, so the steady state is taken on [wL wR iL iR]
A4 = S.A(3:6,3:6);  B4 = S.B(3:6,:);
th_list = [0 P.theta];
fprintf('==================== STEADY STATE  (VL = VR = Vmax = %.1f V) ====================\n', P.Vmax);
for th = th_list
    F  = P.m*P.g*(sind(th) + P.Crr*cosd(th));
    x4 = -A4 \ (B4*[P.Vmax; P.Vmax; F]);
    fprintf('ramp %4.1f deg :  w = %.2f rad/s (%.0f rpm)   v = %.3f m/s   i = %.2f A per motor\n', ...
            th, x4(1), x4(1)*60/2/pi, x4(1)*P.r, x4(3));
end
fprintf('\n');

%% 5) Open loop response of the whole robot -------------------------------
t  = (0:0.001:3)';
u  = [P.Vmax*ones(size(t)), P.Vmax*ones(size(t)), P.m*P.g*P.Crr*ones(size(t))];
u(t<0.2,:) = 0;
y  = lsim(sys, u, t);

figure('Name','Open loop step - whole robot');
subplot(3,1,1); plot(t, y(:,3), 'LineWidth',1.5); grid on; ylabel('v [m/s]');
title(sprintf('Open loop: step of %.1f V on both motors (flat ground)', P.Vmax));
subplot(3,1,2); plot(t, y(:,5), t, y(:,6), '--', 'LineWidth',1.5); grid on; ylabel('\omega [rad/s]'); legend('\omega_L','\omega_R');
subplot(3,1,3); plot(t, y(:,7), 'LineWidth',1.5); grid on; ylabel('i [A]'); xlabel('t [s]');

% turning: VL = 0.3 Vmax, VR = Vmax
u2 = [0.3*P.Vmax*ones(size(t)), P.Vmax*ones(size(t)), P.m*P.g*P.Crr*ones(size(t))];
y2 = lsim(sys, u2, t);
figure('Name','Open loop - turning');
subplot(2,1,1); plot(t, y2(:,3), 'LineWidth',1.5); grid on; ylabel('v [m/s]'); title('V_L = 0.3 V_{max} ,  V_R = V_{max}');
subplot(2,1,2); plot(t, y2(:,4), 'LineWidth',1.5); grid on; ylabel('\psi_{dot} [rad/s]'); xlabel('t [s]');

%% 6) Ramp: can the robot climb? ------------------------------------------
th  = 0:1:35;
F   = P.m*P.g*(sind(th) + P.Crr*cosd(th));          % force needed along the ramp
Tw  = F*P.r/2;                                       % torque needed per wheel
w_ss = (P.Kt*P.Vmax/P.R - Tw) ./ (P.Kt*P.Ke/P.R + P.b);   % speed with full voltage
i_ss = (P.Vmax - P.Ke*w_ss)/P.R;                     % current per motor

T_avail0 = P.Kt*P.Vmax/P.R;                          % torque at zero speed with Vmax
th_torque = asind(min(1, 2*T_avail0/P.r/(P.m*P.g)));
th_slip   = atand(P.mu*P.f_drive);                   % wheels start slipping (approx.)
th_rated  = interp1(Tw, th, 3.9*P.g/100);            % angle where rated torque 3.9 kg.cm is used

fprintf('======================= RAMP =======================\n');
fprintf('Torque per wheel on %g deg: %.3f N.m = %.2f kg.cm  (rated 3.9, stall 12 kg.cm)\n', ...
        P.theta, interp1(th,Tw,P.theta), interp1(th,Tw,P.theta)/P.g*100);
fprintf('Max angle - rated torque     : %.1f deg\n', th_rated);
fprintf('Max angle - motor torque     : %.1f deg\n', th_torque);
fprintf('Max angle - tyre grip (mu=%.1f): %.1f deg  <- the real limit\n\n', P.mu, th_slip);

figure('Name','Ramp analysis');
subplot(2,1,1); plot(th, Tw/P.g*100, 'LineWidth',1.5); hold on;
yline(3.9,'--','rated 3.9 kg.cm'); yline(12,'r--','stall 12 kg.cm'); xline(th_slip,'k:','slip limit');
grid on; ylabel('torque per wheel [kg.cm]'); title('Torque needed vs ramp angle');
subplot(2,1,2); yyaxis left; plot(th, w_ss*P.r, 'LineWidth',1.5); ylabel('v at V_{max} [m/s]');
yyaxis right; plot(th, i_ss, 'LineWidth',1.5); ylabel('current per motor [A]'); yline(P.I_L298,'r--','L298N limit');
grid on; xlabel('ramp angle [deg]');

%% 7) PI speed controller --------------------------------------------------
Cpi = pid(S.Kp, S.Ki);
L_ol = Cpi*G_hand;
T_cl = feedback(L_ol, 1);
fprintf('================ PI CONTROLLER ================\n');
fprintf('Kp = %.3f   Ki = %.3f   (these go to the ESP32 from Simulink)\n', S.Kp, S.Ki);
info = stepinfo(T_cl);
fprintf('Rise time %.3f s | Settling %.3f s | Overshoot %.1f %% | ss error %.4f\n\n', ...
        info.RiseTime, info.SettlingTime, info.Overshoot, 1 - dcgain(T_cl));
try   % compare with MATLAB auto tuning (optional)
    Ct = pidtune(G_hand, 'PI');
    fprintf('pidtune suggestion: Kp = %.3f  Ki = %.3f\n\n', Ct.Kp, Ct.Ki);
catch
end

figure('Name','Closed loop step');
step(G_hand*S.K^-1, T_cl, 1.5); grid on;
legend('open loop (normalised)','closed loop with PI'); title('Wheel speed step response');
plot_poles(pole(G_hand), pole(T_cl), zero(T_cl), 'Closed loop poles');
fprintf('Closed loop poles: %s\n\n', mat2str(pole(T_cl).', 4));

%% 7b) Stability: Bode (gain / phase margin) and root locus ---------------
[Gm, Pm, Wcg, Wcp] = margin(L_ol);
fprintf('Stability margins (PI + motor): gain margin = %.1f dB , phase margin = %.1f deg at %.1f rad/s\n\n', ...
        20*log10(Gm), Pm, Wcp);
figure('Name','Bode - open loop with PI'); margin(L_ol); grid on;
figure('Name','Root locus'); rlocus(tf([1 S.Ki/S.Kp], [1 0]) * G_hand); hold on;
p_sel = pole(T_cl);  plot(real(p_sel), imag(p_sel), 'rs', 'MarkerSize', 10, 'LineWidth', 2);
xlim([-40 5]); ylim([-20 20]); grid on;
title(sprintf('Root locus with the PI zero at -%.2f  (red squares = chosen gain Kp = %.2f)', S.Ki/S.Kp, S.Kp));

%% 8) Closed loop: robot drives and climbs the ramp (full model) ----------
dt = 1e-4;  Tend = 9;  n = round(Tend/dt);
v_ref = 0.30;                                   % [m/s]
x = zeros(6,1);  I = zeros(2,1);
rec = zeros(round(n/10), 9);  k2 = 0;
for k = 1:n
    tk   = k*dt;
    wref = (tk > 0.5) * v_ref/P.r * [1;1];
    e    = wref - x(3:4);
    uPI  = S.Kp*e + S.Ki*I;
    uSat = max(-P.Vmax, min(P.Vmax, uPI));
    I    = I + e*dt .* ((uPI == uSat) | (sign(e) ~= sign(uPI)));   % anti-windup
    on_ramp = x(1) >= P.s_ramp && x(1) <= P.s_ramp + P.L_ramp;
    th_k = P.theta*on_ramp;
    Fl   = P.m*P.g*(sind(th_k) + P.Crr*cosd(th_k)*tanh(P.r*mean(x(3:4))/0.01));
    x    = x + dt*(S.A*x + S.B*[uSat; Fl]);
    if mod(k,10) == 0
        k2 = k2 + 1;
        rec(k2,:) = [tk, x(1), P.r*mean(x(3:4)), v_ref*(tk>0.5), uSat(1), uSat(2), x(5), x(6), th_k];
    end
end
rec = rec(1:k2,:);

figure('Name','Ramp climb - closed loop');
subplot(4,1,1); plot(rec(:,1), rec(:,4), 'k--', rec(:,1), rec(:,3), 'LineWidth',1.5); grid on;
ylabel('v [m/s]'); legend('v_{ref}','v'); title(sprintf('Robot climbing a %g^o ramp with PI speed control', P.theta));
subplot(4,1,2); plot(rec(:,1), rec(:,9), 'LineWidth',1.5); grid on; ylabel('\theta [deg]');
subplot(4,1,3); plot(rec(:,1), rec(:,5), 'LineWidth',1.5); yline(P.Vmax,'r--','V_{max}'); grid on; ylabel('V [V]');
subplot(4,1,4); plot(rec(:,1), rec(:,7), 'LineWidth',1.5); grid on; ylabel('i [A]'); xlabel('t [s]');

%% 9) Save everything for Simulink -------------------------------------------
save('robot_model.mat', 'P', 'S', 'sys', 'G_hand', 'G_from_ss', 'T_cl');
disp('saved robot_model.mat');
