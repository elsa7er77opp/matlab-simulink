%% RST_DESIGN - digital RST (polynomial pole placement) speed controller
%
%   S(q^-1) u(t) = T(q^-1) y*(t) - R(q^-1) y(t)
%
%   plant (one wheel, sampled with ZOH, Ts = 10 ms):
%       A(q^-1) y(t) = B(q^-1) u(t)      A = 1 + a1 q^-1 ,  B = b1 q^-1
%   integral action  : S = (1 - q^-1) S'      (zero steady-state error, rejects the ramp load)
%   Diophantine eq.  : A (1 - q^-1) S' + B R = P(q^-1)     (P = wanted closed-loop poles)
%   reference        : T = P(1) / B(1)        (unity gain, no extra zero -> no overshoot)
%
% The result is compared with the PI controller on the COMPLETE robot model
% (6-state state space) driving at 0.3 m/s and hitting a 15 deg ramp.
clc; close all;
P = robot_params(false);  S = robot_ss(P);
Ts = P.Ts;                                     % 0.01 s (ESP32 loop)

%% 1) discrete plant ----------------------------------------------------------
% electrical pole (-2000) is 300x faster than the mechanical one -> 1st order model
a1 = -exp(-Ts/S.tau);
b1 = S.K*(1 - exp(-Ts/S.tau));
fprintf('Discrete plant (ZOH, Ts = %g s):  A = 1 %+.5f q^-1 ,  B = %.5f q^-1\n', Ts, a1, b1);
try
    Gz_full = c2d(tf(S.num, S.den), Ts, 'zoh');
    fprintf('(full 2nd order c2d for comparison:)\n'); Gz_full %#ok<NOPTS>
catch
end

%% 2) wanted closed-loop poles -----------------------------------------------
wn = 10;  zeta = 1;                            % same speed as the PI design
p1 = -2*exp(-zeta*wn*Ts)*cos(wn*Ts*sqrt(max(0, 1 - zeta^2)));
p2 = exp(-2*zeta*wn*Ts);
Pz = [1 p1 p2];
fprintf('P(q^-1) = 1 %+.5f q^-1 %+.5f q^-2   (poles z = %s)\n', p1, p2, mat2str(roots(Pz).', 4));

%% 3) solve the Diophantine equation (Sylvester matrix) ------------------------
Ap = conv([1 a1], [1 -1]);                     % A' = A (1 - q^-1) = [1 A'1 A'2]
%       unknowns  x = [s0 ; r0 ; r1]
M = [ 1      0   0  ;                          % q^0  : s0             = 1
      Ap(2)  b1  0  ;                          % q^-1 : s0 A'1 + b1 r0 = p1
      Ap(3)  0   b1 ];                         % q^-2 : s0 A'2 + b1 r1 = p2
sol = M \ Pz(:);
s0 = sol(1);  r0 = sol(2);  r1 = sol(3);
Rq = [r0 r1];  Sq = conv(s0, [1 -1]);  T0 = sum(Pz)/b1;

fprintf('\n================ RST CONTROLLER ================\n');
fprintf('R(q^-1) = %.4f %+.4f q^-1\n', r0, r1);
fprintf('S(q^-1) = 1 - q^-1\n');
fprintf('T       = %.4f\n', T0);
fprintf('control law:  u(t) = u(t-1) + %.4f y*(t) - %.4f y(t) %+.4f y(t-1)\n', T0, r0, -r1);
fprintf('\nfor the ESP32 code:\nconst float RST_R0 = %.5f, RST_R1 = %.5f, RST_T0 = %.5f;\n\n', r0, r1, T0);

% check: closed loop polynomial A S + B R must equal P
chk = conv([1 a1], Sq) + conv([0 b1], Rq);
fprintf('check A*S + B*R = %s   (should be P = %s)\n\n', mat2str(chk, 5), mat2str(Pz, 5));

try   % closed-loop transfer function y/y* = B T / P
    Hcl = tf([0 b1*T0], Pz, Ts, 'Variable', 'z^-1') %#ok<NOPTS>
catch
end

%% 4) PI vs RST on the complete robot model ----------------------------------
dt = 2e-4;  Tend = 6;  n = round(Tend/dt);  every = round(Ts/dt);
wref_f = @(t) (t > 0.5) * 0.30/P.r;
load_f = @(t) (t > 2.5 && t < 4.5) * P.theta*pi/180;    % on the ramp between 2.5 and 4.5 s
names = {'PI', 'RST'};  res = cell(1,2);
for c = 1:2
    x = zeros(6,1);  u = 0;  I = 0;  yprev = 0;  lg = zeros(round(n/every), 5);  j = 0;
    for k = 1:n
        t = k*dt;
        if mod(k-1, every) == 0                      % controller runs every Ts (sampled)
            y = x(3);  ys = wref_f(t);
            if c == 1                                % discrete PI (same Kp Ki as before)
                e = ys - y;  uu = S.Kp*e + S.Ki*(I + e*Ts);
                us = max(-P.Vmax, min(P.Vmax, uu));
                if us == uu, I = I + e*Ts; end
                u = us;
            else                                     % RST, velocity form -> natural anti-windup
                u = max(-P.Vmax, min(P.Vmax, u + T0*ys - r0*y - r1*yprev));
                yprev = y;
            end
            j = j + 1;  lg(j,:) = [t, ys, y, u, load_f(t)*180/pi];
        end
        th = load_f(t);  v = P.r*mean(x(3:4));
        Fl = P.m*P.g*(sin(th) + P.Crr*cos(th)*tanh(v/0.01));
        x = x + dt*(S.A*x + S.B*[u; u; Fl]);
    end
    res{c} = lg(1:j,:);
end

figure('Name','PI vs RST','Color','w');
subplot(3,1,1); plot(res{1}(:,1), res{1}(:,2), 'k--', res{1}(:,1), res{1}(:,3), res{2}(:,1), res{2}(:,3), 'LineWidth', 1.4);
grid on; ylabel('\omega_{wheel} [rad/s]'); legend('reference', 'PI', 'RST', 'Location', 'southeast');
title(sprintf('Speed control on the full robot model - %g^o ramp between 2.5 s and 4.5 s', P.theta));
subplot(3,1,2); plot(res{1}(:,1), res{1}(:,4), res{2}(:,1), res{2}(:,4), 'LineWidth', 1.4); grid on;
ylabel('V [V]'); legend('PI', 'RST');
subplot(3,1,3); plot(res{1}(:,1), res{1}(:,5), 'LineWidth', 1.4); grid on; ylabel('ramp \theta [deg]'); xlabel('t [s]');

fprintf('================ COMPARISON ================\n');
fprintf('            overshoot   rise time   max drop on ramp\n');
for c = 1:2
    L = res{c};  yf = L(end,2);
    st = L(:,1) > 0.5 & L(:,1) < 2.4;
    i1 = find(L(:,3) >= 0.1*yf & L(:,1) > 0.5, 1);  i9 = find(L(:,3) >= 0.9*yf & L(:,1) > 0.5, 1);
    os = max(0, (max(L(st,3)) - yf)/yf*100);
    rp = L(:,1) > 2.5 & L(:,1) < 4.5;
    fprintf('  %-4s      %5.1f %%     %5.3f s     %5.1f %%\n', names{c}, os, L(i9,1) - L(i1,1), (yf - min(L(rp,3)))/yf*100);
end
