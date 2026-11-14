%% ROBOT_KINEMATICS - forward and inverse kinematics of the differential-drive robot
%
%  FORWARD kinematics  (wheel speeds  ->  robot motion)
%       v     = r (wR + wL) / 2
%       omega = r (wR - wL) / d
%       x_dot = v cos(psi) ,  y_dot = v sin(psi) ,  psi_dot = omega
%
%       [x_dot; y_dot; psi_dot] = [ r/2 cos(psi)   r/2 cos(psi) ;
%                                   r/2 sin(psi)   r/2 sin(psi) ;
%                                     -r/d            r/d       ] * [wL; wR]
%
%  INVERSE kinematics  (wanted robot motion  ->  wheel speeds)
%       wL = (v - omega d/2) / r
%       wR = (v + omega d/2) / r
%       turning radius  R_icr = v / omega   (ICR = instantaneous centre of rotation)
%
%  (the inverse kinematics is the matrix S.Kinv used in Simulink and on the ESP32)
clc; close all;
P = robot_params(false);  S = robot_ss(P);
r = P.r;  d = P.d;
w_max = P.Kt*P.Vmax/(P.R*P.b + P.Kt*P.Ke);         % max wheel speed (flat, Vmax)

FK = @(wL, wR) deal(r*(wR + wL)/2, r*(wR - wL)/d);   % -> [v, omega]
IK = @(v, om)  deal((v - om*d/2)/r, (v + om*d/2)/r); % -> [wL, wR]

fprintf('r = %.3f m   d = %.3f m   max wheel speed = %.2f rad/s (%.0f rpm)\n\n', r, d, w_max, w_max*30/pi);
fprintf('Forward matrix  [v; omega] = Jf*[wL; wR]\n');  Jf = [r/2 r/2; -r/d r/d]  %#ok<NOPTS>
fprintf('Inverse matrix  [wL; wR] = Ji*[v; omega]   (= S.Kinv)\n'); Ji = inv(Jf) %#ok<NOPTS>
fprintf('check Jf*Ji = I : error = %.2e\n\n', norm(Jf*Ji - eye(2)));

%% 1) Forward kinematics - examples ---------------------------------------
cases = [ 4  4 ;  4 -4 ;  2  4 ;  0  4 ];
fprintf('=========== FORWARD KINEMATICS ===========\n');
fprintf('  wL [rad/s]  wR [rad/s] |  v [m/s]  omega [rad/s]  R_icr [m]   motion\n');
for k = 1:size(cases,1)
    [v, om] = FK(cases(k,1), cases(k,2));
    if abs(om) < 1e-9, Ricr = inf; txt = 'straight';
    elseif abs(v) < 1e-9, Ricr = 0; txt = 'spin in place';
    else, Ricr = v/om; txt = 'arc';
    end
    fprintf('  %8.2f   %8.2f    | %7.3f   %8.3f     %8.3f    %s\n', cases(k,:), v, om, Ricr, txt);
end

%% 2) Inverse kinematics - examples ----------------------------------------
want = [0.30 0 ; 0 1.5 ; 0.20 0.4 ; 0.25 -1.0];
fprintf('\n=========== INVERSE KINEMATICS ===========\n');
fprintf('  v [m/s]  omega [rad/s] |  wL [rad/s]  wR [rad/s]  (rpm L / R)   possible?\n');
for k = 1:size(want,1)
    [wL, wR] = IK(want(k,1), want(k,2));
    ok = max(abs([wL wR])) <= w_max;
    yn = {'NO (too fast)', 'YES'};
    fprintf('  %6.2f   %8.2f      | %9.2f   %9.2f   (%3.0f / %3.0f)     %s\n', ...
            want(k,:), wL, wR, wL*30/pi, wR*30/pi, yn{ok+1});
end

%% 3) Forward kinematics: drive the wheels -> path of the robot -----------
dt = 0.01;  t = (0:dt:12)';
wL = 3*ones(size(t));  wR = 3*ones(size(t));
wL(t > 3 & t <= 6) = 1.5;                       % turn left (left wheel slower)
wR(t > 6 & t <= 8) = -3;  wL(t > 6 & t <= 8) = 3;   % spin to the right
x = zeros(size(t)); y = x; psi = x;
for k = 2:numel(t)
    [v, om] = FK(wL(k-1), wR(k-1));
    x(k)   = x(k-1) + v*cos(psi(k-1))*dt;
    y(k)   = y(k-1) + v*sin(psi(k-1))*dt;
    psi(k) = psi(k-1) + om*dt;
end
figure('Name','Forward kinematics','Color','w');
subplot(1,2,1); plot(t, wL, t, wR, '--', 'LineWidth', 1.5); grid on;
xlabel('t [s]'); ylabel('\omega [rad/s]'); legend('\omega_L','\omega_R'); title('Input: wheel speeds');
subplot(1,2,2); plot(x, y, 'LineWidth', 1.8); hold on; plot(x(1), y(1), 'go', x(end), y(end), 'rs', 'MarkerSize', 9, 'LineWidth', 2);
quiver(x(1:150:end), y(1:150:end), 0.08*cos(psi(1:150:end)), 0.08*sin(psi(1:150:end)), 0, 'k');
axis equal; grid on; xlabel('x [m]'); ylabel('y [m]'); title('Output: robot path (forward kinematics)');
legend('path','start','end','heading','Location','best');

%% 4) Inverse kinematics: wanted path -> wheel speeds -> check with FK ----
% wanted path: figure-8 (two circles of radius 0.4 m) at 0.2 m/s
Rc = 0.4;  vd = 0.2;  T1 = 2*pi*Rc/vd;
t2 = (0:dt:2*T1)';
v_d  = vd*ones(size(t2));
om_d = (vd/Rc) * ones(size(t2));  om_d(t2 > T1) = -vd/Rc;        % left circle then right circle
[wL2, wR2] = IK(v_d, om_d);                                       % INVERSE kinematics
% desired path (exact geometry)
xd = zeros(size(t2)); yd = xd; pd = xd;
x2 = xd; y2 = xd; p2 = xd;
for k = 2:numel(t2)
    pd(k) = pd(k-1) + om_d(k-1)*dt;
    xd(k) = xd(k-1) + v_d(k-1)*cos(pd(k-1))*dt;   yd(k) = yd(k-1) + v_d(k-1)*sin(pd(k-1))*dt;
    [v, om] = FK(wL2(k-1), wR2(k-1));                              % FORWARD kinematics back
    p2(k) = p2(k-1) + om*dt;
    x2(k) = x2(k-1) + v*cos(p2(k-1))*dt;          y2(k) = y2(k-1) + v*sin(p2(k-1))*dt;
end
fprintf('\nFigure-8 check: max distance between wanted path and FK(IK(path)) = %.2e m\n', ...
        max(hypot(xd - x2, yd - y2)));
fprintf('Wheel speeds needed: wL = %.2f / %.2f rad/s , wR = %.2f / %.2f rad/s (max allowed %.2f)\n', ...
        max(wL2), min(wL2), max(wR2), min(wR2), w_max);

figure('Name','Inverse kinematics','Color','w');
subplot(1,2,1); plot(t2, wL2, t2, wR2, '--', 'LineWidth', 1.5); grid on; yline(w_max, 'r:', 'max');
xlabel('t [s]'); ylabel('\omega [rad/s]'); legend('\omega_L','\omega_R'); title('IK result: wheel speeds needed');
subplot(1,2,2); plot(xd, yd, 'k--', 'LineWidth', 2.5); hold on; plot(x2, y2, 'LineWidth', 1.5);
axis equal; grid on; xlabel('x [m]'); ylabel('y [m]'); title('Wanted path vs FK of the IK wheel speeds');
legend('wanted figure-8', 'robot (FK check)', 'Location', 'best');
