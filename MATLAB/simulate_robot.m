function R = simulate_robot(P, S, scenario)
% SIMULATE_ROBOT  closed-loop simulation of the whole robot (state space + PI)
%   R = simulate_robot(P, S, 'ramp')    drive 0.3 m/s and climb the ramp
%   R = simulate_robot(P, S, 'square')  drive a 0.6 m square on flat ground
% Returns time histories + the pose of the robot (X, Y, Z, heading, pitch, wheel angles)

if nargin < 3, scenario = 'ramp'; end
dt = 2e-4;                    % integration step (electrical pole -2000 -> needs small dt)
th = P.theta*pi/180;  s1 = P.s_ramp;  Lr = P.L_ramp;

switch scenario
    case 'ramp'
        Tend = 9;
        ref = @(t) [0.30*(t > 0.5), 0];                         % [v_ref, psi_dot_ref]
    case 'square'
        seg_f = 2.4;  seg_t = (pi/2)/1.2;  T1 = seg_f + seg_t;  % forward 0.6 m, turn 90 deg
        Tend = 0.5 + 4*T1 + 0.5;
        ref = @(t) square_ref(t, seg_f, T1);
    otherwise
        error('scenario must be ''ramp'' or ''square''');
end

n = round(Tend/dt);  keep = 50;                 % log every 50 steps (10 ms)
x = zeros(6,1);  I = zeros(2,1);
X = 0; Y = 0; thL = 0; thR = 0;
m = floor(n/keep);
R.t = zeros(m,1); R.vref = R.t; R.v = R.t; R.wref = zeros(m,2); R.w = zeros(m,2);
R.V = zeros(m,2); R.i = zeros(m,2); R.s = R.t; R.psi = R.t; R.theta = R.t;
R.X = R.t; R.Y = R.t; R.thL = R.t; R.thR = R.t;
k2 = 0;
for k = 1:n
    t = k*dt;
    r_ = ref(t);  wref = S.Kinv*r_(:);
    e  = wref - x(3:4);
    uPI = S.Kp*e + S.Ki*I;
    uS  = max(-P.Vmax, min(P.Vmax, uPI));
    I   = I + e*dt .* ((uPI == uS) | (sign(e) ~= sign(uPI)));      % anti-windup
    if strcmp(scenario, 'ramp')
        thk = th*(x(1) >= s1 && x(1) <= s1 + Lr);
    else
        thk = 0;
    end
    v  = P.r*mean(x(3:4));
    Fl = P.m*P.g*(sin(thk) + P.Crr*cos(thk)*tanh(v/0.01));
    x  = x + dt*(S.A*x + S.B*[uS; Fl]);
    % planar pose (flat part of the path)
    X = X + dt*v*cos(x(2));   Y = Y + dt*v*sin(x(2));
    thL = thL + dt*x(3);      thR = thR + dt*x(4);
    if mod(k, keep) == 0
        k2 = k2 + 1;
        R.t(k2) = t;  R.vref(k2) = r_(1);  R.v(k2) = v;
        R.wref(k2,:) = wref.';  R.w(k2,:) = x(3:4).';  R.V(k2,:) = uS.';  R.i(k2,:) = x(5:6).';
        R.s(k2) = x(1);  R.psi(k2) = x(2);  R.theta(k2) = thk*180/pi;
        R.X(k2) = X;  R.Y(k2) = Y;  R.thL(k2) = thL;  R.thR(k2) = thR;
    end
end

% ---- 3D position on the ramp (s is measured along the surface)
if strcmp(scenario, 'ramp')
    [R.X, R.Z] = ramp_xz(R.s, s1, Lr, th);
    R.Y = zeros(size(R.X));
    wb = P.x_axle - P.x_cast;                    % axle -> caster distance
    [Xc, Zc] = ramp_xz(R.s - wb, s1, Lr, th);
    R.pitch = atan2(R.Z - Zc, R.X - Xc);         % body pitch from the 2 contact points
else
    R.Z = zeros(size(R.X));  R.pitch = zeros(size(R.X));
end
R.scenario = scenario;
end

function r = square_ref(t, seg_f, T1)
t = t - 0.5;
if t < 0 || t > 4*T1, r = [0 0]; return; end
tl = mod(t, T1);
if tl < seg_f, r = [0.25 0]; else, r = [0 1.2]; end
end

function [X, Z] = ramp_xz(s, s1, Lr, th)
X = s;  Z = zeros(size(s));
on = s > s1 & s <= s1 + Lr;
X(on) = s1 + (s(on) - s1)*cos(th);   Z(on) = (s(on) - s1)*sin(th);
up = s > s1 + Lr;
X(up) = s1 + Lr*cos(th) + (s(up) - s1 - Lr);   Z(up) = Lr*sin(th);
end
