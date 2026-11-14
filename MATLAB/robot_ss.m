function S = robot_ss(P)
% ROBOT_SS  Builds the complete state-space model of the robot (no toolbox needed)
%
% ---------------- ELECTRICAL MODEL (each motor, armature circuit) ----------------
%       L di/dt = V - R i - Ke w                                   (1)
%
% ---------------- MECHANICAL MODEL (each wheel)  J*theta_dd = T ----------------
%       Jw * wdot = Kt i - b w - r*F                               (2)
%       F = traction force between the wheel and the ground
%
% ---------------- ROBOT BODY (Newton for the chassis) ----------------------------
%       m  * vdot   = F_L + F_R - m g (sin(th) + Crr cos(th))     (3)
%       Iz * psi_dd = (F_R - F_L) d/2                              (4)
%
% ---------------- KINEMATICS (no slip) -------------------------------------------
%       v      = r (wL + wR)/2           psi_dot = r (wR - wL)/d   (5)
%
% Eliminating F_L , F_R from (2)-(5) gives J*theta_dd = T in matrix form:
%
%       M * [wL_dot; wR_dot] = Kt*[iL; iR] - b*[wL; wR] - (r/2)*F_load*[1; 1]
%
%       M = [ Jw + m r^2/4 + Iz r^2/d^2      m r^2/4 - Iz r^2/d^2 ;
%             m r^2/4 - Iz r^2/d^2           Jw + m r^2/4 + Iz r^2/d^2 ]
%
%       F_load = m g (sin(th) + Crr cos(th))      (ramp + rolling resistance)
%
% ---------------- ONE STATE SPACE FOR THE WHOLE SYSTEM ---------------------------
%   states  x = [ s ; psi ; wL ; wR ; iL ; iR ]
%             s   distance travelled [m]     psi  heading angle [rad]
%             wL,wR wheel speeds [rad/s]     iL,iR motor currents [A]
%   inputs  u = [ VL ; VR ; F_load ]
%   outputs y = [ s ; psi ; v ; psi_dot ; wL ; wR ; iL ; iR ]
%
%       x_dot = A x + B u ,   y = C x + D u

r = P.r; d = P.d; m = P.m; Iz = P.Iz;

M11 = P.Jw + m*r^2/4 + Iz*r^2/d^2;
M12 = m*r^2/4 - Iz*r^2/d^2;
M   = [M11 M12; M12 M11];
Mi  = inv(M);

I2 = eye(2); Z2 = zeros(2);

A = [ 0 0   r/2   r/2      0 0 ;
      0 0  -r/d   r/d      0 0 ;
      zeros(2,2)  -Mi*P.b   Mi*P.Kt ;
      zeros(2,2)  -P.Ke/P.L*I2   -P.R/P.L*I2 ];

B = [ zeros(2,3) ;
      Z2   -Mi*[r/2; r/2] ;
      I2/P.L   zeros(2,1) ];

C = [ 1 0 0     0     0 0 ;      % s
      0 1 0     0     0 0 ;      % psi
      0 0 r/2   r/2   0 0 ;      % v
      0 0 -r/d  r/d   0 0 ;      % psi_dot
      0 0 1     0     0 0 ;      % wL
      0 0 0     1     0 0 ;      % wR
      0 0 0     0     1 0 ;      % iL
      0 0 0     0     0 1 ];     % iR

D = zeros(8,3);

S.A = A; S.B = B; S.C = C; S.D = D; S.M = M;
S.states  = {'s','psi','wL','wR','iL','iR'};
S.inputs  = {'VL','VR','F_load'};
S.outputs = {'s','psi','v','psi_dot','wL','wR','iL','iR'};

% ---------------- single wheel transfer function (straight line, VL = VR = V) -----
%   Jeq wdot = Kt i - b w ,  L di/dt = V - R i - Ke w
%   w(s)/V(s) = Kt / ( (L s + R)(Jeq s + b) + Kt Ke )
S.num = P.Kt;
S.den = conv([P.L P.R], [P.Jeq P.b]) + [0 0 P.Kt*P.Ke];

% first order approximation (L is very small):  K/(tau s + 1)
S.K   = P.Kt / (P.R*P.b + P.Kt*P.Ke);              % steady-state gain [rad/s per V]
S.tau = P.R*P.Jeq / (P.R*P.b + P.Kt*P.Ke);         % mechanical time constant [s]

% ---------------- PI speed controller (pole placement on K/(tau s+1)) -----------
%   closed loop: s^2 + (1 + K Kp)/tau s + K Ki/tau = s^2 + 2 z wn s + wn^2
S.wn = 10;  S.zeta = 1.0;                          % design: critically damped, ts ~ 0.6 s
S.Kp = (2*S.zeta*S.wn*S.tau - 1) / S.K;
S.Ki = S.wn^2 * S.tau / S.K;

% wheel speed references from (v, psi_dot):  [wL; wR] = Kinv * [v; psi_dot]
S.Kinv = (1/r) * [1 -d/2 ; 1 d/2];
end
