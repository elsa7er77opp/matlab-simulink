function [r0, r1, T0] = rst_coeffs(P, S)
% RST_COEFFS  RST speed controller (same design as rst_design.m)
%   plant  A = 1 + a1 q^-1 , B = b1 q^-1   (1st order, ZOH, Ts = P.Ts)
%   S = 1 - q^-1 (integrator),  A*S + B*R = P(q^-1),  T = P(1)/B(1)
Ts = P.Ts;
a1 = -exp(-Ts/S.tau);   b1 = S.K*(1 - exp(-Ts/S.tau));
wn = S.wn;  z = S.zeta;
p1 = -2*exp(-z*wn*Ts)*cos(wn*Ts*sqrt(max(0, 1 - z^2)));
p2 = exp(-2*z*wn*Ts);
Ap = conv([1 a1], [1 -1]);
r0 = (p1 - Ap(2))/b1;
r1 = (p2 - Ap(3))/b1;
T0 = (1 + p1 + p2)/b1;
end
