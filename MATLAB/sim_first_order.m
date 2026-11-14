function y = sim_first_order(p, V, dt)
% first order model  tau*w_dot + w = K V   (discrete, same sample time as the data)
K = p(1); tau = max(p(2), 1e-3);
y = zeros(size(V)); a = exp(-dt/tau);
for i = 2:numel(V)
    y(i) = a*y(i-1) + (1-a)*K*V(i-1);
end
end
