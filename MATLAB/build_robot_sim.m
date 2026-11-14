function build_robot_sim()
% BUILD_ROBOT_SIM  creates Robot_Simulation.slx  (full model + PI + ramp)
%
%   v_ref --> [Kinv] --> (+)-> [PI, |V|<=Vmax] --> [ State Space: whole robot ] --> outputs
%                         -^                                ^ F_load (ramp)
%                         |__________ wL , wR ______________|
%
% Run once:  >> build_robot_sim      then press Run in Simulink.
% If you change theta / ramp position in robot_params.m, run this file again.

mdl = 'Robot_Simulation';
evalin('base', 'robot_setup');                 % P and S in the base workspace
P = evalin('base', 'P');

if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist([mdl '.slx'], 'file'), delete([mdl '.slx']); end
new_system(mdl); open_system(mdl);
set_param(mdl, 'InitFcn', 'robot_setup;', 'StopTime', '9', ...
          'Solver', 'ode23t', 'MaxStep', '1e-3');

%% ---------------- references
add_block('simulink/Sources/Step', [mdl '/v_ref'], 'Position', [30 40 70 80], ...
          'Time', '0.5', 'Before', '0', 'After', '0.3');                  % 0.3 m/s
add_block('simulink/Sources/Constant', [mdl '/psi_dot_ref'], 'Position', [30 110 70 140], 'Value', '0');
add_block('simulink/Signal Routing/Mux', [mdl '/Mux_ref'], 'Position', [110 40 115 140], 'Inputs', '2');
add_block('simulink/Math Operations/Gain', [mdl '/Kinv (v,psi_dot -> wL,wR)'], 'Position', [150 70 230 110], ...
          'Gain', 'S.Kinv', 'Multiplication', 'Matrix(K*u)');
add_block('simulink/Math Operations/Sum', [mdl '/Error'], 'Position', [270 75 290 105], 'Inputs', '+-');

%% ---------------- PI controller (same gains as the ESP32)
add_block('simulink/Continuous/PID Controller', [mdl '/PI speed controller'], 'Position', [320 65 400 115]);
pidp = {'Controller','PI', 'P','S.Kp', 'I','S.Ki', 'LimitOutput','on', ...
        'UpperSaturationLimit','P.Vmax', 'LowerSaturationLimit','-P.Vmax', 'AntiWindupMode','clamping'};
for k = 1:2:numel(pidp)
    try, set_param([mdl '/PI speed controller'], pidp{k}, pidp{k+1});
    catch, warning('Set "%s" = %s by hand in the PI block', pidp{k}, pidp{k+1}); end
end

%% ---------------- plant: complete state space of the robot
add_block('simulink/Signal Routing/Mux', [mdl '/Mux_u'], 'Position', [440 60 445 160], 'Inputs', '2');
add_block('simulink/Continuous/State-Space', [mdl '/Robot (state space)'], 'Position', [480 85 600 135], ...
          'A', 'S.A', 'B', 'S.B', 'C', 'S.C', 'D', 'S.D');
add_block('simulink/Signal Routing/Demux', [mdl '/Demux_y'], 'Position', [640 40 645 260], 'Outputs', '8');
% outputs: 1 s, 2 psi, 3 v, 4 psi_dot, 5 wL, 6 wR, 7 iL, 8 iR
add_block('simulink/Signal Routing/Mux', [mdl '/Mux_w'], 'Position', [700 160 705 210], 'Inputs', '2');

%% ---------------- ramp: angle from position, load force from angle
th = P.theta*pi/180;  s1 = P.s_ramp;  s2 = P.s_ramp + P.L_ramp;
add_fcn([mdl '/Ramp angle theta(s)'], [700 280 820 310], sprintf('%.6g*(u>=%.6g)*(u<=%.6g)', th, s1, s2));
add_block('simulink/Signal Routing/Mux', [mdl '/Mux_th_v'], 'Position', [850 280 855 340], 'Inputs', '2');
add_fcn([mdl '/F_load = m g (sin + Crr cos)'], [880 295 1040 325], sprintf('%.6g*(sin(u(1)) + %.6g*cos(u(1))*tanh(u(2)/0.01))', P.m*P.g, P.Crr));
add_block('simulink/Math Operations/Gain', [mdl '/rad2deg'], 'Position', [850 370 890 400], 'Gain', '180/pi');

%% ---------------- scopes + logging
add_block('simulink/Signal Routing/Mux', [mdl '/Mux_v'], 'Position', [760 20 765 70], 'Inputs', '2');
add_block('simulink/Sinks/Scope', [mdl '/Scope v_ref vs v'], 'Position', [800 25 840 65]);
add_block('simulink/Sinks/Scope', [mdl '/Scope wheel speeds'], 'Position', [800 160 840 200]);
add_block('simulink/Signal Routing/Mux', [mdl '/Mux_i'], 'Position', [700 225 705 265], 'Inputs', '2');
add_block('simulink/Sinks/Scope', [mdl '/Scope currents'], 'Position', [760 225 800 265]);
add_block('simulink/Sinks/Scope', [mdl '/Scope motor voltage'], 'Position', [480 190 520 230]);
add_block('simulink/Sinks/Scope', [mdl '/Scope ramp angle'], 'Position', [920 370 960 410]);
add_block('simulink/Sinks/Scope', [mdl '/Scope position s'], 'Position', [700 350 740 390]);
add_block('simulink/Signal Routing/Mux', [mdl '/Mux_log'], 'Position', [1000 20 1005 140], 'Inputs', '5');
add_block('simulink/Sinks/To Workspace', [mdl '/sim_log'], 'Position', [1040 65 1110 95], ...
          'VariableName', 'sim_log', 'SaveFormat', 'Structure With Time');

%% ---------------- wiring
L = @(a,b) add_line(mdl, a, b, 'autorouting', 'on');
L('v_ref/1', 'Mux_ref/1');               L('psi_dot_ref/1', 'Mux_ref/2');
L('Mux_ref/1', 'Kinv (v,psi_dot -> wL,wR)/1');
L('Kinv (v,psi_dot -> wL,wR)/1', 'Error/1');
L('Error/1', 'PI speed controller/1');
L('PI speed controller/1', 'Mux_u/1');   L('F_load = m g (sin + Crr cos)/1', 'Mux_u/2');
L('PI speed controller/1', 'Scope motor voltage/1');
L('Mux_u/1', 'Robot (state space)/1');
L('Robot (state space)/1', 'Demux_y/1');
L('Demux_y/5', 'Mux_w/1');               L('Demux_y/6', 'Mux_w/2');
L('Mux_w/1', 'Error/2');                 L('Mux_w/1', 'Scope wheel speeds/1');
L('Demux_y/7', 'Mux_i/1');               L('Demux_y/8', 'Mux_i/2');
L('Mux_i/1', 'Scope currents/1');
L('Demux_y/1', 'Ramp angle theta(s)/1'); L('Demux_y/1', 'Scope position s/1');
L('Ramp angle theta(s)/1', 'Mux_th_v/1'); L('Demux_y/3', 'Mux_th_v/2');
L('Mux_th_v/1', 'F_load = m g (sin + Crr cos)/1');
L('Ramp angle theta(s)/1', 'rad2deg/1'); L('rad2deg/1', 'Scope ramp angle/1');
L('v_ref/1', 'Mux_v/1');                 L('Demux_y/3', 'Mux_v/2');
L('Mux_v/1', 'Scope v_ref vs v/1');
L('v_ref/1', 'Mux_log/1');  L('Demux_y/3', 'Mux_log/2');  L('Demux_y/1', 'Mux_log/3');
L('rad2deg/1', 'Mux_log/4'); L('Demux_y/7', 'Mux_log/5');
L('Mux_log/1', 'sim_log/1');

% unused outputs -> terminators (no warnings)
for k = [2 4]
    add_block('simulink/Sinks/Terminator', sprintf('%s/T%d', mdl, k), 'Position', [680 20+25*k 690 30+25*k]);
    L(sprintf('Demux_y/%d', k), sprintf('T%d/1', k));
end

save_system(mdl);
fprintf('%s.slx created. Press Run (stop time 9 s), then open the scopes.\n', mdl);
end

function add_fcn(blk, pos, expr)
% Fcn block (parameter is 'Expr' in new releases, 'Expression' in old ones)
add_block('simulink/User-Defined Functions/Fcn', blk, 'Position', pos);
try
    set_param(blk, 'Expr', expr);
catch
    set_param(blk, 'Expression', expr);
end
end
