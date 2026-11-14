function build_robot_wireless()
% BUILD_ROBOT_WIRELESS  creates Robot_Wireless_RT.slx - drive the REAL robot from Simulink
%
%  Simulink --(WiFi, UDP)--> ESP32 : wheel speed references + mode + PI gains   (50 Hz)
%  ESP32    --(WiFi, UDP)--> Simulink : measured wheel speeds + motor voltages  (50 Hz)
%  The same references also drive our state-space model, so every scope shows
%       REFERENCE  |  REAL ROBOT  |  MODEL        live, in real time.
%
%  Before Run :  1) generate_esp32_code  and upload ESP32_Robot.ino
%                2) connect the laptop WiFi to "Robot_ESP32" (password 12345678)
%  While running: move the sliders (speed / turning) -> the robot moves immediately.
%                 or double click v_ref / psi_dot_ref / mode and type a value.
%
%  mode = 1 : PI on the ESP32 (Kp, Ki below)     mode = 2 : RST on the ESP32
%  mode = 0 : open loop, both motors get V_test volts (for identify_params.m)
%  Safety  : v_ref limited to +/-0.4 m/s, turning to +/-2 rad/s, V_test to 0..9 V

mdl = 'Robot_Wireless_RT';
evalin('base', 'robot_setup');
P  = evalin('base', 'P');
Ts = num2str(P.Ts_link);
fprintf('Link: WiFi - connect the laptop to the network "Robot_ESP32" before Run.\n');

if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist([mdl '.slx'], 'file'), delete([mdl '.slx']); end
new_system(mdl); open_system(mdl);
set_param(mdl, 'InitFcn', 'robot_setup;', 'StartFcn', 'esp32_link(''open'');', ...
          'StopFcn', 'esp32_link(''close'');', 'StopTime', 'inf', ...
          'SolverType', 'Fixed-step', 'Solver', 'ode4', 'FixedStep', '0.001');
try
    set_param(mdl, 'EnablePacing', 'on', 'PacingRate', '1');      % run at real-time speed
catch
end

%% ======================= operator inputs (tunable while running) =======================
cst = {'v_ref',       '0',     [40  40 110  70];      % m/s
       'psi_dot_ref', '0',     [40 100 110 130];      % rad/s
       'mode',        '1',     [40 190 110 220];      % 0 open loop / 1 PI / 2 RST
       'V_test',      '0',     [40 250 110 280];      % V (open loop only)
       'Kp',          'S.Kp',  [40 310 110 340];
       'Ki',          'S.Ki',  [40 370 110 400]};
for k = 1:size(cst,1)
    add_block('simulink/Sources/Constant', [mdl '/' cst{k,1}], 'Value', cst{k,2}, ...
              'Position', cst{k,3}, 'SampleTime', Ts);
end
% safety limits
lim = {'v_ref', 'limit v',  '0.4', '-0.4';  'psi_dot_ref', 'limit turn', '2', '-2';  'V_test', 'limit V', '9', '0'};
for k = 1:size(lim,1)
    p = get_param([mdl '/' lim{k,1}], 'Position');
    add_block('simulink/Discontinuities/Saturation', [mdl '/' lim{k,2}], 'Position', p + [100 0 100 0], ...
              'UpperLimit', lim{k,3}, 'LowerLimit', lim{k,4});
    add_line(mdl, [lim{k,1} '/1'], [lim{k,2} '/1']);
end
add_block('simulink/Signal Routing/Mux', [mdl '/Mux_ref'], 'Position', [260 40 265 130], 'Inputs', '2');
add_block('simulink/Math Operations/Gain', [mdl '/Inverse kinematics'], 'Position', [300 65 380 105], ...
          'Gain', 'S.Kinv', 'Multiplication', 'Matrix(K*u)');               % -> [wL_ref; wR_ref]
add_line(mdl, 'limit v/1', 'Mux_ref/1');  add_line(mdl, 'limit turn/1', 'Mux_ref/2');
add_line(mdl, 'Mux_ref/1', 'Inverse kinematics/1');

% dashboard: sliders to drive the robot live (optional - works without them)
add_dash(mdl, 'Slider', 'SPEED v [m per s]',        'v_ref',       [-0.4 0.1 0.4], [40 450 240 500]);
add_dash(mdl, 'Knob',   'TURN psi_dot [rad per s]', 'psi_dot_ref', [-2 0.5 2],     [260 440 360 520]);
add_dash(mdl, 'Slider', 'V_test [V] (mode 0)',  'V_test',      [0 1 9],        [380 450 580 500]);

%% ======================= ESP32 over WiFi (UDP) =======================
add_block('simulink/Signal Routing/Mux', [mdl '/Mux_cmd'], 'Position', [440 40 445 400], 'Inputs', '5');
add_line(mdl, 'Inverse kinematics/1', 'Mux_cmd/1');
add_line(mdl, 'mode/1', 'Mux_cmd/2');   add_line(mdl, 'limit V/1', 'Mux_cmd/3');
add_line(mdl, 'Kp/1', 'Mux_cmd/4');     add_line(mdl, 'Ki/1', 'Mux_cmd/5');

blk = [mdl '/ESP32 (WiFi)'];
try
    add_block('simulink/User-Defined Functions/Interpreted MATLAB Function', blk, ...
              'Position', [490 195 620 245], 'MATLABFcn', 'esp32_link(u)', 'OutputDimensions', '4');
    try, set_param(blk, 'SampleTime', Ts); catch, end
catch    % releases without the interpreted block -> MATLAB Function block
    add_block('simulink/User-Defined Functions/MATLAB Function', blk, 'Position', [490 195 620 245]);
    code = sprintf(['function y = esp32_io(u)\n' 'coder.extrinsic(''esp32_link'');\n' ...
                    'y = zeros(4,1);\n' 'y = esp32_link(u);\n']);
    rt = sfroot; ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', blk);  ch.Script = code;
end
add_line(mdl, 'Mux_cmd/1', 'ESP32 (WiFi)/1');
add_block('simulink/Signal Routing/Demux', [mdl '/Demux_real'], 'Position', [660 180 665 260], 'Outputs', '4');
add_line(mdl, 'ESP32 (WiFi)/1', 'Demux_real/1');            % 1 wL  2 wR  3 VL  4 VR
add_block('simulink/Signal Routing/Mux', [mdl '/Mux_wreal'], 'Position', [720 180 725 220], 'Inputs', '2');
add_line(mdl, 'Demux_real/1', 'Mux_wreal/1');  add_line(mdl, 'Demux_real/2', 'Mux_wreal/2');
add_block('simulink/Math Operations/Gain', [mdl '/v real (r x mean w)'], 'Position', [760 180 840 220], ...
          'Gain', '[P.r/2 P.r/2]', 'Multiplication', 'Matrix(K*u)');
add_line(mdl, 'Mux_wreal/1', 'v real (r x mean w)/1');

%% ======================= the model next to the robot (same input) =======================
add_block('simulink/Math Operations/Sum', [mdl '/Error'], 'Position', [490 600 510 630], 'Inputs', '+-');
add_block('simulink/Continuous/PID Controller', [mdl '/PI (model)'], 'Position', [540 590 610 640]);
pidp = {'Controller','PI', 'P','S.Kp', 'I','S.Ki', 'LimitOutput','on', ...
        'UpperSaturationLimit','P.Vmax', 'LowerSaturationLimit','-P.Vmax', 'AntiWindupMode','clamping'};
for k = 1:2:numel(pidp)
    try, set_param([mdl '/PI (model)'], pidp{k}, pidp{k+1}); catch, end
end
add_block('simulink/Math Operations/Gain', [mdl '/V_test both'], 'Position', [540 660 590 690], ...
          'Gain', 'ones(2,1)', 'Multiplication', 'Matrix(K*u)');
add_block('simulink/Signal Routing/Switch', [mdl '/open loop?'], 'Position', [640 600 670 680], ...
          'Criteria', 'u2 > Threshold', 'Threshold', '0.5');
add_block('simulink/Signal Routing/Mux', [mdl '/Mux_u'], 'Position', [700 600 705 700], 'Inputs', '2');
add_block('simulink/Continuous/State-Space', [mdl '/Robot model (state space)'], 'Position', [730 625 850 675], ...
          'A', 'S.A', 'B', 'S.B', 'C', 'S.C', 'D', 'S.D');
add_block('simulink/Signal Routing/Demux', [mdl '/Demux_model'], 'Position', [880 580 885 740], 'Outputs', '8');
add_block('simulink/Signal Routing/Mux', [mdl '/Mux_wm'], 'Position', [920 650 925 690], 'Inputs', '2');
add_fcn(mdl, 'F_load (flat)', [920 740 1020 770], sprintf('%.6g*tanh(u/0.01)', P.m*P.g*P.Crr));

add_line(mdl, 'Inverse kinematics/1', 'Error/1');  add_line(mdl, 'Error/1', 'PI (model)/1');
add_line(mdl, 'PI (model)/1', 'open loop?/1');     add_line(mdl, 'mode/1', 'open loop?/2');
add_line(mdl, 'limit V/1', 'V_test both/1');       add_line(mdl, 'V_test both/1', 'open loop?/3');
add_line(mdl, 'open loop?/1', 'Mux_u/1');          add_line(mdl, 'F_load (flat)/1', 'Mux_u/2');
add_line(mdl, 'Mux_u/1', 'Robot model (state space)/1');
add_line(mdl, 'Robot model (state space)/1', 'Demux_model/1');     % 1 s 2 psi 3 v 4 psi_dot 5 wL 6 wR 7 iL 8 iR
add_line(mdl, 'Demux_model/5', 'Mux_wm/1');  add_line(mdl, 'Demux_model/6', 'Mux_wm/2');
add_line(mdl, 'Mux_wm/1', 'Error/2');        add_line(mdl, 'Demux_model/3', 'F_load (flat)/1');

%% ======================= live scopes =======================
add_block('simulink/Signal Routing/Demux', [mdl '/Demux_ref'], 'Position', [420 140 425 180], 'Outputs', '2');
add_line(mdl, 'Inverse kinematics/1', 'Demux_ref/1');
sc = {'ROBOT speed v [m per s] - ref, real, model',   3, [1000  40 1050 100];
      'LEFT wheel [rad per s] - ref, real, model',    3, [1000 140 1050 200];
      'RIGHT wheel [rad per s] - ref, real, model',   3, [1000 240 1050 300];
      'Left motor voltage [V] - real, model',         2, [1000 340 1050 390]};
for k = 1:size(sc,1)
    add_block('simulink/Sinks/Scope', [mdl '/' sc{k,1}], 'Position', sc{k,3}, 'NumInputPorts', num2str(sc{k,2}));
end
add_block('simulink/Signal Routing/Demux', [mdl '/Demux_um'], 'Position', [720 520 725 560], 'Outputs', '2');
add_line(mdl, 'open loop?/1', 'Demux_um/1');
L = @(a, b) add_line(mdl, a, b, 'autorouting', 'on');
L('limit v/1',               [sc{1,1} '/1']);
L('v real (r x mean w)/1',   [sc{1,1} '/2']);
L('Demux_model/3',           [sc{1,1} '/3']);
L('Demux_ref/1', [sc{2,1} '/1']);  L('Demux_real/1', [sc{2,1} '/2']);  L('Demux_model/5', [sc{2,1} '/3']);
L('Demux_ref/2', [sc{3,1} '/1']);  L('Demux_real/2', [sc{3,1} '/2']);  L('Demux_model/6', [sc{3,1} '/3']);
L('Demux_real/3', [sc{4,1} '/1']); L('Demux_um/1',   [sc{4,1} '/2']);

% log for identify_params.m (all at 50 Hz): wL_ref wR_ref wL wR VL VR mode V_test
add_block('simulink/Signal Routing/Mux', [mdl '/Mux_log'], 'Position', [720 40 725 140], 'Inputs', '4');
add_block('simulink/Sinks/To Workspace', [mdl '/log_real'], 'Position', [760 75 830 105], ...
          'VariableName', 'log_real', 'SaveFormat', 'Structure With Time');
L('Inverse kinematics/1', 'Mux_log/1');  L('ESP32 (WiFi)/1', 'Mux_log/2');
L('mode/1', 'Mux_log/3');                L('limit V/1', 'Mux_log/4');
L('Mux_log/1', 'log_real/1');

% unused outputs
term = {'Demux_model/1','Demux_model/2','Demux_model/4','Demux_model/7','Demux_model/8','Demux_real/4','Demux_um/2'};
for k = 1:numel(term)
    add_block('simulink/Sinks/Terminator', sprintf('%s/T%d', mdl, k), 'Position', [960 560+25*k 970 570+25*k]);
    L(term{k}, sprintf('T%d/1', k));
end

annotate(mdl, sprintf(['DRIVE THE ROBOT: Run, then move the SPEED slider / TURN knob.\n' ...
    'mode: 1 = PI   2 = RST   0 = open loop (V_test).   Stop = motors stop.']), [40 540]);
save_system(mdl);
fprintf(['\n%s.slx created.\n  1) laptop WiFi -> "Robot_ESP32" (password 12345678)\n' ...
         '  2) (Windows firewall: allow MATLAB on private + public networks)\n' ...
         '  3) (first time) run test_robot_connection\n' ...
         '  4) Run, then move the SPEED slider\n'], mdl);
end

% ------------------------------------------------------------------------------------
function add_dash(mdl, type, name, src, lims, pos)
% dashboard slider / knob bound to the "Value" of a Constant block
blk = [mdl '/' name];
try
    add_block(['simulink/Dashboard/' type], blk, 'Position', pos);
    b = Simulink.HMI.ParamSourceInfo;
    b.BlockPath = Simulink.BlockPath([mdl '/' src]);
    b.ParamName = 'Value';
    set_param(blk, 'Binding', b);
    try, set_param(blk, 'Limits', lims); catch
        try, set_param(blk, 'Limits', lims([1 3])); catch, end
    end
catch
    fprintf('  (dashboard %s not added - double click "%s" while running instead)\n', name, src);
    try, delete_block(blk); catch, end
end
end

function add_fcn(mdl, name, pos, expr)
blk = [mdl '/' name];
add_block('simulink/User-Defined Functions/Fcn', blk, 'Position', pos);
try, set_param(blk, 'Expr', expr); catch, set_param(blk, 'Expression', expr); end
end

function annotate(mdl, txt, xy)
try
    a = Simulink.Annotation([mdl '/' strtok(txt, newline)]);
    a.Text = txt;  a.Position = [xy xy + [420 40]];  a.FontSize = 12;
catch
end
end
