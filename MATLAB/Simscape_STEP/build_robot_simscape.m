function build_robot_simscape(scenario)
% BUILD_ROBOT_SIMSCAPE  Simscape Multibody model of the robot, imported from STEP files.
%
%   >> build_robot_simscape            % robot climbing the ramp
%   >> build_robot_simscape('square')  % robot driving a square (turning)
%
% The CAD (STEP) parts are loaded with File Solid blocks. The motion comes from OUR
% state-space model + PI (simulate_robot.m), so what you see in Mechanics Explorer is
% the mathematical model moving the real CAD of the robot.
%
%   World --- Cartesian joint (X,Y,Z) --- Gimbal joint (rx,ry,rz) --- robot body (8 STEP solids)
%                                                         |--- Gimbal (wheel angle) --- left wheel STEP
%                                                         |--- Gimbal (wheel angle) --- right wheel STEP
%   World --- track STEP (floor + ramp)
%
% Needs: Simscape + Simscape Multibody. Put this folder INSIDE the MATLAB folder
% of the project (it uses robot_params.m, robot_ss.m, simulate_robot.m from there).

if nargin < 1, scenario = 'ramp'; end
here = fileparts(mfilename('fullpath'));
addpath(here);  addpath(fileparts(here));         % project MATLAB folder
mdl = ['Robot_Simscape_' scenario];

sm_prepare_data(scenario);
Tend = evalin('base', 'sm_Tend');

if bdIsLoaded(mdl), close_system(mdl, 0); end
f = fullfile(here, [mdl '.slx']);  if exist(f, 'file'), delete(f); end
new_system(mdl);  open_system(mdl);
set_param(mdl, 'Solver', 'ode23t', 'MaxStep', '0.01', 'StopTime', num2str(Tend), ...
          'InitFcn', sprintf('addpath(''%s''); addpath(''%s''); sm_prepare_data(''%s'');', ...
                             here, fileparts(here), scenario));

%% ---------------- environment
add_block('sm_lib/Frames and Transforms/World Frame', [mdl '/World'], 'Position', [40 300 80 340]);
add_block('sm_lib/Utilities/Mechanism Configuration', [mdl '/Mechanism Configuration'], 'Position', [40 200 80 240]);
add_block('nesl_utility/Solver Configuration', [mdl '/Solver Configuration'], 'Position', [40 400 80 440]);
pc(mdl, 'Mechanism Configuration', 'any', 'World', 'any');
pc(mdl, 'Solver Configuration', 'any', 'World', 'any');

trk = 'track_ramp.step';  if ~strcmp(scenario, 'ramp'), trk = 'track_flat.step'; end
add_solid(mdl, 'Track (STEP)', fullfile(here, trk), [0.80 0.66 0.45], [140 420 200 470]);
pc(mdl, 'Track (STEP)', 'any', 'World', 'any');

%% ---------------- body pose joints
add_block('sm_lib/Joints/Cartesian Joint', [mdl '/Body position XYZ'], 'Position', [260 290 330 350]);
set_actuation([mdl '/Body position XYZ']);
add_block('sm_lib/Joints/Gimbal Joint', [mdl '/Body rotation'], 'Position', [420 290 490 350]);
set_actuation([mdl '/Body rotation']);
pc(mdl, 'World', 'any', 'Body position XYZ', 'B');
pc(mdl, 'Body position XYZ', 'F', 'Body rotation', 'B');

%% ---------------- robot body: 8 STEP solids on the same frame
parts = {'acrylic',     [0.75 0.88 0.95];
         'alu',         [0.78 0.79 0.81];
         'steel',       [0.60 0.60 0.62];
         'motor',       [0.72 0.72 0.70];
         'caster',      [0.55 0.55 0.55];
         'electronics', [0.10 0.35 0.60];
         'battery',     [0.15 0.50 0.25];
         'printed',     [0.95 0.60 0.15]};
for k = 1:size(parts, 1)
    nm = ['Body ' parts{k,1} ' (STEP)'];
    add_solid(mdl, nm, fullfile(here, ['robot_' parts{k,1} '.step']), parts{k,2}, [640 40+60*k 700 80+60*k]);
    pc(mdl, 'Body rotation', 'F', nm, 'any');
end

%% ---------------- wheels: rotation about the axle (body Y axis through the origin)
side = {'L', 'Left'; 'R', 'Right'};
for k = 1:2
    jn = [side{k,2} ' wheel joint'];  sn = [side{k,2} ' wheel (STEP)'];
    add_block('sm_lib/Joints/Gimbal Joint', [mdl '/' jn], 'Position', [640 560+120*(k-1) 710 620+120*(k-1)]);
    set_actuation([mdl '/' jn]);
    pc(mdl, 'Body rotation', 'F', jn, 'B');
    add_solid(mdl, sn, fullfile(here, ['robot_wheel_' side{k,1} '.step']), [0.12 0.12 0.12], ...
              [800 560+120*(k-1) 860 600+120*(k-1)]);
    pc(mdl, jn, 'F', sn, 'any');
end

%% ---------------- motion inputs from our model (Simulink -> physical signals)
% joint input ports: 2 = x / rx , 3 = y / ry , 4 = z / rz
ins = { 'sm_X',   'Body position XYZ', 2, 'm';
        'sm_Y',   'Body position XYZ', 3, 'm';
        'sm_Z',   'Body position XYZ', 4, 'm';
        '0',      'Body rotation',     2, 'rad';
        'sm_ry',  'Body rotation',     3, 'rad';
        'sm_rz',  'Body rotation',     4, 'rad';
        '0',      'Left wheel joint',  2, 'rad';
        'sm_thL', 'Left wheel joint',  3, 'rad';
        '0',      'Left wheel joint',  4, 'rad';
        '0',      'Right wheel joint', 2, 'rad';
        'sm_thR', 'Right wheel joint', 3, 'rad';
        '0',      'Right wheel joint', 4, 'rad'};
for k = 1:size(ins, 1)
    y0 = 40 + 45*(k-1);
    src = sprintf('in%02d %s', k, ins{k,1});  cv = sprintf('PS%02d', k);
    if strcmp(ins{k,1}, '0')
        add_block('simulink/Sources/Constant', [mdl '/' src], 'Value', '0', 'Position', [-420 y0 -380 y0+25]);
    else
        add_block('simulink/Sources/From Workspace', [mdl '/' src], 'VariableName', ins{k,1}, ...
                  'Position', [-440 y0 -360 y0+25]);
    end
    add_block('nesl_utility/Simulink-PS Converter', [mdl '/' cv], 'Position', [-300 y0 -270 y0+25]);
    sp([mdl '/' cv], 'Unit', ins{k,4});
    sp([mdl '/' cv], 'FilteringAndDerivatives', 'filter');
    sp([mdl '/' cv], 'SimscapeFilterOrder', '2');
    sp([mdl '/' cv], 'InputFilterTimeConstant', '0.005');
    add_line(mdl, [src '/1'], [cv '/1'], 'autorouting', 'on');
    ph = get_param([mdl '/' cv], 'PortHandles');
    pj = get_param([mdl '/' ins{k,2}], 'PortHandles');
    try
        add_line(mdl, ph.RConn(1), pj.LConn(ins{k,3}), 'autorouting', 'on');
    catch err
        warning('Connect %s to input %d of "%s" by hand (%s)', cv, ins{k,3}-1, ins{k,2}, err.message);
    end
end

%% ---------------- speed scope (same signal as our model)
add_block('simulink/Sources/From Workspace', [mdl '/speed v and v_ref'], 'VariableName', 'sm_v', 'Position', [-440 620 -360 650]);
add_block('simulink/Sinks/Scope', [mdl '/Scope speed'], 'Position', [-300 615 -260 655]);
add_line(mdl, 'speed v and v_ref/1', 'Scope speed/1', 'autorouting', 'on');

save_system(mdl, f);
fprintf('\n%s.slx created.  Press Run -> Mechanics Explorer opens and shows the robot.\n', mdl);
fprintf('(In Mechanics Explorer: View -> Fit to view, and use the play controls to replay.)\n');
end

% =================================================================================
function add_solid(mdl, name, file, rgb, pos)
blk = [mdl '/' name];
add_block('sm_lib/Body Elements/File Solid', blk, 'Position', pos);
set_param(blk, 'ExtGeomFileName', file);
set_inertia(blk);
set_color(blk, rgb);
end

function set_actuation(blk)
% every primitive: motion = provided by input, force/torque = automatically computed.
% Names in R2023+: PxMotionActuationMode / PxTorqueActuationMode (Cartesian),
%                  RxMotionActuationMode / RxTorqueActuationMode (Gimbal)
names = fieldnames(get_param(blk, 'DialogParameters'));
mot = names(~cellfun(@isempty, regexpi(names, 'MotionActuationMode$')));
frc = names(~cellfun(@isempty, regexpi(names, '(Torque|Force)ActuationMode$')));
for k = 1:numel(mot), try_values(blk, mot{k}, {'InputMotion', 'Input'}); end
for k = 1:numel(frc), try_values(blk, frc{k}, {'ComputedForce', 'ComputedTorque', 'Computed'}); end
end

function ok = try_values(blk, prm, vals)
% tries the candidate values one by one until set_param accepts one
ok = false;
for v = vals
    try
        set_param(blk, prm, v{1});
        if strcmpi(get_param(blk, prm), v{1}), ok = true; return; end
    catch
    end
end
fprintf('  !! could not set %s in "%s" (now: %s)\n', prm, get_param(blk, 'Name'), get_param(blk, prm));
end

function set_inertia(blk)
% STEP files have no density -> give each solid a custom (small) mass.
% The motion is prescribed by our model, so these values do not change the result.
names = fieldnames(get_param(blk, 'DialogParameters'));
ok = false;
if any(strcmp(names, 'InertiaType'))
    ok = try_values(blk, 'InertiaType', {'Custom', 'CustomInertia', 'PointMass', 'CalculateFromGeometry'});
end
vals = {'Mass', '0.2'; 'CenterOfMass', '[0 0 0]'; 'MomentsOfInertia', '[1e-3 1e-3 1e-3]'; ...
        'ProductsOfInertia', '[0 0 0]'; 'PointMass', '0.2'};
names = fieldnames(get_param(blk, 'DialogParameters'));
for k = 1:size(vals, 1)
    if any(strcmp(names, vals{k,1})), try, set_param(blk, vals{k,1}, vals{k,2}); catch, end, end
end
% if it is still "calculate from geometry": use density instead of "from file"
try
    if ~ok || ~isempty(regexpi(get_param(blk, 'InertiaType'), 'Geometry'))
        b = names(~cellfun(@isempty, regexpi(names, 'Based')));
        for k = 1:numel(b), try_values(blk, b{k}, {'Density', 'density'}); end
        if any(strcmp(names, 'Density')), set_param(blk, 'Density', '1000'); end
    end
catch
end
try
    fprintf('  %-26s inertia: %s\n', get_param(blk, 'Name'), get_param(blk, 'InertiaType'));
catch
    m = names(~cellfun(@isempty, regexpi(names, 'Inertia|Mass|Density|Based')));
    fprintf('  !! inertia params of "%s": %s\n', get_param(blk, 'Name'), strjoin(m.', ', '));
end
end

function set_color(blk, rgb)
% colour of the solid (names differ between releases -> try quietly)
names = fieldnames(get_param(blk, 'DialogParameters'));
vp = names(~cellfun(@isempty, regexpi(names, 'VisualProperties')));
for k = 1:numel(vp), try, set_param(blk, vp{k}, 'Simple'); catch, end, end
names = fieldnames(get_param(blk, 'DialogParameters'));
dc = names(~cellfun(@isempty, regexpi(names, 'DiffuseColor$')));
for k = 1:numel(dc), try, set_param(blk, dc{k}, mat2str(rgb)); catch, end, end
end

function sp(blk, prm, val)
% set_param that does not stop the build if a parameter name differs in your release
try
    set_param(blk, prm, val);
catch
    fprintf('  note: set "%s" = %s by hand in block "%s"\n', prm, val, get_param(blk, 'Name'));
end
end

function pc(mdl, a, pa, b, pb)
% physical connection between frame ports.  'B' = base (left), 'F' = follower (right),
% 'any' = the block's only frame port
ha = port_of([mdl '/' a], pa);  hb = port_of([mdl '/' b], pb);
add_line(mdl, ha, hb, 'autorouting', 'on');
end

function h = port_of(blk, which)
ph = get_param(blk, 'PortHandles');
switch which
    case 'B', h = ph.LConn(1);
    case 'F', h = ph.RConn(1);
    otherwise
        if ~isempty(ph.RConn), h = ph.RConn(1); else, h = ph.LConn(1); end
end
end
