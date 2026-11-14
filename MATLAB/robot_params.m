function P = robot_params(show)
% ROBOT_PARAMS  All parameters of the 2WD + caster mobile robot (Milestone 2)
%
%   P = robot_params()       -> returns the parameter struct
%   P = robot_params(true)   -> also prints the parameter table and the
%                               weight distribution on the wheels
%
% Sources:
%   [CAD]   FinalDesignForControlProject.stp  (mass, CG, inertia, wheel data)
%   [DS]    JGB37-520 12V 110 rpm datasheet (Makers Electronics listing)
%   [CALC]  calculated from the datasheet values (see comments)
%   [EST]   estimated -> to be corrected with identify_params.m after testing
%
% Change the values marked [EST] / [MEASURE] when you measure the real robot.

if nargin < 1, show = false; end

%% ------------------------------------------------------------------ general
P.g      = 9.81;            % gravity                               [m/s^2]

%% ------------------------------------------------------------ robot body [CAD]
P.m      = 2.801;           % total mass of the robot (CAD)         [kg]   [MEASURE on a scale]
P.r      = 0.065;           % drive wheel radius (D130)             [m]
P.d      = 0.250;           % track width (wheel centre to centre)  [m]
P.Iz     = 0.02858;         % yaw inertia about axle mid-point      [kg m^2]
P.x_cg   = 0.0071;          % CG x position (CAD frame)             [m]
P.h_cg   = 0.1051;          % CG height above the ground            [m]
P.x_axle = 0.025;           % drive axle x position                 [m]
P.x_cast = -0.115;          % caster wheel contact x (trailing)     [m]
P.Jwheel = 0.000387;        % one wheel (tyre + rim + hub) about axle [kg m^2]

%% ----------------------------------------------- DC motor JGB37-520  [DS]
P.V_rated = 12;             % rated voltage                         [V]
P.n_nl    = 110;            % no-load speed (gearbox output)        [rpm]
P.I_nl    = 0.15;           % no-load current                       [A]
P.I_st    = 2.4;            % stall current                         [A]
P.T_st    = 12 * P.g/100;   % stall torque 12 kg.cm -> N.m          [N m]
P.N       = 90;             % gear ratio  (11 PPR x 90 = 990 PPR)   [-]    [EST - check encoder]
P.PPR     = 11;             % encoder pulses / motor rev (one channel) [-]
P.L       = 2.5e-3;         % armature inductance                   [H]    [EST]
P.Jrotor  = 1.0e-6;         % rotor inertia (motor side)            [kg m^2] [EST]

% ---- electrical / mechanical constants calculated from the datasheet [CALC]
P.w_nl = P.n_nl*2*pi/60;                       % no-load speed      [rad/s]
P.R    = P.V_rated / P.I_st;                   % at stall w = 0 -> V = R*I
P.Ke   = (P.V_rated - P.R*P.I_nl) / P.w_nl;    % back-emf const (output shaft) [V s/rad]
P.Kt   = P.T_st / P.I_st;                      % torque const incl. gearbox losses [N m/A]
P.eta  = P.Kt / P.Ke;                          % gearbox efficiency (Kt = eta*Ke)
P.b    = P.Kt*P.I_nl / P.w_nl;                 % viscous friction (no-load balance) [N m s/rad]

%% ----------------------------------------------------- power / driver
P.V_bat  = 11.1;            % 3 x 18650 nominal                     [V]
P.V_drop = 2.0;             % L298N voltage drop                    [V]
P.Vmax   = P.V_bat - P.V_drop;  % max voltage that reaches the motor [V]
P.I_L298 = 2.0;             % L298N continuous current / channel    [A]

%% ------------------------------------------------------ ground / ramp
P.Crr    = 0.02;            % rolling resistance coefficient        [-]   [EST]
P.mu     = 0.6;             % tyre-ramp friction coefficient        [-]   [EST]
P.theta  = 15;              % ramp angle used in the simulation     [deg] [MEASURE the real ramp]
P.s_ramp = 0.5;             % ramp starts after 0.5 m               [m]
P.L_ramp = 1.2;             % ramp length                           [m]

%% --------------------------------------------- derived inertias (J*theta_dd = T)
P.Jw  = P.Jrotor*P.N^2 + P.Jwheel;        % one wheel + motor rotor seen at the wheel
P.Jeq = P.Jw + P.m*P.r^2/2;               % one wheel carrying half of the robot (straight line)

%% ---------------------------------------------- weight distribution
wb        = P.x_axle - P.x_cast;                    % axle to caster distance
P.f_cast  = (P.x_axle - P.x_cg) / wb;               % fraction on the caster
P.f_drive = 1 - P.f_cast;                           % fraction on the 2 drive wheels
P.N_wheel = P.f_drive*P.m*P.g/2;                    % normal force on ONE drive wheel [N]
P.N_cast  = P.f_cast *P.m*P.g;                      % normal force on the caster      [N]

%% ---------------------------------------------- controller sample time
P.Ts = 0.01;                % ESP32 control loop  (100 Hz)          [s]
P.Ts_link = 0.02;           % Bluetooth link to Simulink (50 Hz)    [s]

if show
    names = {'m';'r';'d';'Iz';'Jwheel';'N';'R';'L';'Ke';'Kt';'eta';'b';'Jrotor';'Jw';'Jeq';'Vmax';'Crr';'mu';'theta'};
    units = {'kg';'m';'m';'kg m^2';'kg m^2';'-';'Ohm';'H';'V s/rad';'N m/A';'-';'N m s/rad';'kg m^2';'kg m^2';'kg m^2';'V';'-';'-';'deg'};
    desc  = {'robot mass';'wheel radius';'track width';'yaw inertia';'wheel inertia';'gear ratio';...
             'armature resistance';'armature inductance';'back emf constant (output)';...
             'torque constant (output)';'gearbox efficiency';'viscous friction';'rotor inertia (motor side)';...
             'wheel + rotor at wheel';'equivalent inertia per wheel';'max motor voltage';...
             'rolling resistance';'tyre friction';'ramp angle'};
    vals = cellfun(@(n) P.(n), names);
    T = table(names, vals, units, desc, 'VariableNames', {'Parameter','Value','Unit','Description'});
    disp('===================== ROBOT PARAMETERS =====================');
    disp(T);
    fprintf('Weight distribution: drive wheels %.1f %%  |  caster %.1f %%\n', 100*P.f_drive, 100*P.f_cast);
    fprintf('Normal force: %.2f N on each drive wheel, %.2f N on the caster\n\n', P.N_wheel, P.N_cast);
end
end
