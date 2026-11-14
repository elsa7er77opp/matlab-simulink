%% EXPORT_RESULTS - runs everything and saves ALL results in Desktop\Milestone2_Results
%   - every figure as PNG (for the report / slides) + .fig
%   - everything printed in the Command Window -> Results_log.txt
%   - the state-space matrices, TF, gains -> Results_data.mat
% Just run:  >> export_results
clc; close all; diary off;
OUT = fullfile(get_desktop_dir(), 'Milestone2_Results');
if ~exist(OUT, 'dir'), mkdir(OUT); end
logfile = fullfile(OUT, 'Results_log.txt');
if exist(logfile, 'file'), delete(logfile); end
diary(logfile);
fprintf('Milestone 2 results  -  %s\n\n', datestr(now));

% 1) full model, state space, TF, steady state, ramp, PI, Bode, root locus, ramp climb
fprintf('\n#################### 1) MATHEMATICAL MODEL ####################\n');
robot_model;
OUT = fullfile(get_desktop_dir(), 'Milestone2_Results');      % (robot_model uses "clear")
save_all_figs(OUT, '1_model');
copyfile('robot_model.mat', fullfile(OUT, 'Results_data.mat'));
close all;

% 2) forward / inverse kinematics
fprintf('\n#################### 2) KINEMATICS ####################\n');
robot_kinematics;
OUT = fullfile(get_desktop_dir(), 'Milestone2_Results');
save_all_figs(OUT, '2_kinematics');
close all;

% 3) RST controller + comparison with PI
fprintf('\n#################### 3) RST CONTROLLER ####################\n');
rst_design;
OUT = fullfile(get_desktop_dir(), 'Milestone2_Results');
save_all_figs(OUT, '3_rst');
close all;

% 4) one picture of the robot on the ramp (last frame of the animation)
fprintf('\n#################### 4) ROBOT ON THE RAMP ####################\n');
scenario = 'ramp'; record = false; speed = 50;
robot_animation;
OUT = fullfile(get_desktop_dir(), 'Milestone2_Results');
save_all_figs(OUT, '4_animation');
close all;

diary off;
fprintf('\nDONE - all results are in:\n  %s\n', OUT);
if ispc, winopen(OUT); end
