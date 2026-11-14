%% RECORD_ANIMATION - records the robot animations as MP4 videos on the Desktop
%   Robot_ramp_animation.mp4    robot climbing the ramp (+ speed, voltage, current)
%   Robot_square_animation.mp4  robot driving a square (differential drive turning)
% Just run:  >> record_animation      (takes ~1 minute, do not resize the window)
clear; clc; close all;

% ---- find the Desktop (Windows, Windows + OneDrive, Mac, Linux)
if ispc
    cands = {fullfile(getenv('OneDrive'), 'Desktop'), fullfile(getenv('USERPROFILE'), 'OneDrive', 'Desktop'), ...
             fullfile(getenv('USERPROFILE'), 'Desktop')};
else
    cands = {fullfile(getenv('HOME'), 'Desktop')};
end
outdir = pwd;
for k = 1:numel(cands)
    if ~isempty(cands{k}) && exist(cands{k}, 'dir'), outdir = cands{k}; break; end
end
fprintf('Videos will be saved in: %s\n', outdir);

record = true;
for scn = {'ramp', 'square'}
    scenario = scn{1};
    fprintf('Recording %s ...\n', scenario);
    robot_animation;
end
close all;
fprintf('\nDone. Open the Desktop:\n  %s\n  %s\n', ...
        fullfile(outdir, 'Robot_ramp_animation.mp4'), fullfile(outdir, 'Robot_square_animation.mp4'));
if ispc, winopen(outdir); end
