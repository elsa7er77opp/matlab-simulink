function save_simscape_video(scenario)
% SAVE_SIMSCAPE_VIDEO  builds (if needed) + runs the Simscape model and saves the
% Mechanics Explorer animation as a video on the Desktop.
%   >> save_simscape_video            % ramp   -> Desktop\Robot_Simscape_ramp.mp4
%   >> save_simscape_video('square')  % square -> Desktop\Robot_Simscape_square.mp4
if nargin < 1, scenario = 'ramp'; end
here = fileparts(mfilename('fullpath'));
addpath(here);  addpath(fileparts(here));
mdl = ['Robot_Simscape_' scenario];
f = fullfile(here, [mdl '.slx']);
build_robot_simscape(scenario);              % always rebuild (latest colours / data)
if ~bdIsLoaded(mdl), load_system(f); end
open_system(mdl);

fprintf('Running %s ... (Mechanics Explorer will open)\n', mdl);
sim(mdl);
pause(2);

try
    outdir = get_desktop_dir();
catch
    outdir = here;
end
vid = fullfile(outdir, mdl);
fprintf('Writing video ... (do not close Mechanics Explorer)\n');
try
    smwritevideo(mdl, vid, 'PlaybackSpeedRatio', 1, 'FrameRate', 30, 'VideoFormat', 'mpeg-4');
    vid = [vid '.mp4'];
catch
    smwritevideo(mdl, vid);                  % default format (.avi) for older releases
    vid = [vid '.avi'];
end
fprintf('\nSaved: %s\n', vid);
if ispc, winopen(outdir); end
end
