%% ROBOT_ANIMATION - the real robot (from our CAD) moving, driven by the state-space model
% Left : 3D robot climbing the ramp (or driving a square)
% Right: live response - speed vs reference, motor voltage, motor current
%
% scenario = 'ramp'   -> 0.3 m/s, ramp of P.theta degrees (robot_params.m)
% scenario = 'square' -> 0.6 m square on flat ground (shows the differential drive turning)
% record   = true     -> also saves an MP4 for the presentation
% (record_animation.m sets these before calling this script)
close all;
if ~exist('scenario', 'var'), scenario = 'ramp';  end    % 'ramp' or 'square'
if ~exist('record',   'var'), record   = false;   end    % true -> robot_<scenario>.mp4
if ~exist('outdir',   'var'), outdir   = pwd;     end    % where the video is saved
if ~exist('speed', 'var'), speed = 1; end                                               % playback speed (1 = real time)

P = robot_params(false);  S = robot_ss(P);
R = simulate_robot(P, S, scenario);
load('robot_mesh.mat', 'mesh');                  % exported from the CAD (metres, origin = axle centre on the ground)

%% ---------------- figure layout
fig = figure('Name', 'Robot animation', 'Color', 'w', 'Position', [60 60 1400 720]);
ax3 = axes('Parent', fig, 'Position', [0.02 0.06 0.55 0.88]);
hold(ax3, 'on'); axis(ax3, 'equal'); axis(ax3, 'off');
view(ax3, [-38 22]);

% ---- ground / ramp
th = P.theta*pi/180;  W = 0.45;
if strcmp(scenario, 'ramp')
    x1 = P.s_ramp;  x2 = x1 + P.L_ramp*cos(th);  h = P.L_ramp*sin(th);  x3 = x2 + 1.5;
    gx = [-0.6 x1 x2 x3];  gz = [0 0 h h];
    for k = 1:3                                           % road surface
        patch(ax3, [gx(k) gx(k+1) gx(k+1) gx(k)], [-W -W W W], [gz(k) gz(k+1) gz(k+1) gz(k)], ...
              [0.80 0.66 0.45], 'EdgeColor', [0.5 0.4 0.3]);
    end
    for sY = [-W W]                                       % side of the ramp
        patch(ax3, [x1 x2 x3 x3 x1], sY*[1 1 1 1 1], [0 h h 0 0], [0.70 0.55 0.35], 'EdgeColor', 'none');
    end
    patch(ax3, [-0.8 x3+0.3 x3+0.3 -0.8], [-1 -1 1 1], -0.002*[1 1 1 1], [0.92 0.92 0.92], 'EdgeColor', 'none');
else
    patch(ax3, [-0.6 1.3 1.3 -0.6], [-0.6 -0.6 1.3 1.3], [0 0 0 0], [0.92 0.92 0.92], 'EdgeColor', [0.7 0.7 0.7]);
    plot3(ax3, R.X, R.Y, 0.001*ones(size(R.X)), ':', 'Color', [0.3 0.3 0.3]);
end

% ---- robot (each mesh group is a patch; vertices are moved every frame)
fn = fieldnames(mesh);  hp = cell(numel(fn),1);   V0 = cell(numel(fn),1);
for k = 1:numel(fn)
    g = mesh.(fn{k});  V0{k} = double(g.V);
    hp{k} = patch(ax3, 'Vertices', V0{k}, 'Faces', double(g.F), 'FaceColor', double(g.color), ...
                  'EdgeColor', 'none', 'FaceLighting', 'gouraud', 'AmbientStrength', 0.45);
end
Ry = @(a) [cos(a) 0 sin(a); 0 1 0; -sin(a) 0 cos(a)];
Rz = @(a) [cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1];
trail = plot3(ax3, nan, nan, nan, 'b-', 'LineWidth', 1.5);
camlight(ax3, 'headlight'); lighting(ax3, 'gouraud');
ttl = title(ax3, '', 'FontSize', 13);

%% ---------------- live graphs
ax1 = axes('Parent', fig, 'Position', [0.63 0.71 0.35 0.23]);
plot(ax1, R.t, R.vref, 'k--'); hold(ax1, 'on'); hv = plot(ax1, nan, nan, 'b', 'LineWidth', 1.6);
grid(ax1, 'on'); ylabel(ax1, 'v [m/s]'); title(ax1, 'Speed: reference vs robot');
legend(ax1, 'v_{ref}', 'v', 'Location', 'southeast'); xlim(ax1, [0 R.t(end)]); ylim(ax1, [min(-0.05, min(R.v)) 0.45]);

ax2 = axes('Parent', fig, 'Position', [0.63 0.39 0.35 0.23]);
hVL = plot(ax2, nan, nan, 'LineWidth', 1.5); hold(ax2, 'on'); hVR = plot(ax2, nan, nan, '--', 'LineWidth', 1.5);
plot(ax2, [0 R.t(end)], P.Vmax*[1 1], 'r:');
grid(ax2, 'on'); ylabel(ax2, 'V [V]'); title(ax2, 'Motor voltage (PI output)');
legend(ax2, 'left', 'right', 'V_{max}', 'Location', 'southeast'); xlim(ax2, [0 R.t(end)]); ylim(ax2, [-P.Vmax-0.5 P.Vmax+0.5]);

ax4 = axes('Parent', fig, 'Position', [0.63 0.07 0.35 0.23]);
hiL = plot(ax4, nan, nan, 'LineWidth', 1.5); hold(ax4, 'on'); hiR = plot(ax4, nan, nan, '--', 'LineWidth', 1.5);
if strcmp(scenario, 'ramp')
    yyaxis(ax4, 'right'); hth = plot(ax4, nan, nan, 'Color', [0.85 0.33 0.1], 'LineWidth', 1.2);
    ylabel(ax4, '\theta ramp [deg]'); ylim(ax4, [0 P.theta*1.6]); yyaxis(ax4, 'left');
end
grid(ax4, 'on'); ylabel(ax4, 'i [A]'); xlabel(ax4, 't [s]'); title(ax4, 'Motor current');
xlim(ax4, [0 R.t(end)]); ylim(ax4, [min(-0.2, min(R.i(:))-0.1) max(R.i(:))+0.2]);

%% ---------------- animation
if record
    vfile = fullfile(outdir, sprintf('Robot_%s_animation.mp4', scenario));
    vw = VideoWriter(vfile, 'MPEG-4'); vw.FrameRate = 30; vw.Quality = 95; open(vw);
    set(fig, 'Resize', 'off');                       % frame size must stay the same
end
fps = 30;  idx = unique(round(linspace(1, numel(R.t), round(R.t(end)*fps))));
t0 = tic;
for j = idx
    if ~ishandle(fig), break; end
    % body pose: heading psi, pitch on the ramp, wheels turn about the axle
    Rb = Rz(R.psi(j)) * Ry(-R.pitch(j));   pos = [R.X(j) R.Y(j) R.Z(j)];
    for k = 1:numel(fn)
        Vk = V0{k};
        if strcmp(fn{k}, 'wheel_L'), Vk = (Ry(R.thL(j)) * (Vk - [0 0 P.r]).').' + [0 0 P.r]; end
        if strcmp(fn{k}, 'wheel_R'), Vk = (Ry(R.thR(j)) * (Vk - [0 0 P.r]).').' + [0 0 P.r]; end
        set(hp{k}, 'Vertices', (Rb * Vk.').' + pos);
    end
    set(trail, 'XData', R.X(1:j), 'YData', R.Y(1:j), 'ZData', R.Z(1:j) + 0.002);
    % camera follows the robot
    c = [R.X(j) R.Y(j) R.Z(j)];
    if strcmp(scenario, 'ramp')
        xlim(ax3, c(1) + [-0.7 0.7]); ylim(ax3, [-0.6 0.6]); zlim(ax3, c(3) + [-0.25 0.55]);
    else
        xlim(ax3, [-0.5 1.2]); ylim(ax3, [-0.5 1.2]); zlim(ax3, [-0.05 0.5]);
    end
    set(ttl, 'String', sprintf('t = %.2f s   v = %.2f m/s   s = %.2f m   ramp %.0f^o', ...
        R.t(j), R.v(j), R.s(j), R.theta(j)));
    % graphs
    set(hv, 'XData', R.t(1:j), 'YData', R.v(1:j));
    set(hVL, 'XData', R.t(1:j), 'YData', R.V(1:j,1)); set(hVR, 'XData', R.t(1:j), 'YData', R.V(1:j,2));
    set(hiL, 'XData', R.t(1:j), 'YData', R.i(1:j,1)); set(hiR, 'XData', R.t(1:j), 'YData', R.i(1:j,2));
    if strcmp(scenario, 'ramp'), set(hth, 'XData', R.t(1:j), 'YData', R.theta(1:j)); end
    drawnow;
    if record
        writeVideo(vw, getframe(fig));
    else
        while toc(t0) < R.t(j)/speed, end          % real-time playback
    end
end
if record, close(vw); fprintf('saved %s\n', vfile); end
