function plot_poles(p_ol, p_cl, z_cl, fname)
% PLOT_POLES  readable pole map: full view + zoom on the slow (dominant) poles.
% The electrical poles (~ -2000) are 300x faster than the mechanical ones (~ -7),
% so a normal pzmap squeezes the important poles into a dot near zero.
if nargin < 3, z_cl = []; end
if nargin < 4, fname = 'Poles of the system'; end
figure('Name', fname, 'Color', 'w');
for k = 1:2
    subplot(1,2,k); hold on; grid on;
    plot(real(p_ol), imag(p_ol), 'bx', 'MarkerSize', 12, 'LineWidth', 2);
    if ~isempty(p_cl), plot(real(p_cl), imag(p_cl), 'rx', 'MarkerSize', 12, 'LineWidth', 2); end
    if ~isempty(z_cl), plot(real(z_cl), imag(z_cl), 'ro', 'MarkerSize', 9, 'LineWidth', 1.5); end
    xline(0, 'k-');  yline(0, 'k-');
    xlabel('Real axis [1/s]'); ylabel('Imag axis');
    if k == 1
        title('All poles (electrical poles far left)');
        xlim([1.15*min(real([p_ol(:); p_cl(:)])) 50]);
    else
        slow = [p_ol(abs(p_ol) < 200); p_cl(abs(p_cl) < 200); z_cl(abs(z_cl) < 200)];
        title('ZOOM: mechanical (dominant) poles');
        xlim([1.4*min(real(slow)) - 1, 3]);
        ylim(max(3, 1.3*max(abs(imag(slow)))) * [-1 1]);
        slow = slow(:);
        for i = 1:numel(slow)                  % staggered labels so they do not overlap
            dy = 0.6*(1 + mod(i-1, 3)) * (-1)^i;
            text(real(slow(i)), imag(slow(i)) + dy, sprintf('%.2f', real(slow(i))), ...
                 'HorizontalAlignment', 'center', 'FontSize', 9);
        end
    end
end
lg = {'open loop poles'};
if ~isempty(p_cl), lg{end+1} = 'closed loop poles (with PI)'; end
if ~isempty(z_cl), lg{end+1} = 'closed loop zero'; end
legend(lg, 'Location', 'southwest');
end
