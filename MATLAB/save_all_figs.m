function save_all_figs(outdir, prefix)
% SAVE_ALL_FIGS  saves every open figure as PNG (for the report) and .fig (editable)
figs = findobj(0, 'Type', 'figure');
nums = zeros(numel(figs), 1);
for k = 1:numel(figs), nums(k) = fig_number(figs(k)); end
[~, order] = sort(nums);  figs = figs(order);
for k = 1:numel(figs)
    f = figs(k);
    nm = get(f, 'Name');  if isempty(nm), nm = sprintf('figure_%d', k); end
    nm = regexprep(nm, '[^A-Za-z0-9_\- ]', '');  nm = strtrim(strrep(nm, ' ', '_'));
    base = fullfile(outdir, sprintf('%s_%02d_%s', prefix, k, nm));
    set(f, 'Color', 'w');
    try
        exportgraphics(f, [base '.png'], 'Resolution', 200);   % R2020a+
    catch
        print(f, [base '.png'], '-dpng', '-r200');
    end
    try, savefig(f, [base '.fig']); catch, end
    fprintf('  saved %s.png\n', base);
end
end

function n = fig_number(f)
try
    n = get(f, 'Number');
catch
    n = double(f);
end
end
