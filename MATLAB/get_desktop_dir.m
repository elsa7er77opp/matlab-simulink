function d = get_desktop_dir()
% GET_DESKTOP_DIR  path of the Desktop (Windows, Windows + OneDrive, Mac, Linux)
cands = {};
if ispc
    if ~isempty(getenv('OneDrive')), cands{end+1} = fullfile(getenv('OneDrive'), 'Desktop'); end
    cands{end+1} = fullfile(getenv('USERPROFILE'), 'OneDrive', 'Desktop');
    cands{end+1} = fullfile(getenv('USERPROFILE'), 'Desktop');
else
    cands{end+1} = fullfile(getenv('HOME'), 'Desktop');
end
d = pwd;                                   % fallback: current folder
for k = 1:numel(cands)
    if exist(cands{k}, 'dir'), d = cands{k}; return; end
end
end
