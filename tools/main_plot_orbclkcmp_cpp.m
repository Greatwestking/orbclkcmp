function summaries = main_plot_orbclkcmp_cpp(varargin)
% Plot the four bundled examples after running their configurations.
% main_plot_orbclkcmp_cpp('FigureVisible', 'on') displays figures.
% main_plot_orbclkcmp_cpp('Systems', 'C') selects BDS only.
projectDir = fileparts(fileparts(mfilename('fullpath')));
opts.FigureVisible = 'off';
opts.MakeFigures = true;
opts.OutputDir = '';
opts.Systems = 'GREC';
if mod(numel(varargin), 2) ~= 0
    error('Options must be name-value pairs.');
end
for i = 1:2:numel(varargin)
    switch lower(char(varargin{i}))
        case 'figurevisible'
            opts.FigureVisible = char(varargin{i + 1});
        case 'makefigures'
            opts.MakeFigures = varargin{i + 1};
        case 'outputdir'
            opts.OutputDir = char(varargin{i + 1});
        case 'systems'
            opts.Systems = upper(char(varargin{i + 1}));
        otherwise
            error('Unknown option: %s', char(varargin{i}));
    end
end
systems = unique(opts.Systems(~isspace(opts.Systems)), 'stable');
if isempty(systems) || any(~ismember(systems, 'GREC'))
    error('The bundled examples support G, R, E, and C.');
end
jobs = {'precise_cod_wum', 'GREC'; 'regional_med', 'C'; ...
        'regional_cmed', 'C'; 'ssr_shao', 'C'};
summaries = {};
for j = 1:size(jobs, 1)
    selected = systems(ismember(systems, jobs{j, 2}));
    if isempty(selected)
        continue;
    end
    directory = fullfile(projectDir, 'output', jobs{j, 1});
    if ~isfolder(directory)
        warning('Run examples/%s.conf first. Missing: %s', jobs{j, 1}, directory);
        continue;
    end
    for system = selected
        args = {directory, system, 'both', ...
                'FigureVisible', opts.FigureVisible, 'MakeFigures', opts.MakeFigures};
        if ~isempty(opts.OutputDir)
            args = [args, {'OutputDir', fullfile(opts.OutputDir, jobs{j, 1}, system)}]; %#ok<AGROW>
        end
        summaries{end + 1, 1} = plot_orbclkcmp_cpp(args{:}); %#ok<AGROW>
    end
end
end
