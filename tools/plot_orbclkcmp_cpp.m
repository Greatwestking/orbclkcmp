function summary = plot_orbclkcmp_cpp(fdir, system, mode, varargin)
% Plot and summarize orbclkcmp C++ output files.
%
% Usage:
%   plot_orbclkcmp_cpp('output/test_2026_cod_wum', 'C', 'both')
%   plot_orbclkcmp_cpp('output/test_2026_cod_wum_orb', 'C', 'orb')
%   plot_orbclkcmp_cpp('output/test_2026_cod_wum_clk', 'C', 'clk')
%   plot_orbclkcmp_cpp(dir1, 'C', 'both', 'CompareDir', dir2)
%   plot_orbclkcmp_cpp(dir1, 'C', 'both', 'ExcludePRNs', [9 10])
% ExcludePRNs removes these PRNs from plots and statistics for this system.
%
% Mode:
%   both: SISRE_Cxx.txt      PRN week sow dT dN dR dClk RminusClk SISRE SISRE_Orb
%   orb : SISRE_orb_Cxx.txt  PRN week sow dT dN dR SISRE_Orb
%   clk : SISRE_clk_Cxx.txt  PRN week sow dClk

if nargin < 2 || isempty(system)
    system = 'C';
end
if nargin < 3 || isempty(mode)
    mode = 'auto';
end

opts = parse_options(varargin{:});
scriptDir = fileparts(mfilename('fullpath'));
fdir = resolve_input_dir(char(fdir), scriptDir);
system = char(system);
mode = lower(char(mode));
if strcmp(mode, 'auto')
    mode = detect_mode(fdir, system);
end

if isempty(opts.OutputDir)
    opts.OutputDir = fullfile(fdir, 'plot');
else
    opts.OutputDir = resolve_output_dir(char(opts.OutputDir), scriptDir);
end
if ~isempty(opts.CompareDir)
    opts.CompareDir = resolve_input_dir(char(opts.CompareDir), scriptDir);
end

[prefix, labels, valueCols, filterCol] = mode_definition(mode);
satNum = satellite_count(system);
if satNum <= 0
    error('Unsupported system: %s', system);
end

if ~isfolder(opts.OutputDir), mkdir(opts.OutputDir); end

allRows = [];
summary = struct();
summary.mode = mode;
summary.system = system;
summary.directory = fdir;
summary.satellites = [];
summary.excludedPRNs = opts.ExcludePRNs;

summaryPath = fullfile(opts.OutputDir, sprintf('all_%s_%s.txt', system, mode));
fid = fopen(summaryPath, 'w');
if fid < 0
    error('Cannot write summary file: %s', summaryPath);
end
cleanupObj = onCleanup(@() fclose(fid));

write_summary_header(fid, labels);

daySet = [];
satData = cell(satNum, 1);
for prn = 1:satNum
    if ismember(prn, opts.ExcludePRNs)
        continue;
    end
    filePath = fullfile(fdir, sprintf('%s%s%02d.txt', prefix, system, prn));
    data = read_result_file(filePath);
    if isempty(data)
        continue;
    end
    satData{prn} = data;
    daySet = union(daySet, unique(day_key(data)));
end

for di = 1:numel(daySet)
    currDay = daySet(di);
    [year, month, day] = gps_day_to_date(currDay);
    fprintf(fid, '%04d/%02d/%02d\n', year, month, day);

    dayStats = [];
    dayPrns = [];
    hTs = [];
    if opts.MakeFigures
        hTs = figure('Position', [400 200 760 460], 'Visible', opts.FigureVisible);
    end

    for prn = 1:satNum
        data = satData{prn};
        if isempty(data)
            continue;
        end
        rows = find(day_key(data) == currDay);
        if isempty(rows)
            continue;
        end

        stats = calc_stats(data(rows, :), labels, valueCols, filterCol);
        if isempty(stats)
            continue;
        end
        dayStats = [dayStats; stats]; %#ok<AGROW>
        dayPrns = [dayPrns; prn]; %#ok<AGROW>
        write_sat_summary(fid, system, prn, stats);

        if opts.MakeFigures
            plot_timeseries_panel(hTs, data(rows, :), labels, valueCols, filterCol, prn, system, year, month, day);
        end
    end

    if ~isempty(dayStats)
        write_group_summary(fid, system, dayStats, dayPrns, year, month, day, labels);
        if opts.MakeFigures
            save_timeseries_figure(hTs, opts.OutputDir, system, mode, year, month, day);
            plot_day_bars(opts.OutputDir, system, mode, year, month, day, dayStats, dayPrns, labels, opts.FigureVisible);
        end
    elseif opts.MakeFigures && ~isempty(hTs)
        close(hTs);
    end

    allRows = [allRows; pack_all_rows(dayStats, dayPrns, year, month, day)]; %#ok<AGROW>
end

allStats = [];
allPrns = [];
for prn = 1:satNum
    data = satData{prn};
    if isempty(data)
        continue;
    end
    stats = calc_stats(data, labels, valueCols, filterCol);
    if isempty(stats)
        continue;
    end
    allStats = [allStats; stats]; %#ok<AGROW>
    allPrns = [allPrns; prn]; %#ok<AGROW>
end

if ~isempty(allStats)
    fprintf(fid, 'ALL\n');
    [groupYear, groupMonth, groupDay] = group_date_from_days(daySet);
    write_group_summary(fid, system, allStats, allPrns, groupYear, groupMonth, groupDay, labels);
    if opts.MakeFigures
        plot_all_bars(opts.OutputDir, system, mode, allStats, allPrns, labels, daySet, opts.FigureVisible);
    end
end

summary.satellites = allPrns;
summary.stats = allStats;
summary.rows = allRows;
summary.summaryFile = summaryPath;

if ~isempty(opts.CompareDir)
    summary.compare = compare_directories(fdir, opts.CompareDir, opts.OutputDir, system, mode, prefix, labels, valueCols, satNum, opts.MakeFigures, opts.FigureVisible, opts.ExcludePRNs);
end
end

function opts = parse_options(varargin)
opts.OutputDir = '';
opts.CompareDir = '';
opts.MakeFigures = true;
opts.FigureVisible = 'off';
opts.ExcludePRNs = [];

i = 1;
while i <= numel(varargin)
    key = char(varargin{i});
    value = varargin{i + 1};
    switch lower(key)
        case 'outputdir'
            opts.OutputDir = char(value);
        case 'comparedir'
            opts.CompareDir = char(value);
        case 'makefigures'
            opts.MakeFigures = logical(value);
        case 'figurevisible'
            opts.FigureVisible = char(value);
        case 'excludeprns'
            if ~isnumeric(value) || ~isreal(value) || ...
                    (~isempty(value) && ~isvector(value)) || ...
                    any(~isfinite(value(:)) | value(:) < 1 | value(:) ~= fix(value(:)))
                error('ExcludePRNs must be a vector of positive integer PRNs.');
            end
            opts.ExcludePRNs = unique(double(value(:)'));
        otherwise
            error('Unknown option: %s', key);
    end
    i = i + 2;
end

end

function resolved = resolve_input_dir(pathValue, scriptDir)
if isfolder(pathValue)
    resolved = pathValue;
    return;
end
if is_absolute_path(pathValue)
    resolved = pathValue;
    return;
end

projectDir = fileparts(scriptDir);
candidates = {
    fullfile(projectDir, pathValue)
    fullfile(scriptDir, pathValue)
    fullfile(pwd, pathValue)
};
for i = 1:numel(candidates)
    if isfolder(candidates{i})
        resolved = candidates{i};
        return;
    end
end
resolved = pathValue;
end

function resolved = resolve_output_dir(pathValue, scriptDir)
if is_absolute_path(pathValue)
    resolved = pathValue;
    return;
end
projectDir = fileparts(scriptDir);
resolved = fullfile(projectDir, pathValue);
end

function tf = is_absolute_path(pathValue)
tf = startsWith(pathValue, filesep) ...
    || ~isempty(regexp(pathValue, '^[A-Za-z]:[\\/]', 'once')) ...
    || startsWith(pathValue, '\\');
end

function mode = detect_mode(fdir, system)
if exist(fullfile(fdir, sprintf('SISRE_%s01.txt', system)), 'file')
    mode = 'both';
elseif exist(fullfile(fdir, sprintf('SISRE_orb_%s01.txt', system)), 'file')
    mode = 'orb';
elseif exist(fullfile(fdir, sprintf('SISRE_clk_%s01.txt', system)), 'file')
    mode = 'clk';
else
    mode = 'both';
end
end

function [prefix, labels, valueCols, filterCol] = mode_definition(mode)
switch lower(mode)
    case 'both'
        prefix = 'SISRE_';
        labels = {'Along', 'Cross', 'Radial', 'Clk', 'R-C', 'SISRE', 'SISRE_Orb'};
        valueCols = 4:10;
        filterCol = 10;
    case 'orb'
        prefix = 'SISRE_orb_';
        labels = {'Along', 'Cross', 'Radial', 'SISRE_Orb'};
        valueCols = 4:7;
        filterCol = 7;
    case 'clk'
        prefix = 'SISRE_clk_';
        labels = {'Clk'};
        valueCols = 4;
        filterCol = 4;
    otherwise
        error('mode must be both, orb, clk, or auto: %s', mode);
end
end

function n = satellite_count(system)
switch upper(system)
    case 'G'
        n = 32;
    case 'R'
        n = 27;
    case 'C'
        n = 46;
    case 'E'
        n = 36;
    case 'J'
        n = 7;
    case 'I'
        n = 7;
    case 'L'
        n = 60;
    otherwise
        n = 0;
end
end

function data = read_result_file(filePath)
if ~isfile(filePath)
    data = [];
    return;
end

try
    data = readmatrix(filePath, 'FileType', 'text', 'NumHeaderLines', 1);
catch
    data = dlmread(filePath, '', 1, 0); %#ok<DLMRD>
end

if isempty(data)
    return;
end
data = data(~all(isnan(data), 2), :);
if isempty(data) || size(data, 2) < 4
    data = [];
end
end

function key = day_key(data)
key = data(:, 2) * 7 + floor(data(:, 3) / 86400);
end

function [year, month, day] = gps_day_to_date(dayKey)
gps0 = datenum(1980, 1, 6);
dn = gps0 + dayKey;
v = datevec(dn);
year = v(1);
month = v(2);
day = v(3);
end

function stats = calc_stats(data, labels, valueCols, filterCol)
if isempty(data)
    stats = [];
    return;
end

valid = abs(data(:, filterCol)) < 10;
if ~any(valid)
    stats = [];
    return;
end

stats.avg = nan(1, numel(labels));
stats.std = nan(1, numel(labels));
stats.rms = nan(1, numel(labels));

t = data(:, 2) * 7 + data(:, 3) / 86400;
breakOrbit = find(diff(t) > 0.25);
breakClock = find(diff(t) > 5 / 1440);

for k = 1:numel(labels)
    values = data(:, valueCols(k));
    stats.avg(k) = mean(values(valid), 'omitnan');
    stats.rms(k) = sqrt(mean(values(valid).^2, 'omitnan'));
    if strcmp(labels{k}, 'Clk')
        stats.std(k) = segmented_std(values, valid, breakClock);
    elseif strcmp(labels{k}, 'R-C')
        stats.std(k) = segmented_std(values, valid, breakOrbit);
    else
        stats.std(k) = std(values(valid), 'omitnan');
    end
end
end

function s = segmented_std(values, valid, breakIndex)
starts = [1; breakIndex(:) + 1];
ends = [breakIndex(:); numel(values)];
baseline = nan(size(values));
for i = 1:numel(starts)
    idx = starts(i):ends(i);
    baseline(idx) = mean(values(idx), 'omitnan');
end
res = values(valid) - baseline(valid);
s = sqrt(mean(res.^2, 'omitnan'));
end

function write_summary_header(fid, labels)
fprintf(fid, '%6s', 'SAT');
for group = {'AVG', 'STD', 'RMS'}
    for k = 1:numel(labels)
        fprintf(fid, '%14s', [group{1} '_' labels{k}]);
    end
end
fprintf(fid, '\n');
end

function write_sat_summary(fid, system, prn, stats)
fprintf(fid, '%1s%02d', system, prn);
write_stat_values(fid, stats);
end

function write_group_summary(fid, system, statsRows, prns, year, month, day, labels)
if isempty(statsRows)
    return;
end
allStats = mean_stats(statsRows);
fprintf(fid, '==%s ', system);
write_stat_values(fid, allStats);

if system == 'C'
    groups = bds_groups(prns, year, month, day);
    groupNames = {'GEO', 'IGSO', 'MEO', 'Other'};
    for i = 1:numel(groupNames)
        idx = strcmp(groups, groupNames{i});
        if any(idx)
            fprintf(fid, '%-6s', groupNames{i});
            write_stat_values(fid, mean_stats(statsRows(idx)));
        end
    end
end
end

function stats = mean_stats(statsRows)
avg = vertcat(statsRows.avg);
stdv = vertcat(statsRows.std);
rms = vertcat(statsRows.rms);
stats.avg = mean(avg, 1, 'omitnan');
stats.std = mean(stdv, 1, 'omitnan');
stats.rms = mean(rms, 1, 'omitnan');
end

function write_stat_values(fid, stats)
for k = 1:numel(stats.avg)
    fprintf(fid, '%14.3f', stats.avg(k));
end
for k = 1:numel(stats.std)
    fprintf(fid, '%14.3f', stats.std(k));
end
for k = 1:numel(stats.rms)
    fprintf(fid, '%14.3f', stats.rms(k));
end
fprintf(fid, '\n');
end

function rows = pack_all_rows(statsRows, prns, year, month, day)
rows = struct('prn', {}, 'year', {}, 'month', {}, 'day', {}, 'avg', {}, 'std', {}, 'rms', {});
for i = 1:numel(prns)
    rows(i, 1).prn = prns(i); %#ok<AGROW>
    rows(i, 1).year = year;
    rows(i, 1).month = month;
    rows(i, 1).day = day;
    rows(i, 1).avg = statsRows(i).avg;
    rows(i, 1).std = statsRows(i).std;
    rows(i, 1).rms = statsRows(i).rms;
end
end

function groups = bds_groups(prns, year, month, day)
groups = cell(numel(prns), 1);
useNew = false;
if year > 0
    doy = datenum(year, month, day) - datenum(year, 1, 0);
    useNew = (year * 1000 + doy) >= 2026100;
end
for i = 1:numel(prns)
    prn = prns(i);
    if useNew
        if prn >= 1 && prn <= 4
            groups{i} = 'GEO';
        elseif prn >= 6 && prn <= 10
            groups{i} = 'IGSO';
        elseif prn >= 11 && prn <= 42
            groups{i} = 'MEO';
        else
            groups{i} = 'Other';
        end
    else
        if ismember(prn, [38, 39, 40])
            groups{i} = 'IGSO';
        else
            groups{i} = 'MEO';
        end
    end
end
end

function plot_timeseries_panel(h, data, labels, valueCols, filterCol, prn, system, year, month, day)
set(0, 'CurrentFigure', h);
valid = abs(data(:, filterCol)) < 10;
if ~any(valid)
    return;
end
t = mod(data(:, 3), 86400) / 3600;
panelCount = min(numel(labels), 6);
color = sat_color(prn, system);
for k = 1:panelCount
    subplot('Position', panel_position_2x3(k));
    plot(t(valid), data(valid, valueCols(k)), '.', 'Color', color, 'DisplayName', sprintf('%02d', prn));
    hold on;
    grid on;
    title(plot_label(labels{k}));
    ylabel('m');
    apply_axis_style();
    if k <= 2 || k == 5
        set(gca, 'xticklabel', '');
    elseif k == 4
        xlabel(sprintf('%s Time(h) %04d/%02d/%02d', system, year, month, day));
    else
        xlabel('Time(h)');
    end
end
end

function save_timeseries_figure(h, outDir, system, mode, year, month, day)
if isempty(h)
    return;
end
ds = sprintf('%04d%02d%02d', year, month, day);
save_figure_pair(h, outDir, figure_base('TS', system, mode, ds));
end

function plot_day_bars(outDir, system, mode, year, month, day, statsRows, prns, labels, figureVisible)
ds = sprintf('%04d%02d%02d', year, month, day);
plot_rms_panel(outDir, figure_base('RMS', system, mode, ds), statsRows, prns, labels, mode, system, sprintf('%s %04d/%02d/%02d', system, year, month, day), false, year, month, day, figureVisible);
plot_mode_metric_bars(outDir, system, mode, ds, statsRows, prns, labels, sprintf('%s %04d/%02d/%02d', system, year, month, day), false, year, month, day, figureVisible);
end

function plot_all_bars(outDir, system, mode, statsRows, prns, labels, daySet, figureVisible)
[groupYear, groupMonth, groupDay] = group_date_from_days(daySet);
plot_rms_panel(outDir, figure_base('RMS', system, mode, 'ALL'), statsRows, prns, labels, mode, system, sprintf('%s ALL', system), true, groupYear, groupMonth, groupDay, figureVisible);
plot_mode_metric_bars(outDir, system, mode, 'ALL', statsRows, prns, labels, sprintf('%s ALL', system), true, groupYear, groupMonth, groupDay, figureVisible);
end

function [year, month, day] = group_date_from_days(daySet)
year = 0;
month = 0;
day = 0;
if ~isempty(daySet)
    [year, month, day] = gps_day_to_date(daySet(1));
end
end

function plot_rms_panel(outDir, baseName, statsRows, prns, labels, mode, system, xText, showGroups, year, month, day, figureVisible)
if isempty(statsRows)
    return;
end
panels = rms_panel_definition(mode);
if isempty(panels)
    return;
end
h = figure('Position', [400 200 700 420], 'Visible', figureVisible);
for k = 1:size(panels, 1)
    labelIndex = find(strcmp(labels, panels{k, 1}), 1);
    if isempty(labelIndex)
        continue;
    end
    values = metric_values(statsRows, panels{k, 2}, labelIndex);
    subplot('Position', panel_position_2x2(k));
    b = bar(values, 'FaceColor', bar_color(system), 'EdgeColor', 'none'); %#ok<NASGU>
    grid on;
    title(panel_title(panels{k, 3}, values, prns, system, showGroups, year, month, day));
    ylabel('m');
    xlim([0 numel(prns) + 1]);
    apply_axis_style();
    set(gca, 'XTick', 1:numel(prns));
    if k >= 3
        set(gca, 'XTickLabel', prn_labels(prns));
        set(gca, 'XTickLabelRotation', 90);
        xlabel(xText);
    else
        set(gca, 'xticklabel', '');
    end
end
save_figure_pair(h, outDir, baseName);
end

function plot_mode_metric_bars(outDir, system, mode, suffix, statsRows, prns, labels, xText, showGroups, year, month, day, figureVisible)
if strcmp(mode, 'both')
    plot_single_metric_bar(outDir, figure_base('SISRE', system, mode, suffix), statsRows, prns, labels, 'rms', 'SISRE', 'SISRE (RMS)', system, xText, showGroups, year, month, day, figureVisible);
    plot_single_metric_bar(outDir, figure_base('CLK_STD', system, mode, suffix), statsRows, prns, labels, 'std', 'Clk', 'Clock (STD)', system, xText, showGroups, year, month, day, figureVisible);
    if strcmp(suffix, 'ALL')
        plot_single_metric_bar(outDir, figure_base('ORB_RMS', system, mode, suffix), statsRows, prns, labels, 'rms', 'SISRE_Orb', 'Orbit (RMS)', system, xText, showGroups, year, month, day, figureVisible);
    end
elseif strcmp(mode, 'orb')
    plot_single_metric_bar(outDir, figure_base('ORB_RMS', system, mode, suffix), statsRows, prns, labels, 'rms', 'SISRE_Orb', 'Orbit (RMS)', system, xText, showGroups, year, month, day, figureVisible);
elseif strcmp(mode, 'clk')
    plot_single_metric_bar(outDir, figure_base('CLK_STD', system, mode, suffix), statsRows, prns, labels, 'std', 'Clk', 'Clock (STD)', system, xText, showGroups, year, month, day, figureVisible);
end
end

function plot_single_metric_bar(outDir, baseName, statsRows, prns, labels, fieldName, labelName, titleText, system, xText, showGroups, year, month, day, figureVisible)
labelIndex = find(strcmp(labels, labelName), 1);
if isempty(labelIndex) || isempty(statsRows)
    return;
end
values = metric_values(statsRows, fieldName, labelIndex);
h = figure('Position', [400 200 700 420], 'Visible', figureVisible);
bar(values, 'FaceColor', bar_color(system), 'EdgeColor', 'none');
grid on;
xlim([0 numel(prns) + 1]);
apply_axis_style();
title(panel_title(titleText, values, prns, system, showGroups, year, month, day));
ylabel('m');
set(gca, 'XTick', 1:numel(prns));
set(gca, 'XTickLabel', prn_labels(prns));
set(gca, 'XTickLabelRotation', 90);
xlabel(xText);
save_figure_pair(h, outDir, baseName);
end

function panels = rms_panel_definition(mode)
if strcmp(mode, 'both')
    panels = {
        'Along', 'rms', 'Along (RMS)'
        'Cross', 'rms', 'Cross (RMS)'
        'Radial', 'rms', 'Radial (RMS)'
        'R-C', 'std', 'R-Clk (STD)'
    };
elseif strcmp(mode, 'orb')
    panels = {
        'Along', 'rms', 'Along (RMS)'
        'Cross', 'rms', 'Cross (RMS)'
        'Radial', 'rms', 'Radial (RMS)'
        'SISRE_Orb', 'rms', 'Orbit (RMS)'
    };
else
    panels = {};
end
end

function values = metric_values(statsRows, fieldName, labelIndex)
allValues = vertcat(statsRows.(fieldName));
values = allValues(:, labelIndex);
end

function titleText = panel_title(baseText, values, prns, system, showGroups, year, month, day)
titleText = sprintf('%s, Mean=%.2fm', baseText, mean(values, 'omitnan'));
if showGroups && strcmp(system, 'C')
    groups = bds_groups(prns, year, month, day);
    for name = {'GEO', 'IGSO', 'MEO'}
        idx = strcmp(groups, name{1});
        if any(idx)
            titleText = [titleText, sprintf(', %s=%.2fm', name{1}, mean(values(idx), 'omitnan'))]; %#ok<AGROW>
        end
    end
end
end

function save_figure_pair(h, outDir, baseName)
saveas(h, fullfile(outDir, [baseName '.jpg']));
saveas(h, fullfile(outDir, [baseName '.fig']));
end

function baseName = figure_base(tag, system, mode, suffix)
if strcmp(mode, 'both')
    baseName = sprintf('%s_%s_%s', tag, system, suffix);
else
    baseName = sprintf('%s_%s_%s_%s', tag, system, mode, suffix);
end
end

function labelsText = prn_labels(prns)
labelsText = arrayfun(@(x) sprintf('%02d', x), prns, 'UniformOutput', false);
end

function textValue = plot_label(label)
if strcmp(label, 'Clk')
    textValue = 'Clock';
elseif strcmp(label, 'R-C')
    textValue = 'R-Clk';
elseif strcmp(label, 'SISRE_Orb')
    textValue = 'Orbit';
else
    textValue = label;
end
end

function color = bar_color(system)
idx = system_index(system);
color = [0, idx * 0.1, abs(1 - idx * 0.2)];
color = max(0, min(1, color));
end

function color = sat_color(prn, system)
colors = lines(max(64, satellite_count(system)));
color = colors(mod(prn - 1, size(colors, 1)) + 1, :);
end

function idx = system_index(system)
systems = 'GRCEJIL';
idx = find(systems == upper(system), 1);
if isempty(idx)
    idx = 3;
end
end

function pos = panel_position_2x3(k)
positions = {
    [0.08 0.55 0.28 0.4]
    [0.38 0.55 0.28 0.4]
    [0.08 0.10 0.28 0.4]
    [0.38 0.10 0.28 0.4]
    [0.70 0.55 0.28 0.4]
    [0.70 0.10 0.28 0.4]
};
pos = positions{k};
end

function pos = panel_position_2x2(k)
positions = {
    [0.08 0.55 0.40 0.4]
    [0.58 0.55 0.40 0.4]
    [0.08 0.10 0.40 0.4]
    [0.58 0.10 0.40 0.4]
};
pos = positions{k};
end

function apply_axis_style()
set(gca, 'linewidth', 1);
set(gca, 'box', 'on');
set(gca, 'color', [1 0.98 0.96]);
end

function result = compare_directories(dir1, dir2, outDir, system, mode, prefix, labels, valueCols, satNum, makeFigures, figureVisible, excludePRNs)
diffPath = fullfile(outDir, sprintf('diff_%s_%s.txt', system, mode));
fid = fopen(diffPath, 'w');
if fid < 0
    error('Cannot write diff file: %s', diffPath);
end
cleanupObj = onCleanup(@() fclose(fid));

fprintf(fid, '%6s%8s', 'SAT', 'N');
for k = 1:numel(labels)
    fprintf(fid, '%14s%14s', ['RMS_' labels{k}], ['MAX_' labels{k}]);
end
fprintf(fid, '\n');

result = struct();
result.file = diffPath;
result.rows = [];

for prn = 1:satNum
    if ismember(prn, excludePRNs)
        continue;
    end
    file1 = fullfile(dir1, sprintf('%s%s%02d.txt', prefix, system, prn));
    file2 = fullfile(dir2, sprintf('%s%s%02d.txt', prefix, system, prn));
    a = read_result_file(file1);
    b = read_result_file(file2);
    if isempty(a) || isempty(b)
        continue;
    end
    key1 = a(:, 2) * 604800 + a(:, 3);
    key2 = b(:, 2) * 604800 + b(:, 3);
    [~, ia, ib] = intersect(key1, key2);
    if isempty(ia)
        continue;
    end
    d = a(ia, valueCols) - b(ib, valueCols);
    rms = sqrt(mean(d.^2, 1, 'omitnan'));
    maxAbs = max(abs(d), [], 1, 'omitnan');
    fprintf(fid, '%1s%02d%8d', system, prn, numel(ia));
    for k = 1:numel(labels)
        fprintf(fid, '%14.6g%14.6g', rms(k), maxAbs(k));
    end
    fprintf(fid, '\n');

    row.prn = prn;
    row.count = numel(ia);
    row.rms = rms;
    row.maxAbs = maxAbs;
    result.rows = [result.rows; row]; %#ok<AGROW>
end

if makeFigures && ~isempty(result.rows)
    plot_compare_bars(outDir, system, mode, result.rows, labels, figureVisible);
end
end

function plot_compare_bars(outDir, system, mode, rows, labels, figureVisible)
prns = [rows.prn]';
rms = vertcat(rows.rms);
for k = 1:numel(labels)
    h = figure('Position', [400 200 700 420], 'Visible', figureVisible);
    bar(rms(:, k), 'FaceColor', [0.75 0.25 0.12], 'EdgeColor', 'none');
    grid on;
    apply_axis_style();
    xlim([0 numel(prns) + 1]);
    title(sprintf('Difference RMS %s, Mean=%.6g m', labels{k}, mean(rms(:, k), 'omitnan')));
    ylabel('m');
    set(gca, 'XTick', 1:numel(prns));
    set(gca, 'XTickLabel', prn_labels(prns));
    set(gca, 'XTickLabelRotation', 90);
    save_figure_pair(h, outDir, sprintf('DIFF_RMS_%s_%s_%s', system, mode, labels{k}));
end
end
