# Bundled examples

See the [user manual](../USER_MANUAL.md) for build instructions, all options,
and the complete algorithm appendices.

Download the complete package ZIP from this repository's GitHub Releases page
and extract it before running these examples. It includes `examples/data/`
and `examples/results/`; GitHub's automatic "Source code" archive does not
include these separately distributed data and results.

Run commands from the package root after building. All examples use a 30 s
comparison interval, `both` mode, and compared-minus-reference differences.

| Configuration | Day | Systems | Reference | Compared | Datum |
|---|---|---|---|---|---|
| precise_cod_wum.conf | 2026-05-30, DOY 150 | G, R, E, C | COD SP3/CLK | WUM SP3/CLK | R-C / CMED |
| regional_med.conf | 2026-05-30, DOY 150 | C | COD SP3/CLK | PPP-B2b-derived SP3/CLK | R-C / MED |
| regional_cmed.conf | 2026-05-30, DOY 150 | C | COD SP3/CLK | PPP-B2b-derived SP3/CLK | R-C / CMED |
| ssr_shao.conf | 2026-07-16, DOY 197 | C | GBM SP3/CLK | BRDM + SHAO A01 SSR | CLK / CMED |

The SSR configuration uses `B1I_APC`, `B1B2` clock conventions, and a 30 s
maximum SSR age. The BDS precise clocks in all examples are configured as
`B1B2`. The files under `data/` are shared across configurations where possible.
The full ANTEX file is retained, including its validity intervals.

`examples/data/` contains inputs, `examples/results/<case>/` contains reference
results, and `output/<case>/` receives new results. Keep the bundled inputs
and reference results unchanged.

## Run

```powershell
.\orbclkcmp_cpp.exe examples/precise_cod_wum.conf
.\orbclkcmp_cpp.exe examples/regional_med.conf
.\orbclkcmp_cpp.exe examples/regional_cmed.conf
.\orbclkcmp_cpp.exe examples/ssr_shao.conf
```

On Linux/macOS, replace `.\orbclkcmp_cpp.exe` with `./orbclkcmp_cpp`.
Paths in each configuration are relative to that configuration file; no local
drive paths need editing. Run any one example or all four.

Each successful run creates `output/<configuration-name>/SISRE_<PRN>.txt`.
The header is:

```text
PRN week sow dT dN dR dClk RminusClk SISRE SISRE_Orb
```

The last seven columns are in meters. Some satellite files contain only a
header because no valid matched data exist. This is not an execution failure.
Expected counts from the GNU 14.2.0 Release build are listed below. Count
data rows only, excluding headers. These are basic execution checks, not
proof of numerical agreement.

| Case | Satellites with data | Data rows |
|---|---:|---:|
| precise_cod_wum | 107 | 306876 |
| regional_med | 30 | 33874 |
| regional_cmed | 30 | 33874 |
| ssr_shao | 34 | 91540 |

## Plot using MATLAB

Set the current MATLAB folder to the package root, then run:

```matlab
addpath(fullfile(pwd, 'tools'))
main_plot_orbclkcmp_cpp
```

Figures and statistics are written under each result directory's `plot`
folder. Use `main_plot_orbclkcmp_cpp('FigureVisible', 'on')` to display windows,
or `main_plot_orbclkcmp_cpp('Systems', 'C')` for BDS only. The entry point plots
GREC for COD-WUM and C for the other three examples. Missing result directories
are skipped with a warning.

## Check the results

Check the counts above and the satellite IDs and GPS epochs in the output.
MED and CMED use the same products and should have identical orbit differences
and matched rows; their clock-datum treatments differ. Reference outputs in
`examples/results/<case>/` were generated from this release with GNU 14.2.0
in Release mode. They are reproducibility baselines, not independent accuracy
truth, and apply only to the supplied inputs and configurations.

After running the examples, set the MATLAB current folder to the package root:

```matlab
addpath(fullfile(pwd, 'tools'))
cases = {'precise_cod_wum', 'GREC'; 'regional_med', 'C'; ...
         'regional_cmed', 'C'; 'ssr_shao', 'C'};
for k = 1:size(cases, 1)
    for sys = cases{k, 2}
        plot_orbclkcmp_cpp(fullfile(pwd, 'output', cases{k, 1}), sys, 'both', ...
            'CompareDir', fullfile(pwd, 'examples', 'results', cases{k, 1}), ...
            'MakeFigures', false);
    end
end
```

Read `output/<case>/plot/diff_<system>_both.txt`. `N` counts matched epochs;
`RMS_*` and `MAX_*` describe differences between the two runs in meters.
Exact numerical agreement gives zero differences. This comparison uses common
epochs only: also check file names, epoch lists, and row counts, since missing
files or epochs are not reported as mismatches. If running only one example,
keep only its row in `cases`.

Outputs are rounded to 0.001 m. Different compilers may produce small
rounding-boundary differences; investigate larger or systematic differences.

These examples are independent one-day runs. They do not include the previous
day's navigation carry-over or the complete date ranges of the paper figures.
