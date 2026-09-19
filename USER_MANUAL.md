# orbclkcmp User Manual

The software and its documentation are distributed under the [MIT License](LICENSE).
Third-party GNSS products and antenna data retain their providers' terms;
the MIT License does not grant additional rights to those data.

To run the examples, download and extract the complete package ZIP from this
repository's GitHub Releases page. Choose the package containing
`examples/data/` and `examples/results/`, not GitHub's automatically generated
"Source code" archive. The complete package includes source code, documentation,
configurations, MATLAB scripts, example inputs, and reference results.

`orbclkcmp` compares GNSS orbit and clock products. It supports precise SP3/CLK,
BRD4/BRDM broadcast navigation, optional SSR corrections, and GPS, GLONASS,
Galileo, BDS, and QZSS. Differences are always **compared minus reference**.
Use `orb`, `clk`, or `both` for orbit-only, clock-only, or combined comparison.

[Build](#build-and-run) | [Configuration](#configuration) |
[Input rules](#input-and-product-rules) | [Output](#output) |
[Algorithms](#appendix-a-orbit-clock-and-datum-equations) |
[Bundled examples](examples/README.md)

## Build and Run

Requires CMake 3.16 or later, a `C++17` compiler, and a build tool.
With GNU `g++` and Ninja on PATH, run from the folder containing `CMakeLists.txt`:

```powershell
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=g++
cmake --build build --parallel 4
.\orbclkcmp_cpp.exe cmp.conf
```

The executable is written to the package root. Omitting the configuration
argument uses `cmp.conf`; `--help` shows usage. On Linux/macOS, use
`./orbclkcmp_cpp`. Other CMake generators may be used with a compatible
compiler. Windows MinGW builds require the matching runtime DLLs on PATH.

This package includes source, shared example inputs, reference outputs, and
MATLAB plotting scripts, but no precompiled executable. The default configuration
runs the COD-WUM example. For all four examples, plotting commands, and numerical
checks, use [examples/README.md](examples/README.md).

## Configuration

Use one `key value` or `key = value` per line. `!` starts a comment; blank
lines are ignored. Paths are relative to the configuration file. Avoid spaces
in product paths. Give each run a separate `outdir`; keep `examples/data/`
and `examples/results/` unchanged.

The table lists the main keys. The product and mode restrictions below also apply.

| Key | Allowed values | Meaning |
|---|---|---|
| `year` | integer | Processing year |
| `doy_start` | `1--366` | Start DOY (day of year) |
| `doy_end` | `1--366` | End DOY |
| `antfile` | path | Reference Antenna Exchange Format (ANTEX) model |
| `brd_antfile` | path | BDS broadcast antenna model, from 2024 without SSR |
| `brd_pco` | `RADIAL, XYZ` | BDS broadcast phase-center offset (PCO) treatment |
| `dcbdir` | path | Daily bias-product directory |
| `DCBtype` | `PREFIX.EXT` | Bias-product prefix and extension |
| `reference_ac` | analysis-center (AC) tag | Reference product tag |
| `reference_orbit` | `SP3` | Reference orbit source |
| `reference_clock` | `SP3, CLK` | Reference clock source |
| `reference_dir` | path | Reference product directory |
| `compared_ac` | AC tag or empty | Compared product tag |
| `compared_orbit` | `SP3, BRD4, BRDM` | Compared orbit source |
| `compared_clock` | `SP3, CLK, BRD4, BRDM` | Compared clock source |
| `compared_dir` | path | Compared product directory |
| `reference_bdsfreq` | `B3, B1B3, B1B2` | Reference BDS clock convention |
| `compared_bdsfreq` | `B3, B1B3, B1B2` | Compared BDS clock convention |
| `bds_brd_clock_corr` | `DCB, TGD` | BDS broadcast-clock alignment without SSR |
| `interval(s)` | positive real | Comparison interval (s) |
| `mode` | `both, orb, clk` | Comparison mode |
| `systemused` | `G, R, E, C, J` (combinable) | Selected satellite systems |
| `clkdatum` | `CLK, R-C` | Datum source in combined mode |
| `clkdatum_method` | `MED, CMED, CAVG` | Median (MED), continuous median (CMED), continuous average (CAVG) |
| `ssrcorrtype` | `0, 1, 2` | 0: off; 1: orbit only; 2: BNC SSR |
| `ssr_ref` | empty or `APC, COM, APCPC, APC_B1B2, APC_B1B3, B1I_APC, IF_TO_L1` | SSR reference-point treatment; empty: default |
| `ssrdir` | path | Daily SSR directory |
| `ssr_format` | template | Daily SSR file-name template |
| `ssrfile` | path | Explicit correction-file path; overrides daily search |
| `ssrage` | positive real | Maximum SSR age (s), type 2 |
| `outdir` | path | Output directory |
| `is_cnav` | `true, false` | Civil navigation (CNAV) message selection in BRD4 |

## Input and Product Rules

### Product pairs and file names

- The reference uses SP3 orbit with SP3 or CLK clock. Broadcast products are
  compared products only.
- A CLK clock requires SP3 orbit in the same product. Broadcast orbit and clock
  must both be BRD4 or both be BRDM. The two products need not use the same type.
- Systems are selected with `G`, `R`, `E`, `C`, `J`, individually or combined.
  Processing proceeds by system and day, then by the configured epoch interval.
- Long SP3/CLK names require the configured analysis-center tag, `YYYYDDD`,
  and `_ORB.SP3` or `_CLK.CLK`. Example:
  `COD0MGXFIN_20261500000_01D_05M_ORB.SP3`.
- Traditional short names use AC + GPS week + GPS day, such as `cod24206.sp3`
  and `cod24206.clk`. Bare names without an AC tag are not automatically matched.
- Broadcast short names include `brdmDDD0.YYp` and `brd4DDD0.YYp`.
- Alternatively, set `reference` or `compared` to exact SP3 and CLK paths,
  or to one SP3 file when its embedded clock is used.
- Daily bias files use `PREFIX_YYYYDDD0000_01D_01D_DCB.EXT`, selected by
  `dcbdir` and `DCBtype=PREFIX.EXT`. Common types are `CAS0MGXRAP.BSX`,
  `CAS0OPSRAP.BIA`, `CAS1OPSRAP.BIA`, and `GFZ0OPSRAP.BIA`.

The program does not download data. Prepare the product pair, ANTEX model,
applicable bias products, and any enabled SSR files. The bundled examples
already contain their required inputs; other date ranges require new files.

### BDS clocks and navigation messages

`B3` denotes the native broadcast clock, `B1B3` the B1I/B3I combination,
and `B1B2` the B1C/B2a combination. Set each product's frequency option to
its actual clock convention, not simply to its year or analysis center.
When conventions differ, align the compared clock to the reference.

For broadcast comparison without SSR, `bds_brd_clock_corr DCB` uses external
DCB/BIA products; `TGD` uses the selected navigation record's group delays.
B1I/B3I alignment uses `2.9437*TGD1`. TGD and the later DCB frequency conversion
are not applied together. This option does not change other systems.

**RINEX 3.04 BRDM D1/D2 cannot align B1C/B2a using TGD alone.** Use B1I/B3I
for BRDM+TGD, or use external DCB/BIA or BRD4 CNV1 records containing the required
B1C/B2a parameters. For GFZ BRDM+TGD cases, use compatible `WWWW_IGS20`
products rather than `WWWW_B1C` products.

BRDM requires `is_cnav false`. For BRD4, `false` selects D1/D2 and `true`
selects the supported CNV1 branch. TGD mode cannot be combined with SSR.
With SSR enabled, specify the SSR product's actual clock convention instead
of assuming native B3.

### Antenna reference points

SP3 positions are treated as center-of-mass (COM) coordinates. `antfile`
must be consistent with the precise reference product. ANTEX blocks are
selected for the processing day at 00:00 within their validity intervals.
The selected antenna type determines BDS orbit classification. Whole-day
exclusion for a changed ANTEX satellite identity is limited to 2026-04-10--20;
later validity changes do not automatically exclude an entire day.

For BDS broadcast comparison without SSR from 2024-01-01 onward, a separate
`brd_antfile` supplies the broadcast B3/C06 PCO. Both models must identify the
same PRN and known SVN. Missing calibration, missing frequencies, non-finite
values, or mismatched identities are errors, not zero PCO. This also applies
to clock-only mode. C09/C10 use the valid C06 calibration in this branch, not
a fixed 1.27 m override. Earlier dates and SSR retain their own antenna rules.
The historical non-SSR radial rule is zero before 2017-01-17 and empirical
values from that date through 2023-12-31.

`brd_pco RADIAL` applies the broadcast up component radially. `XYZ` instead
subtracts the rotated full C06 PCO vector from broadcast APC coordinates before
orbit differencing; no second radial addition is made. It uses nominal
orbit-normal attitude for GEO and yaw steering for IGSO/MEO, not measured
attitude or special yaw maneuvers. Low-beta epochs warn rather than disappear.
XYZ is restricted to BDS broadcast orbit comparisons from 2024 with SSR off;
it is rejected for clock-only mode. It does not change the TGD/DCB or clock
PCO conventions. The radial/clock coupling is derived in Appendix A.

### SSR

SSR applies only to the compared BRD4/BRDM product. `ssrcorrtype 1` reads
text orbit corrections; `2` reads SSR/BNC orbit-clock corrections; `0` disables
corrections. Use `ssrdir` with `ssr_format` for daily files, or `ssrfile` for
an exact file overriding daily search. Date formats include `YYYYDDD`,
`DDDYY`, `YYDDD`, and `GPSDAY`; `SSRA01SHA0ddd0.yyC` is a daily template.
`compared_ac` does not identify the SSR stream.

Type 2 rejects expired corrections using `ssrage` and matches broadcast
records by issue-of-data information. For BDS, the detected SSR text format
uses the 3600 s IOD rule, while BNC uses the 720 s modulo-240 rule. Missing
acceptable corrections cause omission of that satellite epoch, not fallback
to an uncorrected broadcast result. The SSR correction frame is separate
from the RTN frame used to report differences.

Select `ssr_ref` from the product's actual reference convention:

| Value | Treatment |
|---|---|
| empty | Default broadcast PCO treatment |
| `COM` | No orbit or clock PCO correction |
| `APC` | Orbit PCO correction, no clock PCO correction |
| `B1I_APC` | Rotate the complete B1I PCO into ECEF before orbit differencing |
| `APC_B1B2`, `APC_B1B3` | Explicit BDS frequency-combination PCO |
| `APCPC` | Supported legacy radial PCO formula |
| `IF_TO_L1` | Legacy IF-to-L1 radial correction |

Do not select an antenna convention solely because it produces smaller residuals.

### Clock datum options

In `both` mode, `clkdatum CLK` estimates the datum from `dClk`, while `R-C`
uses `dClk-dR` (clock minus radial). The keyword selects an input quantity,
not a numerical datum. In `clk` mode omit `clkdatum`: the source is fixed to
`dClk`. Both modes support `MED`, `CMED`, and `CAVG`. Orbit-only mode does
not apply clock-datum compensation. Equations and screening are in Appendix A.

## Output

Each processed satellite has one text file. Epochs use GPS week and seconds
of week; all orbit and clock differences are in meters.

| Mode | Example file | Columns after `PRN week sow` |
|---|---|---|
| `both` | `SISRE_C19.txt` | `dT dN dR dClk RminusClk SISRE SISRE_Orb` |
| `orb` | `SISRE_orb_C19.txt` | `dT dN dR SISRE_Orb` |
| `clk` | `SISRE_clk_C19.txt` | `dClk` |

`dT`, `dN`, and `dR` are along-track, cross-track, and radial differences.
`dClk` is the clock difference after datum compensation; `RminusClk=dR-dClk`.
`SISRE` includes orbit and clock effects; `SISRE_Orb` excludes clock effects.
SISRE uses system-dependent weights, distinguishing BDS GEO/IGSO from MEO.
Files may contain only headers when no valid matched samples exist.

Use `tools/plot_orbclkcmp_cpp.m` for daily time series and satellite statistics.
The example guide contains MATLAB commands and expected result counts.
For comparison with the bundled reference outputs, see [examples/README.md](examples/README.md#check-the-results).

## Common Problems

| Symptom | Check |
|---|---|
| Cannot open configuration or product file | Paths are relative to the configuration file; check names, dates, and AC tags. |
| Missing navigation or bias file | Supply the file for the processing day; confirm product type and `DCBtype`. |
| No valid output rows | Check product overlap, missing clocks, antenna validity, and SSR matching/age; inspect the run log. |
| `clkdatum` rejected outside `both` | Omit it for `orb` and `clk`; retain `clkdatum_method` for clock-only processing. |
| Missing or mismatched antenna calibration | Use valid reference/broadcast blocks for the same spacecraft; do not substitute zero. |
| Windows executable cannot start | Build locally and use the matching compiler runtime DLLs on PATH. |

## Package Notes

`examples/data/` contains shared inputs; `examples/results/` contains reference outputs.
New runs write to `output/`.
The example guide lists expected result counts. The bundled examples are
independent one-day runs, not all
multi-day paper figures. Their exact scope is in the example guide.

Before public distribution, confirm input-data redistribution permissions and
set the software license and repository metadata in `CITATION.cff`. This local
folder does not itself establish a public release.

## Appendix A. Orbit, Clock, and Datum Equations

The differences below follow compared minus reference. RTN construction follows the reference orbit; the component output order is along-track, cross-track, radial.

Invalid positions are skipped. Orbit samples with an ECEF difference component
larger than 1000 m are rejected before RTN projection. This is distinct from
the 999 m clock-datum residual screening below.

For every valid satellite epoch, the Earth-centered Earth-fixed (ECEF) orbit difference is
$$
  \Delta\mathbf{x}_{\mathrm{ECEF}} =
  \mathbf{x}_{\mathrm{compared}} -
  \mathbf{x}_{\mathrm{reference}} .
$$
The comparison radial--along-track--cross-track (RTN) frame is formed from the reference orbit. Because the reference velocity is stored as an ECEF derivative, the velocity used to form the orbital plane is
$$
  \mathbf{v}_{I}^{E}=\mathbf{v}_{E}
  +\boldsymbol{\Omega}_{E}\mathbin{\times}\mathbf{r},
$$
where $\boldsymbol{\Omega}_{E}$ is the Earth-rotation vector. The orthonormal axes are
$$
  \mathbf{e}_{R}=\frac{\mathbf{r}}{\lVert\mathbf{r}\rVert},\qquad
  \mathbf{e}_{N}=\frac{\mathbf{e}_{R}\mathbin{\times}\mathbf{v}_{I}^{E}}
  {\lVert\mathbf{e}_{R}\mathbin{\times}\mathbf{v}_{I}^{E}\rVert},\qquad
  \mathbf{e}_{T}=\mathbf{e}_{N}\mathbin{\times}\mathbf{e}_{R}.
$$
The ECEF difference is then projected into
$$
  \Delta\mathbf{x}_{\mathrm{RTN}} =
  \begin{bmatrix}\mathbf{e}_{T}^{T}\\
                  \mathbf{e}_{N}^{T}\\
                  \mathbf{e}_{R}^{T}\end{bmatrix}
  \Delta\mathbf{x}_{\mathrm{ECEF}}
  =\begin{bmatrix}dT&dN&dR\end{bmatrix}^{T},
$$
where $dT$, $dN$, and $dR$ denote along-track, cross-track, and radial components, respectively. Epochs with invalid positions or orbit differences larger than the internal screening threshold are skipped.

The difference between the two product clocks is multiplied by the speed of light and adjusted for the applicable antenna phase-center offset (PCO) correction to obtain a clock difference in meters:
$$
  dClk =
  \left(t_{\mathrm{compared}} - t_{\mathrm{reference}}\right)c - dPCO ,
$$
where $c$ is the speed of light and $dPCO$ is the antenna reference-point correction applied in the clock comparison.

For broadcast-product comparisons using radial PCO corrections, let $P_{\mathrm{ref}}$ denote the reference-product clock PCO and $P_{\mathrm{brd}}$ the effective broadcast PCO. The clock correction is
$$
  dPCO = P_{\mathrm{ref}} - P_{\mathrm{brd}}.
  
$$
The same $P_{\mathrm{brd}}$ is used when transforming the broadcast orbit to the center-of-mass convention. With the signs used by the program,
$$
\begin{aligned}
  dR_{\mathrm{corr}} &= dR_{\mathrm{raw}} + P_{\mathrm{brd}}, \\
  dClk_{\mathrm{corr}} &= dClk_{\mathrm{raw}}
    - \left(P_{\mathrm{ref}}-P_{\mathrm{brd}}\right).
\end{aligned}
$$
Consequently, the broadcast PCO cancels from the coupled radial-clock term,
$$
  dR_{\mathrm{corr}}-dClk_{\mathrm{corr}}
  = dR_{\mathrm{raw}}-dClk_{\mathrm{raw}}+P_{\mathrm{ref}}.
$$
With the reference-product PCO held fixed, the same broadcast PCO cancels between the radial and clock corrections, so adjusting it does not change the radial-clock difference. Adjusting only the radial orbit without the corresponding clock correction would introduce an artificial offset.

Differences in the reference-clock realizations of clock products can introduce a common datum offset into the product differences (Chen et al., 2014). The options `clkdatum` and `clkdatum_method` select the input quantity for datum estimation and its calculation method, respectively. In combined orbit-clock mode, the datum input $q$ can be selected as:
$$
  q =
  \begin{cases}
  dClk, & \text{from CLK},\\
  dClk-dR, & \text{from CLK-R}.
  \end{cases}
$$
Clock-only mode uses $q=dClk$, while still allowing a choice of datum calculation method.

The option `clkdatum_method` selects `MED`, `CMED`, or `CAVG` in clock-only and combined orbit-clock modes. At each epoch, at least three satellites must provide valid datum inputs $q_i$. After datum estimation, satellite samples satisfying
$$
  |q_i - datum| > 999~\mathrm{m}
$$
are excluded from the epoch output and subsequent state updates of the continuous methods. The estimated datum is subtracted from the clock differences of the remaining satellites:
$$
  dClk_{\mathrm{out}} = dClk - datum .
$$

The `MED` method independently estimates the datum by epoch as the median of the valid satellite datum inputs. In regional clock products, satellites may be lost or reacquired as tracking geometry changes, altering the valid satellite set. Even if the datum inputs of continuously tracked satellites contain no jumps, a change in the set can cause the median to jump and introduce artificial changes into the compensated clock series.

The `CMED` method carries the clock datum between adjacent epochs. When the valid satellite set changes and common satellites are available, changes in their datum inputs are used to estimate the inter-epoch datum change, reducing artificial datum jumps caused by satellites entering or leaving the set. Let $S_k$ denote the valid satellite set at epoch $k$, $q_{i,k}$ the datum input of satellite $i$, and $P_{i,k-1}$ the accepted datum input stored from the previous epoch. Then,
$$
  C_k = S_k \cap S_{k-1}, \qquad
  \delta_{i,k}=q_{i,k}-P_{i,k-1}, \quad i \in C_k .
$$
The median calculated by epoch and the previous accepted median are
$$
  m_k = \operatorname{med}_{i \in S_k}(q_{i,k}), \qquad
  p_{k-1} = \operatorname{med}_{i \in S_{k-1}}(P_{i,k-1}) .
$$
When the valid satellite set changes and $C_k$ is not empty, the applied datum is propagated by the median inter-epoch change of the common satellites:
$$
  g_k = \operatorname{med}_{i \in C_k}(\delta_{i,k}),
  \qquad
  D_k = p_{k-1} - B_{k-1} + g_k,
  \qquad
  B_k = m_k - D_k .
$$
If the valid satellite set does not change, or if no common satellite is available, the fallback datum is
$$
  D_k = m_k - B_{k-1}.
$$
Here $D_k$ is the clock datum applied at epoch $k$, and $B_k$ is the accumulated datum-bias state. After outlier screening, accepted $q_{i,k}$ values are stored as $P_{i,k}$ for the next epoch. The method carries datum information between adjacent epochs to maintain clock-datum continuity and is particularly suited to regional clock products such as PPP-B2b-derived clocks. `CAVG` follows the same continuous datum treatment but uses the mean instead of the median.

## References

- Chen et al. (2014), *GNSS Clock Corrections Densification at SHAO: from 5 minutes to 30 seconds*, Science China Physics, Mechanics & Astronomy, 57, 166-175.
- Montenbruck, Steigenberger, and Hauschild (2018), *Multi-GNSS signal-in-space range error assessment: Methodology and results*, [DOI](https://doi.org/10.1016/j.asr.2018.03.041).
- Yang et al. (2020), *Basic performance and future developments of BeiDou global navigation satellite system*, [DOI](https://doi.org/10.1186/s43020-019-0006-0).
- Montenbruck and Steigenberger (2025), *The 2024 GPS accuracy improvement initiative*, [DOI](https://doi.org/10.1007/s10291-024-01793-6). General reference-point conventions, not BDS numerical calibration values.
- Montenbruck et al. (2015), *GNSS satellite geometry and attitude models*, [paper](https://elib.dlr.de/97732/1/ASR_151015_GNSS_SatGeomAtt.pdf).
