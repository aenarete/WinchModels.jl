## Changelog

### WinchModels v0.3.10 - 2026-08-14
#### Changed
- widen the `KiteUtils` compat bound to `"0.10, 0.11, 0.12"`; `Settings`, the only part of
  KiteUtils this package uses, is unchanged between 0.11 and 0.12
- switched `examples/` and `test/` from `ControlPlots` to `MakieControlPlots`;
  `test/plot_smooth_sign.jl` used raw PyPlot calls with no MakieControlPlots
  equivalent and was rewritten around `plot`/`savefig`
- widen the `Parameters` compat bound to `"0.12, 0.13"`; only `@with_kw`,
  `@with_kw_noshow` and `@deftype` are used, verified against 0.13.1 by running
  the test suite
- CI now tests Julia 1.11 explicitly (alongside `1` and `nightly`), matching
  what `bin/install` already required
- dropped Julia 1.10 from the `julia` compat bound (now `"1.11, 1.12"`),
  matching `bin/install`, which already rejected it

### WinchModels v0.3.9 - 2026-02-21
#### Added
- separate Project.toml files for `examples` and `test` folders
- the file `.markdownlint.json`

#### Changed
- not using `TestEnv` any longer
- make the examples work with the latest version of KiteUtils
- now tested also with Julia 1.12

### WinchModels v0.3.8 - 2025-26-08
#### Added
- Add KiteUtils v0.11 compat

### WinchModels v0.3.7 - 2025-09-04
#### Changed
- Update KiteUtils to v0.10

### WinchModels v0.3.6 - 2024-12-19
#### Added
- example speed_control_step which tests the step response of the asynchronous generator
- example speed_control_step_tc which tests the step response of the speed controller connected to the torque
  controlled generator

#### Changed
- add field `upwind_dir`, remove vector `v_wind_ref` from `Settings`and yaml files
- add the fields `max_acc`, `p_speed` and `i_speed` to `Settings`and yaml files;
  the `max_acc` value is now taken into account correctly for both winch models
- the torque controlled winch can now also operate with a `set_speed` value, using a PI controller

