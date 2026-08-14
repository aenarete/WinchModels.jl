# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A small Julia package modelling the physical dynamics of a winch/ground-station drum for
airborne wind energy systems: reel-out/reel-in acceleration as a function of tether force,
set torque or set speed, and (optionally) a brake. Two models, both `<: AbstractWinchModel`:

- `AsyncMachine` — a 20 kW asynchronous-generator ground station (the one physically built
  at TU Delft), driven through a gearbox with an optional brake below `v_min`.
- `TorqueControlledMachine` — a generic torque-controlled winch without a brake, using a
  `WinchSpeedController` (a `DiscretePID` from DiscretePIDs.jl) to turn a `set_speed` into
  a torque.

Everything hangs off one function per model: `calc_acceleration(wm, speed, force;
set_torque, set_speed, use_brake)`. `calc_force` (currently `AsyncMachine`-only —
`TorqueControlledMachine`'s equivalent is commented out in `torque_controlled_generator.jl`
pending a fix) recovers the tether force from that. The math is in `docs/winch.md`.

## Commands

```bash
bin/install       # copies the matching Manifest-vX.Y.toml.default, instantiates root +
                   # examples/ + test/; only accepts Julia 1.11/1.12 (rejects 1.10, unlike CI.yml's matrix, see below)
bin/run_julia     # LANG=en_US julia --project "$@" from the repo root; no menu(), unlike
                   # WinchControllers.jl/SimpleKiteControllers.jl's bin/run_julia
bin/release [-y]  # cut a release: Project.toml version must match the top CHANGELOG.md
                   # entry, pushes, and posts `@JuliaRegistrator register()` (with that
                   # entry as release notes) to aenarete/WinchModels.jl#1 — see that
                   # issue's history for the established format. TagBot is installed, so
                   # the vX.Y.Z tag is created automatically once the registry PR merges;
                   # this script does not tag. Keep workflow-file edits OUT of the release
                   # commit — TagBot's GITHUB_TOKEN can't push a tag for a commit that
                   # touches .github/workflows/* (see issue #25, a past manual-intervention
                   # failure for exactly this reason).
julia --project=test -e 'using Pkg; Pkg.instantiate(); Pkg.test("WinchModels")'
                   # run the test suite directly (root declares
                   # [workspace] projects = ["examples", "test"], so this and Pkg.test()
                   # from the root both resolve against one shared Manifest)
```

## Architecture

- `src/WinchModels.jl` — module root: the `AbstractWinchModel`/`AWM` abstract type, all
  exports, and the `include` order (`async_generator.jl` → `winch_controller.jl` →
  `torque_controlled_generator.jl` — the second must precede the third, since
  `TorqueControlledMachine` embeds a `WinchSpeedController`).
- `src/async_generator.jl` — `AsyncMachine`. Motor electrical model (reactance/inductance/
  resistance from nameplate data: `u_nom`, `omega_sn`, `omega_mn`, `tau_n`, `tau_b`) plus
  Coulomb + viscous drum friction. `smooth_sign` (a differentiable `x/sqrt(x²+ε²)`, `ε=6`)
  replaces `sign()` in the friction term so the acceleration stays differentiable at zero
  speed. The brake, when `use_brake=true`, engages below `0.9*v_min` and disengages above
  `1.1*v_min` (hysteresis band, not a single threshold) and simply overrides the
  acceleration to `brake_acc * speed` — it does not touch the electrical model at all.
- `src/torque_controlled_generator.jl` — `TorqueControlledMachine`. Same friction model as
  `AsyncMachine` (duplicated, not shared — the two files define their own
  `calc_coulomb_friction`/`calc_viscous_friction` methods), but the drive torque comes from
  `calc_set_torque` (via the embedded `wcs::WinchSpeedController`) instead of an electrical
  slip model. No brake logic gate on `use_brake` alone — it only engages when `set_speed`
  is also given, since the brake decision needs a speed to compare against `v_min`.
- `src/winch_controller.jl` — `WinchSpeedController` and `calc_set_torque`: a PI loop
  (`kp`/`ki` device via `DiscretePID(K=kp, Ti=kp/ki, Ts=dt)`) turning a reel-out speed error
  into a set force, then a torque via `-r/n * (0.0*set_force - err)`. Note the `0.0*` — the
  feed-forward term is multiplied out; only the PID's `err` term is live. Not obviously
  intentional; check `docs/winch.md` and git history before assuming it is or isn't a bug.
- `docs/winch.md` — the equations `calc_acceleration` implements, plus a worked test case
  with plots (`docs/force.png`, `docs/reelout-speed.png`, `docs/wind-speed.png`,
  `docs/smooth_sign.png`) reproduced by `examples/torque_control.jl`.

### The `Settings` dependency

Every model stores a `set::Settings` (from KiteUtils.jl) and reads plant parameters off it:
`drum_radius`, `gear_ratio`, `inertia_total`, `max_acc`, `sample_freq`, `max_force`,
`f_coulomb`, `c_vf`, and (torque-controlled only) `p_speed`/`i_speed`. This is the *entire*
KiteUtils surface this package touches — no `SysState`, no logging, nothing else — which is
why a KiteUtils compat bump is safe here as long as `Settings`'s fields are unchanged (true
for 0.11 → 0.12, verified by diffing `settings.jl` directly rather than trusting the
changelog).

`Settings.winch_model` (a bare string, `"AsyncMachine"` or `"TorqueControlledMachine"`) is
**not read anywhere in `src/`** — it exists only as a convention for the *caller* to branch
on when constructing a model (see `examples/speed_control_step_tc.jl`, which sets it and
then does the `if` itself). Don't expect setting it to change which model gets built.

## Environments

Root `Project.toml` declares `[workspace] projects = ["examples", "test"]`; `examples/` and
`test/` each keep their own `Project.toml` (`MakieControlPlots`, `BenchmarkTools`, etc. —
deliberately not deps of the library) with `[sources] WinchModels = {path = ".."}`, and all
three resolve against **one shared Manifest** at the root. `examples/`/`test/` were migrated
from `ControlPlots` to `MakieControlPlots`, matching `WinchControllers.jl` (a sibling package
in this ecosystem). `test/plot_smooth_sign.jl` used raw PyPlot (`plt.plot`/`plt.grid`/
`plt.savefig`), which has no MakieControlPlots equivalent — it is now `plot(x, s;
disp=true)` + `savefig(...)`, not a like-for-like API swap.

## Things that will bite

- **`test/runtests.jl`'s `KiteUtils.set_data_path("")` does NOT point at this repo's own
  `data/`.** An empty string is KiteUtils' sentinel for "use *KiteUtils'* own bundled data
  path" (`joinpath(dirname(@__DIR__), "data")` inside KiteUtils itself), not this package's
  `data/settings.yaml`. The two files mostly agree, but not on `p_speed`/`i_speed`
  (`1.0`/`0.1` in KiteUtils' bundled defaults vs `20000.0`/`5000.0` here) — harmless today
  only because `runtests.jl` exercises `AsyncMachine`, which never reads those two fields.
  If a `TorqueControlledMachine` test is ever added against `se()`, it will silently use
  KiteUtils' PI gains, not this repo's. The preceding `cd("..")` doesn't affect this
  resolution either (it's independent of `pwd()`); whether it's still doing anything useful
  for something else in the test file is not obvious — check before removing it.
- **`bin/install` rejects Julia 1.10** ("Only Julia 1.11 and 1.12 are supported"); `CI.yml`'s
  matrix agrees now (`'1'`/`'nightly'` only), but `Project.toml`'s own `julia = "1.10, 1.11,
  1.12"` compat bound still claims 1.10 support — that one is unchanged, so don't assume it
  matches what's actually tested.
- **Never use a bare `Manifest.toml`; use `Manifest-v1.11.toml` / `Manifest-v1.12.toml`.**
  This repo's per-Julia-minor-version manifests (the pattern `bin/install` relies on, copying
  the matching `Manifest-vX.Y.toml.default`) only work if there is no plain `Manifest.toml`
  sitting alongside them — its presence makes Pkg prefer it over the version-specific one,
  silently defeating the whole point of keeping 1.11 and 1.12 resolved separately. If one
  ever reappears (e.g. from a bare `Pkg.instantiate()`/`Pkg.resolve()` run without
  `--project` pointed correctly, or a manual `Pkg.add`), rename it to the version-specific
  name for whichever Julia produced it (check its `julia_version` field) rather than leaving
  it in place, or delete it if a `Manifest-vX.Y.toml` for that version already exists. All
  three (`Manifest.toml`, `Manifest-v1.11.toml`, `Manifest-v1.12.toml`) are gitignored, so
  this is a local-workspace hygiene issue, not a git one.
- **`TorqueControlledMachine.calc_force` is commented out** (`torque_controlled_generator.jl`,
  end of file), with a `# TODO: fix the calculation of the force` on the abandoned attempt.
  Only `AsyncMachine` has a working `calc_force`.
- **Friction helpers are duplicated, not shared.** `calc_coulomb_friction`/
  `calc_viscous_friction` are defined once per model file with identical bodies. A change to
  the friction model made in one is not automatically reflected in the other.
- **Registration issue lives in a different org than the sibling packages.** This repo is
  `aenarete/WinchModels.jl`; `WinchControllers.jl` and several others in this ecosystem are
  under `OpenSourceAWE`. Don't assume `gh` calls can reuse an org across the two.

## Current state

- Branch `main`, remote `aenarete/WinchModels.jl`.
- `Project.toml` is at `0.3.10` (compat: `KiteUtils = "0.10, 0.11, 0.12"`, widened from
  `"0.10, 0.11"` — verified against 0.12.0 by diffing `Settings` and by running this
  package's own test suite against it) with a matching `CHANGELOG.md` entry, not yet
  released: `bin/release` has not been run for it yet.
