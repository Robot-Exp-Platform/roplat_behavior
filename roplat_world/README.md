# roplat_world

[English](README.md) | [简体中文](README_zh.md)

Contracts for the worlds robots run in: simulator backends that load a robot's model, physics engines that step, renderers, and the entities placed in a world.

Part of the [roplat_behavior workspace](../README.md). 0.9 is not on crates.io yet; build it from source as [Working from source](../README.md#working-from-source) describes.

## Contents

- Simulator backends and model selection: `SimulationBackend`, `select_simulation_model`.
- Physics engines (`PhysicsEngine`) and renderers (`Renderer`).
- World entities: `Entity`, `AddRobot`, `AddCollision`, `AddVisual`.
- The shared asset directory (`assets_dir`).
- `SimulationRhythm`, which steps an engine as a roplat rhythm (feature `roplat`).
- `KinematicArm`, a simulated arm without dynamics whose `Panel` stops a motion from another thread, with `Hardware` playing its emergency-stop button and protective stop. It is the reference implementation of the panel contract and passes `roplat_behavior::conformance::check_panel`.

A robot type declares its model files through `roplat_behavior::RobotDescription`; this crate chooses among them for a backend.

## Combining robot models and simulator backends

`RobotDescription::SIMULATION_MODELS` can offer different models of the same
device. A loader implementing `SimulationBackend` declares its identifier and
accepted formats; `select_robot_model::<R>()` selects the matching asset before
loading it. Existing descriptions that only provide `URDF` remain supported by
loaders accepting URDF.

```rust
use roplat_behavior::{ModelFormat, RobotDescription, SimulationModel};

struct ArmModels;
impl RobotDescription for ArmModels {
    const SIMULATION_MODELS: &'static [SimulationModel] = &[
        SimulationModel::new(ModelFormat::Urdf, "arm/robot.urdf"),
        SimulationModel::new(ModelFormat::Mjcf, "arm/scene.xml").for_backend("mujoco"),
    ];
}
```

Selection prefers a backend-specific model, then a portable declaration, then
legacy `URDF`. Within a tier, backend format preference comes before declaration
order. The returned `SelectedSimulationModel` records the source; an unsupported
combination returns `ModelSelectionError` naming the backend and formats. Paths
follow that backend's asset/search-path rules. A selected file's loading failure
must be reported, rather than silently choosing different physics.

This is model selection, not automatic conversion or simulator implementation.
The backend still validates the model and provides its actual joint, control,
sensor, and scene capabilities. Platform/build availability belongs to each
backend's installation contract; accepting MJCF does not imply that MuJoCo is
installed or supported on the current machine. `PhysicsEngine` continues to own
stepping separately from the robot's existing motion and control traits. The
[selection tests](tests/simulation_models.rs) demonstrate compatible and
incompatible device/backend combinations without starting a simulator.

## Stepping a simulation in a roplat graph

The optional `SimulationRhythm<E>` owns any compatible `PhysicsEngine`
and drives finite unit ticks in a real `#[roplat::system]` graph. Use
`SimulationRhythm::logical(engine).with_max_steps(n)` for accelerated execution,
or `SimulationRhythm::new(engine, period)` for wall-clock pacing. Physics dt
belongs to the engine; pacing never changes it. Budgets reset per drive without
resetting the world. A step error retains its source, stops the domain, and
prevents a downstream tick for the failed call. Cooperative stopping cancels
idle waits but waits for an active domain to return its owned nodes.

This shared adapter depends on no Bullet or MuJoCo package. Its Tokio dependency
is enabled only with the `roplat` feature; default behavior builds remain free
of that framework/runtime dependency. The backend retains responsibility for
state freshness, command ordering, physical semantics and resource cleanup.
The adapter does not reset an engine on entry or call its consuming `shutdown`
method on exit; use the backend's RAII cleanup or recover it with `into_engine()`.
See the [rhythm tests](tests/simulation_rhythm.rs).

## Features

| Feature | Purpose |
|---|---|
| `roplat` | `SimulationRhythm`; adds `roplat`, `futures` and `tokio`. |

Default features are empty.

## License

Apache-2.0; see [LICENSE](../LICENSE).
