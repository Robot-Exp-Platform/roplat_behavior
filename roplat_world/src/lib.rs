//! Contracts for the worlds robots run in: simulator backends that load a
//! robot's model ([`SimulationBackend`], [`select_simulation_model`]),
//! physics engines that step ([`PhysicsEngine`]), renderers ([`Renderer`]),
//! and the entities placed in a world ([`Entity`], [`AddRobot`],
//! [`AddCollision`], [`AddVisual`]). Feature `roplat` adds
//! [`SimulationRhythm`], which steps an engine as a roplat rhythm.
//! [`KinematicArm`] is a simulated arm without dynamics whose panel
//! (`roplat_behavior::Panel`) is the reference for drivers that stop a motion
//! looping in their own process.
//!
//! A robot type declares its model files through
//! `roplat_behavior::RobotDescription`; this crate chooses among them for a
//! backend and resolves the shared asset directory ([`assets_dir`]).

mod assets;
mod physics_engine;
mod renderer;
mod simulation;
#[cfg(feature = "roplat")]
mod simulation_rhythm;
mod world;

pub use assets::assets_dir;
pub use physics_engine::{AddSearchPath, PhysicsEngine};
pub use renderer::{AttachFrom, Renderer};
pub use simulation::{
    ModelSelectionError, ModelSource, SelectedSimulationModel, SimulationBackend,
    select_simulation_model,
};
#[cfg(feature = "roplat")]
pub use simulation_rhythm::{SimPacing, SimulationRhythm};
pub use world::{AddCollision, AddRobot, AddVisual, Collision, Entity, EntityBuilder, Visual};

// A simulated arm with a panel (roplat_behavior 0.10).
mod kinematic_arm;
pub use kinematic_arm::{Hardware, KinematicArm};
