# Robot Example Checklist

This checklist describes the example set a robot driver should provide before it
can be treated as a reasonably proven `roplat_behavior` implementation.

- [ ] 00 connection and lifecycle
  - [ ] Lifecycle smoke test
    - Recommended file: `00_00_lifecycle.rs`
    - Trait: `Robot`
    - Expected result: print the driver version, connect, run supported lifecycle calls, and exit without motion.
  - [ ] Safety recovery smoke test
    - Recommended file: `00_01_recovery.rs`
    - Trait: `Panel` (from `HasPanel::panel`)
    - Expected result: stop a motion from another thread through the panel; latch an emergency stop, show that `enable` fails until `clear`, then `clear` and `enable`; print `status()` after each step. On hardware, also show that `clear` answers `LocalResetRequired` while the device's own emergency-stop button is pressed.

- [ ] 01 state reading
  - [ ] Read one full state
    - Recommended file: `01_00_read_state.rs`
    - Trait: `Robot`
    - Expected result: print key measured fields from one `Robot::State`.
  - [ ] Echo state in a short loop
    - Recommended file: `01_01_echo_state.rs`
    - Trait: `Robot`
    - Expected result: repeatedly print state samples and show communication errors as returned `RobotResult` errors.

- [ ] 02 joint motion
  - [ ] Move to the documented default joint pose
    - Recommended file: `02_00_move_joint_default.rs`
    - Trait: `Motion`, `MoveTo<JointSpace<N>>`
    - Expected result: robot reaches the default joint pose and `move_to` returns after completion.
  - [ ] Move a small safe joint offset and return
    - Recommended file: `02_01_move_joint_offset.rs`
    - Trait: `Motion`, `MoveTo<JointSpace<N>>`
    - Expected result: robot executes a bounded relative joint move and returns to a known pose.
  - [ ] Follow a dense joint trajectory
    - Recommended file: `02_02_move_joint_trajectory.rs`
    - Trait: `Motion`, `MoveTraj<JointSpace<N>>`
    - Expected result: robot consumes a sampled joint trajectory without requiring an external planner.
  - [ ] Follow a trajectory loaded from file
    - Recommended file: `02_03_move_joint_trajectory_file.rs`
    - Trait: `MotionFile`, `MoveTraj<JointSpace<N>>`
    - Expected result: robot loads a JSON trajectory and executes it.
  - [ ] Follow sparse joint waypoints
    - Recommended file: `02_04_move_joint_waypoints.rs`
    - Trait: `Motion`, `MoveTraj<JointSpace<N>>`
    - Expected result: driver interpolates waypoints or clearly reports that waypoint planning is unsupported.

- [ ] 03 Cartesian motion
  - [ ] Move the flange or TCP by a small safe offset
    - Recommended file: `03_00_move_flange_offset.rs`
    - Trait: `Motion`, `MoveTo<FlangeSpace>` or `MoveTo<TcpSpace>`
    - Expected result: robot executes a bounded Cartesian offset and returns after completion.
  - [ ] Move to a documented Cartesian pose
    - Recommended file: `03_01_move_flange_pose.rs`
    - Trait: `Motion`, `MoveTo<FlangeSpace>` or `MoveTo<TcpSpace>`
    - Expected result: robot reaches a known flange/TCP pose or reports unsupported Cartesian motion.

- [ ] 04 realtime control
  - [ ] Hold or track joint position
    - Recommended file: `04_00_control_joint_position.rs`
    - Trait: `Control`, `ControlWith<JointPositionControl<N>>`
    - Expected result: bounded control closure terminates by returning `done = true`.
  - [ ] Run bounded joint velocity control
    - Recommended file: `04_01_control_joint_velocity.rs`
    - Trait: `Control`, `ControlWith<JointVelocityControl<N>>`
    - Expected result: velocity command stays bounded and the closure terminates.
  - [ ] Run bounded joint torque control
    - Recommended file: `04_02_control_torque.rs`
    - Trait: `Control`, `ControlWith<TorqueControl<N>>`
    - Expected result: torque command is accepted, limited by the driver, and terminated by the closure.
  - [ ] Run arm-level torque control
    - Recommended file: `04_03_control_arm_torque.rs`
    - Trait: `Control`, `ControlWith<ArmTorqueControl<N>>`
    - Expected result: controller receives full arm state and returns joint torque.
  - [ ] Run Cartesian pose control
    - Recommended file: `04_04_control_cartesian_pose.rs`
    - Trait: `Control`, `ControlWith<CartesianPoseControl<N>>`
    - Expected result: controller receives arm state and returns pose commands.
  - [ ] Run Cartesian velocity control
    - Recommended file: `04_05_control_cartesian_velocity.rs`
    - Trait: `Control`, `ControlWith<CartesianVelocityControl<N>>`
    - Expected result: controller receives arm state and returns bounded spatial velocity.

- [ ] 05 reusable controllers
  - [ ] Use a joint impedance controller
    - Recommended file: `05_00_controller_joint_impedance.rs`
    - Trait: `Control`, `ControlWith<TorqueControl<N>>`
    - Expected result: a controller from `roplat_control` can be passed directly to `control_with`.
  - [ ] Use a Cartesian impedance controller
    - Recommended file: `05_01_controller_cartesian_impedance.rs`
    - Trait: `Control`, `ControlWith<ArmTorqueControl<N>>`, `ForwardKinematics`, `JacobianModel`
    - Expected result: model-backed controller computes torque from Cartesian pose error.
  - [ ] Use a trajectory tracking controller
    - Recommended file: `05_02_controller_joint_traj_impedance.rs`
    - Trait: `Control`, `ControlWith<TorqueControl<N>>`
    - Expected result: controller tracks a sampled target sequence rather than a fixed point.

- [ ] 06 observation and logging
  - [ ] Register control observers
    - Recommended file: `06_00_control_observation.rs`
    - Trait: `ControlObservation`, `Control`
    - Expected result: `before` and `after` observers receive full robot state during a bounded control session.
  - [ ] Communication quality log
    - Recommended file: `06_01_communication_test.rs`
    - Trait: `ControlObservation`, `Control`
    - Expected result: log command-success or timing statistics without blocking the realtime callback.

- [ ] 07 kinematics and dynamics model
  - [ ] Print live frame poses
    - Recommended file: `07_00_model_live_frames.rs`
    - Trait: `Robot`, `ForwardKinematics`
    - Expected result: print forward-kinematics poses for meaningful frames using a live robot state.
  - [ ] Print offline model dynamics
    - Recommended file: `07_01_model_offline_dynamics.rs`
    - Trait: `ForwardKinematics`, `JacobianModel`, `DynamicsModel`
    - Expected result: print pose, Jacobian, mass, Coriolis, and gravity values from model data.

- [ ] 08 end effector and IO
  - [ ] Gripper or end-effector smoke test
    - Recommended file: `08_00_gripper.rs`
    - Trait: driver-native end-effector API
    - Expected result: home, move, read state, grasp when safe, and stop the end effector.
  - [ ] Digital or analog IO
    - Recommended file: `08_01_io.rs`
    - Trait: driver-native IO API
    - Expected result: read or write IO when the driver exposes it.

- [ ] 09 async or external control backend
  - [ ] Native async motion
    - Recommended file: `09_00_async_move_joint.rs`
    - Trait: `Motion`, `MoveTo<JointSpace<N>>`
    - Expected result: `move_to_async` returns a future that only runs when polled.
  - [ ] Native async realtime control
    - Recommended file: `09_01_async_control.rs`
    - Trait: `Control`, `ControlWith<S>`
    - Expected result: driver uses a real async backend instead of wrapping blocking calls.
  - [ ] Active read/write loop
    - Recommended file: `09_02_active_read_write.rs`
    - Trait: driver-native active control API
    - Expected result: expose unsupported status explicitly when the backend has no active read/write primitive.

- [ ] 10 simulator-specific behavior
  - [ ] Spawn robot in a scene
    - Recommended file: `10_00_spawn_robot.rs`
    - Trait: simulator-native API
    - Expected result: simulator creates the robot and exposes a handle/resource for later steps.
  - [ ] Step queued motion and controllers
    - Recommended file: `10_01_step_sim_control.rs`
    - Trait: simulator-native API
    - Expected result: each simulation step distributes state, evaluates registered controllers, and applies commands.

- [ ] 11 safety envelope
  - [ ] Conservative safety configuration
    - Recommended file: `11_00_safety_envelope.rs`
    - Trait: `Arm`, driver-native safety API
    - Expected result: show default scale, collision thresholds, limiting, or saturation behavior.
  - [ ] Real-robot operator warning
    - Recommended file: documented in every moving example
    - Trait: not applicable
    - Expected result: moving examples state their expected physical effect clearly enough for review.

- [ ] 12 unsupported capability map
  - [ ] Unsupported common traits
    - Recommended file: `UNSUPPORTED.md`
    - Trait: not applicable
    - Expected result: list intentionally unsupported behavior traits and the reason.
  - [ ] Vendor-specific capabilities
    - Recommended file: driver-native examples or README section
    - Trait: driver-native API
    - Expected result: point users to hardware-specific functions that are not common traits yet.
