//! The 0.10 contract additions: velocity setpoints, car-like vehicles,
//! batteries and segmentation images, with their conformance checks.
#![cfg(feature = "conformance")]

use std::time::Duration;

use roplat_behavior::{
    Battery, BatteryFaults, BatteryReading, BatteryStatus, Camera, CarGeometry, CarLike,
    LabelClass, LabelImage, Multirotor, MultirotorState, PositionSetpoint, Robot, RobotException,
    RobotResult, SteerCommand, VelocitySetpoint, YawCommand,
    conformance::{check_battery, check_car_like, check_segmentation},
    msgs::{CameraInfo, FrameId, Header, Quaternion, Stamp, Stamped, Vector3, clock},
};

fn header(seq: u64) -> Header {
    Header::new(
        Stamp::new(clock::SIMULATION, seq as i64 * 10_000_000),
        FrameId::new(9),
        seq,
    )
}

// ---------------------------------------------------------------------------
// Velocity setpoints

/// A point-mass multirotor that integrates velocity setpoints in the world
/// frame of the contract, as a simulator behind `send_velocity` would.
struct PointMass {
    state: MultirotorState,
    yaw: f64,
    setpoint: Option<(VelocitySetpoint, Duration)>,
    supports_velocity: bool,
}

impl PointMass {
    fn new(supports_velocity: bool) -> Self {
        Self {
            state: MultirotorState {
                device_time: Duration::ZERO,
                position: Vector3::new(0.0, 0.0, 2.0),
                velocity: Some(Vector3::ZERO),
                orientation: Some(Quaternion::IDENTITY),
                angular_velocity: Some(Vector3::ZERO),
                armed: Some(true),
                landed: None,
            },
            yaw: 0.0,
            setpoint: None,
            supports_velocity,
        }
    }

    /// Advances the simulation by `dt`: flies the setpoint while it holds,
    /// then hovers.
    fn step(&mut self, dt: Duration) {
        let now = self.state.device_time + dt;
        let active = self.setpoint.filter(|(s, sent)| now <= *sent + s.duration);
        let (v, rate) = match active {
            Some((s, _)) => {
                let rate = match s.yaw {
                    YawCommand::Hold => 0.0,
                    YawCommand::Rate(rate) => rate,
                    YawCommand::Angle(target) => (target - self.yaw) / dt.as_secs_f64(),
                };
                (s.velocity, rate)
            }
            None => (Vector3::ZERO, 0.0),
        };
        let t = dt.as_secs_f64();
        let p = &mut self.state.position;
        *p = Vector3::new(p.x + v.x * t, p.y + v.y * t, p.z + v.z * t);
        self.yaw += rate * t;
        let half = self.yaw / 2.0;
        self.state.orientation = Some(Quaternion { x: 0.0, y: 0.0, z: half.sin(), w: half.cos() });
        self.state.velocity = Some(v);
        self.state.device_time = now;
    }
}

impl Robot for PointMass {
    type State = MultirotorState;
    const CONTROL_PERIOD: Duration = Duration::from_millis(10);

    fn version() -> String {
        String::new()
    }

    fn read_state(&mut self) -> RobotResult<MultirotorState> {
        Ok(self.state)
    }
}

impl Multirotor for PointMass {
    fn read_flight_state(&mut self) -> RobotResult<Stamped<MultirotorState>> {
        Ok(Stamped::new(header(1), self.state))
    }

    fn take_off(&mut self, _: f64) -> RobotResult<()> {
        Ok(())
    }

    fn send_setpoint(&mut self, _: PositionSetpoint) -> RobotResult<()> {
        Ok(())
    }

    fn go_to(&mut self, _: PositionSetpoint) -> RobotResult<Stamped<MultirotorState>> {
        self.read_flight_state()
    }

    fn land(&mut self) -> RobotResult<()> {
        Ok(())
    }

    fn send_velocity(&mut self, setpoint: VelocitySetpoint) -> RobotResult<()> {
        if !self.supports_velocity {
            return Err(RobotException::NotSupported { operation: "send_velocity" });
        }
        setpoint.validate()?;
        self.setpoint = Some((setpoint, self.state.device_time));
        Ok(())
    }
}

/// A driver that does not override `send_velocity`.
struct PositionOnly;

impl Robot for PositionOnly {
    type State = ();
    const CONTROL_PERIOD: Duration = Duration::from_millis(10);

    fn version() -> String {
        String::new()
    }

    fn read_state(&mut self) -> RobotResult<()> {
        Ok(())
    }
}

impl Multirotor for PositionOnly {
    fn read_flight_state(&mut self) -> RobotResult<Stamped<MultirotorState>> {
        Err(RobotException::NotSupported { operation: "read_flight_state" })
    }

    fn take_off(&mut self, _: f64) -> RobotResult<()> {
        Ok(())
    }

    fn send_setpoint(&mut self, _: PositionSetpoint) -> RobotResult<()> {
        Ok(())
    }

    fn go_to(&mut self, _: PositionSetpoint) -> RobotResult<Stamped<MultirotorState>> {
        self.read_flight_state()
    }

    fn land(&mut self) -> RobotResult<()> {
        Ok(())
    }
}

fn east_south(duration: Duration) -> VelocitySetpoint {
    VelocitySetpoint {
        velocity: Vector3::new(1.5, -1.0, 0.0),
        yaw: YawCommand::Hold,
        duration,
    }
}

#[test]
fn a_velocity_setpoint_flies_in_the_world_frame_for_its_duration() {
    let mut vehicle = PointMass::new(true);
    vehicle.send_velocity(east_south(Duration::from_secs(3))).unwrap();
    let dt = Duration::from_millis(10);
    for _ in 0..500 {
        vehicle.step(dt);
    }
    // 3 s east at 1.5 m/s and south at 1 m/s, then a hover: the world
    // frame is east-north-up, so south is -y.
    let p = vehicle.state.position;
    assert!((p.x - 4.5).abs() < 0.02 && (p.y + 3.0).abs() < 0.02, "{p:?}");
    assert_eq!(vehicle.state.velocity, Some(Vector3::ZERO));

    // A positive yaw rate turns left (counter-clockwise from above).
    let mut turning = PointMass::new(true);
    turning
        .send_velocity(VelocitySetpoint {
            yaw: YawCommand::Rate(0.5),
            ..east_south(Duration::from_secs(1))
        })
        .unwrap();
    for _ in 0..100 {
        turning.step(dt);
    }
    let yaw = turning.state.yaw().unwrap();
    assert!((yaw - 0.5).abs() < 1e-9, "yaw {yaw}");
}

#[test]
fn invalid_velocity_setpoints_are_refused_before_they_are_sent() {
    let good = east_south(Duration::from_secs(1));
    good.validate().unwrap();
    for bad in [
        VelocitySetpoint { velocity: Vector3::new(f64::NAN, 0.0, 0.0), ..good },
        VelocitySetpoint { velocity: Vector3::new(0.0, 0.0, f64::INFINITY), ..good },
        VelocitySetpoint { yaw: YawCommand::Angle(f64::NAN), ..good },
        VelocitySetpoint { yaw: YawCommand::Rate(f64::NEG_INFINITY), ..good },
        VelocitySetpoint { duration: Duration::ZERO, ..good },
    ] {
        assert!(
            matches!(bad.validate(), Err(RobotException::InvalidInstruction(_))),
            "{bad:?}"
        );
        let mut vehicle = PointMass::new(true);
        assert!(vehicle.send_velocity(bad).is_err());
        assert!(vehicle.setpoint.is_none(), "{bad:?} reached the vehicle");
    }
}

#[test]
fn a_driver_without_velocities_answers_not_supported() {
    assert!(matches!(
        PositionOnly.send_velocity(east_south(Duration::from_secs(1))),
        Err(RobotException::NotSupported { operation: "send_velocity" })
    ));
    assert!(matches!(
        PointMass::new(false).send_velocity(east_south(Duration::from_secs(1))),
        Err(RobotException::NotSupported { .. })
    ));
}

// ---------------------------------------------------------------------------
// Car-like vehicles

/// A kinematic bicycle: the reference point is the rear axle's midpoint.
struct Bicycle {
    geometry: CarGeometry,
    command: SteerCommand,
    /// The steering angle the wheels have reached.
    steering: f64,
    pose: [f64; 3],
    /// Faults to inject.
    accepts_nan: bool,
}

impl Bicycle {
    fn new() -> Self {
        Self {
            geometry: CarGeometry {
                wheelbase: 2.5,
                max_steering_angle: 0.6,
                max_steering_rate: Some(0.5),
            },
            command: SteerCommand::default(),
            steering: 0.0,
            pose: [0.0; 3],
            accepts_nan: false,
        }
    }

    fn step(&mut self, dt: f64) {
        let rate = self.geometry.max_steering_rate.unwrap_or(f64::INFINITY) * dt;
        let target = self.command.steering_angle;
        self.steering += (target - self.steering).clamp(-rate, rate);
        // The exact arc of the step, the steering held at its new angle.
        let [x, y, yaw] = &mut self.pose;
        let (v, k) = (self.command.speed, self.geometry.curvature(self.steering));
        let turn = v * k * dt;
        if k.abs() < 1e-12 {
            *x += v * yaw.cos() * dt;
            *y += v * yaw.sin() * dt;
        } else {
            *x += ((*yaw + turn).sin() - yaw.sin()) / k;
            *y -= ((*yaw + turn).cos() - yaw.cos()) / k;
        }
        *yaw += turn;
    }
}

impl Robot for Bicycle {
    type State = [f64; 3];
    const CONTROL_PERIOD: Duration = Duration::from_millis(10);

    fn version() -> String {
        String::new()
    }

    fn read_state(&mut self) -> RobotResult<[f64; 3]> {
        Ok(self.pose)
    }
}

impl CarLike for Bicycle {
    fn geometry(&self) -> CarGeometry {
        self.geometry
    }

    fn drive(&mut self, command: SteerCommand) -> RobotResult<()> {
        if !self.accepts_nan {
            command.validate()?;
        }
        self.command = self.geometry.clamp(command);
        Ok(())
    }
}

#[test]
fn a_held_steering_angle_drives_the_arc_its_curvature_gives() {
    let mut car = Bicycle::new();
    check_car_like(&mut car).unwrap();
    // Steer left at 0.3 rad and drive a quarter circle once the wheels
    // have turned: the reference point turns counter-clockwise on a circle
    // of radius wheelbase / tan(0.3).
    car.drive(SteerCommand { speed: 2.0, steering_angle: 0.3 }).unwrap();
    for _ in 0..100 {
        car.step(0.01);
    }
    assert!((car.steering - 0.3).abs() < 1e-12);
    let radius = 1.0 / car.geometry.curvature(0.3);
    assert!((radius - 2.5 / 0.3f64.tan()).abs() < 1e-12);
    let [x0, y0, yaw0] = car.pose;
    let centre = [x0 - radius * yaw0.sin(), y0 + radius * yaw0.cos()];
    let mut farthest: f64 = 0.0;
    for _ in 0..2000 {
        car.step(0.001);
        let d = (car.pose[0] - centre[0]).hypot(car.pose[1] - centre[1]);
        farthest = farthest.max((d - radius).abs());
    }
    assert!(farthest < 1e-9, "off the circle by {farthest} m");
    assert!(car.pose[2] > yaw0, "a positive angle turns left");
    // And the inverse map gives the angle back.
    let back = car.geometry.steering_angle(car.geometry.curvature(0.3));
    assert!((back - 0.3).abs() < 1e-12);
}

#[test]
fn steering_beyond_the_limit_saturates() {
    let mut car = Bicycle::new();
    car.drive(SteerCommand { speed: 1.0, steering_angle: -2.0 }).unwrap();
    assert_eq!(car.command.steering_angle, -0.6);
    car.drive(SteerCommand { speed: -1.0, steering_angle: 0.7 }).unwrap();
    assert_eq!(car.command, SteerCommand { speed: -1.0, steering_angle: 0.6 });
}

#[test]
fn the_car_check_names_broken_vehicles() {
    let check = |car: &mut Bicycle| check_car_like(car).map_err(|e| e.check);
    let mut bad = Bicycle::new();
    bad.geometry.max_steering_angle = std::f64::consts::FRAC_PI_2;
    assert_eq!(check(&mut bad), Err("car_like"));
    let mut bad = Bicycle::new();
    bad.geometry.wheelbase = 0.0;
    assert_eq!(check(&mut bad), Err("car_like"));
    let mut bad = Bicycle::new();
    bad.geometry.max_steering_rate = Some(-1.0);
    assert_eq!(check(&mut bad), Err("car_like"));
    let mut bad = Bicycle::new();
    bad.accepts_nan = true;
    assert_eq!(check(&mut bad), Err("car_like"));
    // A vehicle whose steering rate is unknown passes.
    let mut unknown = Bicycle::new();
    unknown.geometry.max_steering_rate = None;
    check(&mut unknown).unwrap();
}

// ---------------------------------------------------------------------------
// Batteries

#[derive(Clone, Copy, Debug)]
enum BatteryFault {
    /// Discharge current reported negative, as some packs send it.
    ChargingSign,
    /// State of charge left in percent.
    Percent,
    ChargeAboveCapacity,
}

/// A pack discharging into a constant load.
struct MockPack {
    seq: u64,
    fault: Option<BatteryFault>,
}

impl Battery for MockPack {
    fn read_battery(&mut self) -> RobotResult<Stamped<BatteryReading>> {
        self.seq += 1;
        let charge = 40.0 - 0.1 * self.seq as f64;
        let mut reading = BatteryReading {
            voltage: Some(48.2),
            current: Some(12.0),
            charge: Some(charge),
            capacity: Some(50.0),
            state_of_charge: Some(charge / 50.0),
            temperature: Some(31.5),
            status: Some(BatteryStatus::Discharging),
            faults: Some(BatteryFaults::NONE),
        };
        match self.fault {
            Some(BatteryFault::ChargingSign) => reading.current = Some(-12.0),
            Some(BatteryFault::Percent) => reading.state_of_charge = Some(100.0 * charge / 50.0),
            Some(BatteryFault::ChargeAboveCapacity) => reading.capacity = Some(charge / 2.0),
            None => {}
        }
        Ok(Stamped::new(header(self.seq), reading))
    }
}

#[test]
fn battery_checks() {
    let check = |fault| check_battery(&mut MockPack { seq: 0, fault }, || {}, 3, 0.5);
    check(None).unwrap();
    for fault in [
        BatteryFault::ChargingSign,
        BatteryFault::Percent,
        BatteryFault::ChargeAboveCapacity,
    ] {
        let e = check(Some(fault)).unwrap_err();
        assert_eq!(e.check, "battery", "{fault:?}");
    }
    // A reading with nothing reported passes: missing is `None`.
    struct Silent(u64);
    impl Battery for Silent {
        fn read_battery(&mut self) -> RobotResult<Stamped<BatteryReading>> {
            self.0 += 1;
            Ok(Stamped::new(header(self.0), BatteryReading::default()))
        }
    }
    check_battery(&mut Silent(0), || {}, 2, 0.0).unwrap();
}

#[test]
fn battery_faults_are_a_set() {
    let mut faults = BatteryFaults::NONE;
    assert!(faults.is_empty());
    faults |= BatteryFaults::UNDER_VOLTAGE;
    faults |= BatteryFaults::CELL_IMBALANCE;
    assert!(faults.contains(BatteryFaults::UNDER_VOLTAGE));
    assert!(faults.contains(BatteryFaults::UNDER_VOLTAGE | BatteryFaults::CELL_IMBALANCE));
    assert!(!faults.contains(BatteryFaults::UNDER_VOLTAGE | BatteryFaults::SHORT_CIRCUIT));
    assert!(BatteryFaults::ALL.contains(faults));
    let named = [
        BatteryFaults::OVER_VOLTAGE,
        BatteryFaults::UNDER_VOLTAGE,
        BatteryFaults::OVER_CURRENT,
        BatteryFaults::OVER_TEMPERATURE,
        BatteryFaults::UNDER_TEMPERATURE,
        BatteryFaults::CELL_IMBALANCE,
        BatteryFaults::SHORT_CIRCUIT,
        BatteryFaults::OTHER,
    ];
    // Each fault is one bit of its own, and ALL is exactly their union.
    let mut union = BatteryFaults::NONE;
    for (k, f) in named.iter().enumerate() {
        assert_eq!(f.bits().count_ones(), 1, "{f:?}");
        assert!(named[..k].iter().all(|g| !g.contains(*f)), "{f:?} repeats a bit");
        union |= *f;
    }
    assert_eq!(union, BatteryFaults::ALL);
}

// ---------------------------------------------------------------------------
// Segmentation

#[derive(Clone, Copy, Debug, PartialEq)]
enum LabelFault {
    Missing,
    WrongSize,
    UnlistedId,
    DuplicateClass,
    Unnamed,
    ShortIds,
}

struct Frame {
    labels: Option<LabelImage>,
}

/// A 4 x 3 camera that sees a box in its left half and nothing on the
/// right.
struct LabellingCamera {
    seq: u64,
    fault: Option<LabelFault>,
}

impl Camera for LabellingCamera {
    type Frame = Frame;

    fn camera_info(&self) -> CameraInfo {
        CameraInfo {
            width: 4,
            height: 3,
            k: [2.0, 0.0, 1.5, 0.0, 2.0, 1.0, 0.0, 0.0, 1.0],
            ..Default::default()
        }
    }

    fn capture(&mut self) -> RobotResult<Stamped<Frame>> {
        self.seq += 1;
        let mut image = LabelImage {
            width: 4,
            height: 3,
            ids: (0..12)
                .map(|i| if i % 4 < 2 { 3 } else { LabelImage::UNLABELLED })
                .collect(),
            classes: vec![LabelClass { id: 3, name: "box".into() }],
        };
        match self.fault {
            Some(LabelFault::Missing) => {
                return Ok(Stamped::new(header(self.seq), Frame { labels: None }));
            }
            Some(LabelFault::WrongSize) => image.width = 3,
            Some(LabelFault::UnlistedId) => image.ids[11] = 4,
            Some(LabelFault::DuplicateClass) => {
                image.classes.push(LabelClass { id: 3, name: "crate".into() });
            }
            Some(LabelFault::Unnamed) => image.classes[0].name.clear(),
            Some(LabelFault::ShortIds) => {
                image.ids.pop();
            }
            None => {}
        }
        Ok(Stamped::new(header(self.seq), Frame { labels: Some(image) }))
    }
}

#[test]
fn segmentation_checks() {
    let check = |fault| {
        check_segmentation(
            &mut LabellingCamera { seq: 0, fault },
            || {},
            2,
            |frame: &Frame| frame.labels.as_ref(),
        )
        .map_err(|e| e.check)
    };
    check(None).unwrap();
    for fault in [
        LabelFault::Missing,
        LabelFault::WrongSize,
        LabelFault::UnlistedId,
        LabelFault::DuplicateClass,
        LabelFault::Unnamed,
        LabelFault::ShortIds,
    ] {
        assert_eq!(check(Some(fault)), Err("segmentation"), "{fault:?}");
    }
}

#[test]
fn label_images_are_read_row_by_row() {
    let image = sample_labels();
    assert_eq!(image.id_at(1, 0), Some(3));
    assert_eq!(image.id_at(2, 1), Some(LabelImage::UNLABELLED));
    assert_eq!(image.id_at(0, 2), Some(5));
    assert_eq!(image.class_at(0, 2), Some("cup"));
    assert_eq!(image.class_at(3, 1), None);
    assert_eq!(image.id_at(4, 0), None);
    assert_eq!(image.id_at(0, 3), None);
    assert_eq!(image.class_of(LabelImage::UNLABELLED), None);
}

/// Row 0: box box - -, row 1: box - - -, row 2: cup cup cup -.
fn sample_labels() -> LabelImage {
    let u = LabelImage::UNLABELLED;
    LabelImage {
        width: 4,
        height: 3,
        ids: vec![3, 3, u, u, 3, u, u, u, 5, 5, 5, u],
        classes: vec![
            LabelClass { id: 3, name: "box".into() },
            LabelClass { id: 5, name: "cup".into() },
        ],
    }
}
