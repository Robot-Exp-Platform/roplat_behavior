//! Car-like vehicles: driven by a speed and a steering angle.

use crate::{Robot, RobotException, RobotResult};

/// One command to a [`CarLike`] vehicle.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SteerCommand {
    /// m/s of the rear axle's midpoint along the vehicle's heading;
    /// negative drives backwards.
    pub speed: f64,
    /// Radians, the angle of the bicycle model's front wheel: the one
    /// wheel at the front axle's midpoint that turns the vehicle about the
    /// same point as its real wheels do. Positive steers left
    /// (counter-clockwise seen from above), zero straight ahead.
    pub steering_angle: f64,
}

impl SteerCommand {
    /// `Ok` for a command a driver may send:
    /// [`InvalidInstruction`](RobotException::InvalidInstruction) for a
    /// speed or steering angle that is not finite. A driver calls it before
    /// sending, so that such a command never reaches the vehicle.
    pub fn validate(&self) -> RobotResult<()> {
        if self.speed.is_finite() && self.steering_angle.is_finite() {
            Ok(())
        } else {
            Err(RobotException::InvalidInstruction(format!(
                "steer command {self:?} is not finite"
            )))
        }
    }
}

/// The geometry a [`CarLike`] vehicle steers with.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CarGeometry {
    /// Metres from the rear axle to the front axle.
    pub wheelbase: f64,
    /// Radians, the largest steering angle either way, in `(0, π/2)`.
    pub max_steering_angle: f64,
    /// rad/s, how fast the steering angle can change; `None` when the
    /// driver does not know it.
    pub max_steering_rate: Option<f64>,
}

impl CarGeometry {
    /// The path's curvature, 1/m and positive to the left, while the
    /// vehicle holds `steering_angle`: `tan(δ) / wheelbase`. Its inverse is
    /// the turning radius of the rear axle's midpoint.
    pub fn curvature(&self, steering_angle: f64) -> f64 {
        steering_angle.tan() / self.wheelbase
    }

    /// The steering angle that holds the path's curvature at `curvature`:
    /// `atan(curvature × wheelbase)`, which may lie beyond
    /// [`max_steering_angle`](Self::max_steering_angle).
    pub fn steering_angle(&self, curvature: f64) -> f64 {
        (curvature * self.wheelbase).atan()
    }

    /// `command` with its steering angle within `±max_steering_angle`, the
    /// command a [`CarLike`] vehicle carries out for it.
    pub fn clamp(&self, command: SteerCommand) -> SteerCommand {
        SteerCommand {
            steering_angle: command
                .steering_angle
                .clamp(-self.max_steering_angle, self.max_steering_angle),
            ..command
        }
    }
}

/// A vehicle that steers like a car: its front wheels turn and its rear
/// wheels do not, so it follows arcs whose curvature the steering angle
/// sets, and cannot turn on the spot. Cars, Ackermann robots and
/// simulated vehicles (CARLA, AirSim, Chrono) implement it; a
/// differential or omnidirectional base is a
/// [`MobileBase`](crate::MobileBase) instead.
///
/// # Model and frames
///
/// The commands follow the kinematic bicycle model: the vehicle's
/// reference point is the rear axle's midpoint, the speed is that point's
/// along the heading, and the steering angle is the angle of one front
/// wheel at the front axle's midpoint. Holding a steering angle `δ` drives
/// the reference point round a circle of curvature `tan(δ) / wheelbase`
/// ([`CarGeometry::curvature`]). A driver whose vehicle steers its two
/// front wheels by different angles (Ackermann steering) gives the angle of
/// the virtual middle wheel, and a driver whose device takes a normalised
/// steering input converts it, both documented with the mapping it uses.
/// Left is positive, as yaw counter-clockwise about up is in every
/// contract here.
///
/// # Driving
///
/// [`drive`](Self::drive) sends one command and returns. The vehicle keeps
/// it until the next one: it tracks the speed with its own controller and
/// turns its wheels towards the steering angle no faster than
/// [`CarGeometry::max_steering_rate`]. A device that stops the vehicle when
/// commands stop arriving, as most do, has its driver document the
/// timeout; a caller must send faster than that. A steering angle beyond
/// [`CarGeometry::max_steering_angle`] steers as far as the vehicle can,
/// the command [`CarGeometry::clamp`] gives; the driver does not refuse it,
/// since a path follower saturates its command in tight turns as a matter
/// of course. A command of zero speed brakes to a standstill and holds
/// the vehicle there.
pub trait CarLike: Robot {
    /// The geometry the vehicle steers with; it does not change while the
    /// driver is open.
    fn geometry(&self) -> CarGeometry;

    /// Sends one command and returns. A command that
    /// [`SteerCommand::validate`] refuses is refused with that error and
    /// never sent.
    fn drive(&mut self, command: SteerCommand) -> RobotResult<()>;
}
