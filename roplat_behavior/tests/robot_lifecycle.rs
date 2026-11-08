//! The defaults that are not no-ops: an emergency stop that cannot latch, and
//! async motion a driver does not have.

use std::time::Duration;

use futures::executor::block_on;
use roplat_behavior::{JointSpace, MoveTo, Robot, RobotException, RobotResult};

/// A robot that overrides only `stop`, and counts or fails its calls.
struct Plain {
    stops: usize,
    stop_fails: bool,
}

impl Robot for Plain {
    type State = ();
    const CONTROL_PERIOD: Duration = Duration::from_millis(10);

    fn version() -> String {
        String::new()
    }

    fn read_state(&mut self) -> RobotResult<()> {
        Ok(())
    }

    fn stop(&mut self) -> RobotResult<()> {
        self.stops += 1;
        if self.stop_fails {
            Err(RobotException::NetworkError("link down".into()))
        } else {
            Ok(())
        }
    }
}

#[test]
fn the_default_emergency_stop_stops_then_says_it_did_not_latch() {
    let mut robot = Plain { stops: 0, stop_fails: false };
    let result = robot.emergency_stop();
    assert!(
        matches!(result, Err(RobotException::NoEmergencyStop)),
        "{result:?}"
    );
    assert_eq!(robot.stops, 1);

    // A stop that fails reports its own error.
    let mut robot = Plain { stops: 0, stop_fails: true };
    let result = robot.emergency_stop();
    assert!(
        matches!(result, Err(RobotException::NetworkError(_))),
        "{result:?}"
    );
}

impl MoveTo<JointSpace<2>> for Plain {
    fn move_to(&mut self, _: [f64; 2]) -> RobotResult<()> {
        Ok(())
    }
}

#[test]
fn async_motion_is_not_supported_unless_the_driver_has_it() {
    let mut robot = Plain { stops: 0, stop_fails: false };
    let result = block_on(MoveTo::<JointSpace<2>>::move_to_async(&mut robot, [0.0; 2]));
    assert!(
        matches!(
            result,
            Err(RobotException::NotSupported { operation: "move_to_async" })
        ),
        "{result:?}"
    );
}
