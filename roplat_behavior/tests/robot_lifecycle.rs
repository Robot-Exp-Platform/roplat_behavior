//! The lifecycle defaults that are not no-ops.

use std::time::Duration;

use roplat_behavior::{Robot, RobotException, RobotResult};

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
