//! Signal filters for control loops. Each filter is a struct: its parameters
//! are validated once, in `new`, and its state lives inside it. A caller that
//! keeps the previous output itself, such as a driver that filters against the
//! command the robot reports back, uses `blend`; a caller that wants the filter
//! to remember uses `step`.

use std::f64::consts::PI;
use std::time::Duration;

use nalgebra as na;

/// Why a filter could not be constructed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FilterError {
    /// The cutoff frequency must be finite and positive.
    Cutoff { cutoff_hz: f64 },
    /// The sampling period must be positive.
    Period { period: Duration },
}

impl std::fmt::Display for FilterError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Cutoff { cutoff_hz } => {
                write!(
                    f,
                    "the cutoff frequency must be finite and positive, got {cutoff_hz} Hz"
                )
            }
            Self::Period { period } => {
                write!(f, "the sampling period must be positive, got {period:?}")
            }
        }
    }
}

impl std::error::Error for FilterError {}

/// The gain of a first-order low-pass filter sampled every `period` with the
/// cutoff `cutoff_hz`: `g = T / (T + 1 / (2 π f_c))`.
fn first_order_gain(cutoff_hz: f64, period: Duration) -> Result<f64, FilterError> {
    if !(cutoff_hz.is_finite() && cutoff_hz > 0.0) {
        return Err(FilterError::Cutoff { cutoff_hz });
    }
    if period.is_zero() {
        return Err(FilterError::Period { period });
    }
    let sample_time = period.as_secs_f64();
    Ok(sample_time / (sample_time + 1.0 / (2.0 * PI * cutoff_hz)))
}

/// A first-order low-pass filter over `N` channels:
/// `y = g x + (1 - g) y_last`.
#[derive(Clone, Debug)]
pub struct LowPass<const N: usize> {
    gain: f64,
    last: Option<[f64; N]>,
}

impl<const N: usize> LowPass<N> {
    /// A filter with the cutoff `cutoff_hz`, sampled every `period`.
    pub fn new(cutoff_hz: f64, period: Duration) -> Result<Self, FilterError> {
        Ok(Self { gain: first_order_gain(cutoff_hz, period)?, last: None })
    }

    /// The weight of the new sample in the output.
    pub fn gain(&self) -> f64 {
        self.gain
    }

    /// The filtered value of `x` after the previous output `last`, without
    /// touching the state held inside.
    pub fn blend(&self, last: &[f64; N], x: &[f64; N]) -> [f64; N] {
        std::array::from_fn(|i| self.gain * x[i] + (1.0 - self.gain) * last[i])
    }

    /// Filters `x` against the previous output held inside. The first sample
    /// passes through and seeds the state.
    pub fn step(&mut self, x: [f64; N]) -> [f64; N] {
        let y = match &self.last {
            Some(last) => self.blend(last, &x),
            None => x,
        };
        self.last = Some(y);
        y
    }

    /// Forgets the held output; the next sample passes through.
    pub fn reset(&mut self) {
        self.last = None;
    }
}

/// A first-order low-pass filter of a rigid transform: the translation is
/// blended linearly and the rotation by spherical linear interpolation, both
/// with the same gain.
#[derive(Clone, Debug)]
pub struct CartesianLowPass {
    gain: f64,
    last: Option<na::Isometry3<f64>>,
}

impl CartesianLowPass {
    /// A filter with the cutoff `cutoff_hz`, sampled every `period`.
    pub fn new(cutoff_hz: f64, period: Duration) -> Result<Self, FilterError> {
        Ok(Self { gain: first_order_gain(cutoff_hz, period)?, last: None })
    }

    /// The weight of the new sample in the output.
    pub fn gain(&self) -> f64 {
        self.gain
    }

    /// The filtered transform of `x` after the previous output `last`, without
    /// touching the state held inside.
    pub fn blend(&self, last: &na::Isometry3<f64>, x: &na::Isometry3<f64>) -> na::Isometry3<f64> {
        let translation =
            self.gain * x.translation.vector + (1.0 - self.gain) * last.translation.vector;
        let rotation = last.rotation.slerp(&x.rotation, self.gain);
        na::Isometry3::from_parts(na::Translation3::from(translation), rotation)
    }

    /// Filters `x` against the previous output held inside. The first sample
    /// passes through and seeds the state.
    pub fn step(&mut self, x: na::Isometry3<f64>) -> na::Isometry3<f64> {
        let y = match &self.last {
            Some(last) => self.blend(last, &x),
            None => x,
        };
        self.last = Some(y);
        y
    }

    /// Forgets the held output; the next sample passes through.
    pub fn reset(&mut self) {
        self.last = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn invalid_parameters_are_refused() {
        assert_eq!(
            LowPass::<1>::new(0.0, Duration::from_millis(1)).unwrap_err(),
            FilterError::Cutoff { cutoff_hz: 0.0 }
        );
        assert!(LowPass::<1>::new(f64::INFINITY, Duration::from_millis(1)).is_err());
        assert_eq!(
            LowPass::<1>::new(100.0, Duration::ZERO).unwrap_err(),
            FilterError::Period { period: Duration::ZERO }
        );
    }

    #[test]
    fn the_first_sample_passes_through_and_later_ones_are_blended() {
        let mut filter = LowPass::<2>::new(100.0, Duration::from_millis(1)).unwrap();
        assert_eq!(filter.step([1.0, -1.0]), [1.0, -1.0]);
        let y = filter.step([2.0, 0.0]);
        let g = filter.gain();
        assert!((y[0] - (g * 2.0 + (1.0 - g) * 1.0)).abs() < 1e-12);
        assert!((y[1] + (1.0 - g)).abs() < 1e-12);
        assert_eq!(filter.blend(&[0.0, 0.0], &[1.0, 1.0]), [g, g]);
        filter.reset();
        assert_eq!(filter.step([5.0, 5.0]), [5.0, 5.0]);
    }

    #[test]
    fn a_transform_blends_its_translation_and_turns_its_rotation() {
        let filter = CartesianLowPass::new(100.0, Duration::from_millis(1)).unwrap();
        let last = na::Isometry3::identity();
        let x = na::Isometry3::from_parts(
            na::Translation3::new(1.0, 0.0, 0.0),
            na::UnitQuaternion::from_euler_angles(0.0, 0.0, 1.0),
        );
        let y = filter.blend(&last, &x);
        let g = filter.gain();
        assert!((y.translation.vector.x - g).abs() < 1e-12);
        assert!((y.rotation.euler_angles().2 - g).abs() < 1e-9);
    }
}
