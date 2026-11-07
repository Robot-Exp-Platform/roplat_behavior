#![cfg(feature = "roplat")]
//! `JointStateNode` in an actual System: the buffer is a borrowed resource,
//! and a reader borrows it after the writer through a graph dependency.

mod support;

use std::sync::Arc;
use std::time::Duration;

use roplat::rhythm::IntoRhythm;
use roplat::{Lifecycle, Node, RoplatError, RoplatResult};
use roplat_frames::roplat::JointStateNode;
use roplat_frames::{FrameBuffer, FrameError, History, JointGroup};
use roplat_msgs::{Clock, JointSample, JointState, ManualClock, Pose, Stamp, StateView, clock};
use support::*;

const PERIOD: Duration = Duration::from_millis(1);

/// Where the hand is at the instant the joint state was stamped.
struct HandPose;

impl Lifecycle for HandPose {
    type Error = RoplatError;
}

impl Node for HandPose {
    type Resources = (FrameBuffer,);
    type Input = (Stamp, JointState<2>);
    type Output = Result<Pose, FrameError>;

    async fn process<'call>(
        &'call mut self,
        (at, _): Self::Input,
        (frames,): (&'call mut FrameBuffer,),
    ) -> Self::Output {
        frames.lookup(BASE, HAND, at)
    }
}

/// A simulator stepping its clock once per cycle; the step stamps the state.
#[roplat::system]
async fn follow(
    frames: &mut FrameBuffer,
    arm: JointGroup<2>,
    clock: Arc<ManualClock>,
    path: Vec<JointState<2>>,
) -> RoplatResult<Vec<Result<Pose, FrameError>>> {
    let mut cycles = path.into_iter().into_collect_rhythm();
    let mut step = roplat::node::map(move |state: JointState<2>| (clock.advance(PERIOD), state));
    let mut record = JointStateNode::<2>::new(arm);
    let mut hand = HandPose;
    let mut report = roplat::node::map(|poses: Vec<Result<Pose, FrameError>>| poses);

    cycles
        >> |state| {
            state >> step;
            &mut frames >> record.resources.0;
            step >> record;
            record.output?;
            &mut frames >> hand.resources.0;
            step >> hand;
            record >> hand.trigger;
            hand
        };
    cycles >> report;
    Ok(report.output)
}

fn measured(q: [f64; 2]) -> JointState<2> {
    StateView::from_meas(JointSample::EMPTY.with_q(q))
}

fn sim_buffer() -> (FrameBuffer, JointGroup<2>) {
    let frames =
        FrameBuffer::new(cell(History::new(64), History::new(1)), clock::SIMULATION).unwrap();
    let arm = frames.tree().group::<2>("arm").unwrap();
    (frames, arm)
}

#[tokio::test]
async fn records_each_cycle_and_lends_the_buffer_back() {
    let (mut frames, arm) = sim_buffer();
    let sim = Arc::new(ManualClock::simulation());
    let path: Vec<[f64; 2]> = (0..5).map(|i| [0.1 * i as f64, -0.05 * i as f64]).collect();

    let poses = follow(
        &mut frames,
        arm,
        sim.clone(),
        path.iter().copied().map(measured).collect(),
    )
    .await
    .unwrap();
    assert_eq!(poses.len(), path.len());
    for (pose, q) in poses.into_iter().zip(&path) {
        assert_pose(pose.unwrap(), hand_in_base(*q));
    }

    // The buffer stays with its owner, holding the whole run on the sim clock.
    let between = Stamp::new(clock::SIMULATION, 2_500_000);
    assert_eq!(sim.now(), Stamp::new(clock::SIMULATION, 5_000_000));
    assert_pose(
        frames.lookup(BASE, HAND, between).unwrap(),
        hand_in_base([0.15, -0.075]),
    );
}

#[tokio::test]
async fn a_state_without_positions_fails_when_raised() {
    let (mut frames, arm) = sim_buffer();
    let sim = Arc::new(ManualClock::simulation());
    let path = vec![measured([0.2, 0.1]), JointState::default()];

    let error = follow(&mut frames, arm, sim, path).await.unwrap_err();
    assert!(error.to_string().contains("no positions"), "{error}");
    // The first cycle was recorded before the second one failed.
    let first = Stamp::new(clock::SIMULATION, 1_000_000);
    assert_pose(
        frames.lookup(BASE, HAND, first).unwrap(),
        hand_in_base([0.2, 0.1]),
    );
}

#[tokio::test]
async fn node_checks_positions_then_delegates_clock_and_group_validation() {
    let (mut frames, _) = sim_buffer();
    let (_, foreign) = sim_buffer();
    let mut node = JointStateNode::<2>::new(foreign);
    assert_eq!(
        node.process((ms(0.), JointState::default()), (&mut frames,))
            .await,
        Err(FrameError::MissingPositions)
    );
    assert_eq!(
        node.process((ms(0.), measured([1., 0.])), (&mut frames,))
            .await,
        Err(FrameError::ClockMismatch { expected: clock::SIMULATION, got: clock::MONOTONIC })
    );
    let at = Stamp::new(clock::SIMULATION, 0);
    assert_eq!(
        node.process((at, measured([1., 0.])), (&mut frames,)).await,
        Err(FrameError::ForeignGroup)
    );
    assert_eq!(frames.lookup(BASE, HAND, at), Err(FrameError::NoData(FORE)));
}

#[tokio::test]
async fn accepts_positions_samples_and_states() {
    let (mut frames, arm) = sim_buffer();
    let at = |ms: i64| Stamp::new(clock::SIMULATION, ms * 1_000_000);

    let mut array = JointStateNode::<2, [f64; 2]>::new(arm);
    array
        .process((at(0), [0.1, 0.]), (&mut frames,))
        .await
        .unwrap();
    let mut sample = JointStateNode::<2, JointSample<2>>::new(arm);
    let q = JointSample::EMPTY.with_q([0.2, 0.]);
    sample.process((at(1), q), (&mut frames,)).await.unwrap();
    let mut state = JointStateNode::<2>::new(arm);
    state
        .process((at(2), measured([0.3, 0.])), (&mut frames,))
        .await
        .unwrap();

    for (ms, q0) in [(0, 0.1), (1, 0.2), (2, 0.3)] {
        assert_pose(
            frames.lookup(BASE, HAND, at(ms)).unwrap(),
            hand_in_base([q0, 0.]),
        );
    }
    assert_eq!(
        state
            .process((at(3), JointState::default()), (&mut frames,))
            .await,
        Err(FrameError::MissingPositions)
    );
}
