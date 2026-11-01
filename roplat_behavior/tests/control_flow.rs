mod support;
use roplat_behavior::ControlExit;
use roplat_behavior::{Control, ControlStep};
use std::{ops::ControlFlow, task::Poll};
use support::{TestControl, TestRobot};

#[test]
fn legacy_callback_can_borrow_and_sends_its_final_command() {
    let mut robot = TestRobot::new();
    let mut calls = 0;
    robot
        .control_with::<TestControl, _>(|obs, _| {
            calls += 1;
            (obs + 10, obs == 2)
        })
        .unwrap();
    assert_eq!(calls, 3);
    let trace = robot.trace.lock().unwrap();
    assert_eq!(trace.commands, [10, 11, 12]);
    assert_eq!(trace.exits, 1);
}

#[test]
fn break_sends_no_command_and_never_calls_again() {
    let mut robot = TestRobot::new();
    robot
        .control_with_flow::<TestControl, _>(|obs, _| {
            if obs == 1 {
                ControlFlow::Break(ControlExit::Stopped)
            } else {
                ControlFlow::Continue((42, false))
            }
        })
        .unwrap();
    let trace = robot.trace.lock().unwrap();
    assert_eq!(trace.commands, [42]);
    assert_eq!(trace.callbacks, 2);
    assert_eq!(trace.exits, 1);
}

#[test]
fn async_callback_is_fully_polled_before_next_cycle_and_can_borrow() {
    let mut robot = TestRobot::new();
    let mut completions = 0;
    robot
        .control_with_async::<TestControl, _>(async |obs, _| {
            let mut pending = true;
            futures::future::poll_fn(|cx| {
                if pending {
                    pending = false;
                    cx.waker().wake_by_ref();
                    Poll::Pending
                } else {
                    Poll::Ready(())
                }
            })
            .await;
            completions += 1;
            (obs + 20, obs == 1)
        })
        .unwrap();
    assert_eq!(completions, 2);
    assert_eq!(robot.trace.lock().unwrap().commands, [20, 21]);
}

#[test]
fn async_flow_break_can_terminate_first_cycle() {
    let mut robot = TestRobot::new();
    robot
        .control_with_flow_async::<TestControl, _>(async |_, _| -> ControlStep<u32> {
            ControlFlow::Break(ControlExit::Stopped)
        })
        .unwrap();
    let trace = robot.trace.lock().unwrap();
    assert!(trace.commands.is_empty());
    assert_eq!(trace.callbacks, 1);
    assert_eq!(trace.exits, 1);
}
