use std::path::Path;

#[cfg(any(target_os = "linux", target_os = "windows"))]
use roplat_behavior::RobotException;
use roplat_behavior::RobotResult;

pub trait RealtimeBehavior<C, H> {
    fn enter_realtime(&mut self, realtime_config: C) -> RobotResult<H>;
    fn exit_realtime(&mut self) -> RobotResult<()>;
    fn quality_of_service(&self) -> f64;
}

pub fn is_hardware_realtime() -> bool {
    if cfg!(target_os = "linux") {
        Path::new("/sys/kernel/realtime").exists()
    } else if cfg!(target_os = "windows") {
        true
    } else {
        println!("Unknown OS, assuming realtime kernel.");
        true
    }
}

/// Sets the current thread to the highest possible scheduler priority.
///
/// # Errors
/// * RealtimeException if realtime priority cannot be set for the current thread.
///
/// ## Linux
/// If the method returns an Error please check your /etc/security/limits.conf file
/// There should be a line like this:
/// ```text
///marco            -       rtprio          99
/// ```
///
/// ##
pub fn set_realtime_priority() -> RobotResult<()> {
    #[cfg(target_os = "windows")]
    {
        use winapi::um::{
            processthreadsapi::{
                GetCurrentProcess, GetCurrentThread, SetPriorityClass, SetThreadPriority,
            },
            winbase::{REALTIME_PRIORITY_CLASS, THREAD_PRIORITY_TIME_CRITICAL},
        };

        unsafe {
            let process_handle = GetCurrentProcess();
            if SetPriorityClass(process_handle, REALTIME_PRIORITY_CLASS) == 0 {
                return Err(RobotException::RealtimeException(
                    "unable to set realtime priority, try run as an adminstrator!".to_string(),
                ));
            }

            // Raise the current thread to the highest priority.
            let thread_handle = GetCurrentThread();
            if SetThreadPriority(thread_handle, THREAD_PRIORITY_TIME_CRITICAL as i32) == 0 {
                return Err(RobotException::RealtimeException(
                    "unable to set max thread priority".to_string(),
                ));
            }
        }
    }
    #[cfg(target_os = "linux")]
    unsafe {
        let max_priority = libc::sched_get_priority_max(libc::SCHED_FIFO);
        if max_priority == -1 {
            return Err(RobotException::RealtimeException(
                "unable to get maximum possible thread priority".to_string(),
            ));
        }
        let thread_param = libc::sched_param {
            // In the original libfranka the priority is set to the maximum priority (99 in this
            // case). However, we will set the priority 1 lower as
            // https://rt.wiki.kernel.org/index.php/HOWTO:_Build_an_RT-application recommends
            sched_priority: max_priority - 1,
        };
        if libc::pthread_setschedparam(libc::pthread_self(), libc::SCHED_FIFO, &thread_param) != 0 {
            return Err(RobotException::RealtimeException(
                "unable to set realtime scheduling".to_string(),
            ));
        }
        // The original libfranka does not use mlock. However, we use it to prevent our memory from
        // being swapped.
        if libc::mlockall(libc::MCL_CURRENT | libc::MCL_FUTURE) != 0 {
            return Err(RobotException::RealtimeException(
                "unable to lock memory".to_string(),
            ));
        }
    }

    Ok(())
}
