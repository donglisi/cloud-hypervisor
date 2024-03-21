// Copyright © 2021 Intel Corporation
//
// SPDX-License-Identifier: Apache-2.0
//

use crate::{
    epoll_helper::EpollHelperError,
    ActivateError,
};
use std::{
    panic::AssertUnwindSafe,
    thread::{self, JoinHandle},
};
use vmm_sys_util::eventfd::EventFd;

pub(crate) fn spawn_virtio_thread<F>(
    name: &str,
    epoll_threads: &mut Vec<JoinHandle<()>>,
    exit_evt: &EventFd,
    f: F,
) -> Result<(), ActivateError>
where
    F: FnOnce() -> std::result::Result<(), EpollHelperError>,
    F: Send + 'static,
{
    let thread_exit_evt = exit_evt
        .try_clone()
        .map_err(ActivateError::CloneExitEventFd)?;
    let thread_name = name.to_string();

    thread::Builder::new()
        .name(name.to_string())
        .spawn(move || {
            match std::panic::catch_unwind(AssertUnwindSafe(f)) {
                Err(_) => {
                    error!("{} thread panicked", thread_name);
                    thread_exit_evt.write(1).ok();
                }
                Ok(r) => {
                    if let Err(e) = r {
                        error!("Error running worker: {:?}", e);
                        thread_exit_evt.write(1).ok();
                    }
                }
            };
        })
        .map(|thread| epoll_threads.push(thread))
        .map_err(|e| {
            error!("Failed to spawn thread for {}: {}", name, e);
            ActivateError::ThreadSpawn(e)
        })
}
