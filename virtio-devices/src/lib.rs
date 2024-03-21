// Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
//
// Portions Copyright 2017 The Chromium OS Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE-BSD-3-Clause file.
//
// Copyright © 2019 Intel Corporation
//
// SPDX-License-Identifier: Apache-2.0 AND BSD-3-Clause

//! Implements virtio devices, queues, and transport mechanisms.

#[macro_use]
extern crate log;

use serde::{Deserialize, Serialize};
use std::io;
use thiserror::Error;

#[macro_use]
mod device;
pub mod block;
mod console;
pub mod epoll_helper;
pub mod net;
mod rng;
pub mod seccomp_filters;
mod thread_helper;
pub mod transport;

pub use self::block::{Block, BlockState};
pub use self::console::{Console, ConsoleResizer, Endpoint};
pub use self::device::{
    DmaRemapping, UserspaceMapping, VirtioCommon, VirtioDevice, VirtioInterrupt,
    VirtioInterruptType, VirtioSharedMemoryList,
};
pub use self::epoll_helper::{
    EpollHelper, EpollHelperError, EpollHelperHandler, EPOLL_HELPER_EVENT_LAST,
};
pub use self::net::{Net, NetCtrlEpollHandler};
pub use self::rng::Rng;
use vm_memory::{bitmap::AtomicBitmap, GuestAddress, GuestMemory};
use vm_virtio::VirtioDeviceType;

type GuestMemoryMmap = vm_memory::GuestMemoryMmap<AtomicBitmap>;
type GuestRegionMmap = vm_memory::GuestRegionMmap<AtomicBitmap>;

const DEVICE_INIT: u32 = 0x00;
const DEVICE_ACKNOWLEDGE: u32 = 0x01;
const DEVICE_DRIVER: u32 = 0x02;
const DEVICE_DRIVER_OK: u32 = 0x04;
const DEVICE_FEATURES_OK: u32 = 0x08;
const DEVICE_FAILED: u32 = 0x80;

const VIRTIO_F_RING_INDIRECT_DESC: u32 = 28;
const VIRTIO_F_VERSION_1: u32 = 32;
const VIRTIO_F_IOMMU_PLATFORM: u32 = 33;
#[allow(dead_code)]
const VIRTIO_F_SR_IOV: u32 = 37;

#[derive(Error, Debug)]
pub enum ActivateError {
    #[error("Failed to activate virtio device")]
    BadActivate,
    #[error("Failed to clone exit event fd: {0}")]
    CloneExitEventFd(std::io::Error),
    #[error("Failed to spawn thread: {0}")]
    ThreadSpawn(std::io::Error),
    #[error("Failed to create seccomp filter: {0}")]
    CreateSeccompFilter(seccompiler::Error),
    #[error("Failed to create rate limiter: {0}")]
    CreateRateLimiter(std::io::Error),
}

pub type ActivateResult = std::result::Result<(), ActivateError>;

pub type DeviceEventT = u16;

#[derive(Error, Debug)]
pub enum Error {
    #[error("Failed to single used queue: {0}")]
    FailedSignalingUsedQueue(io::Error),
    #[error("I/O Error: {0}")]
    IoError(io::Error),
    #[error("Failed to set shared memory region")]
    SetShmRegionsNotSupported,
    #[error("Failed to process net queue: {0}")]
    NetQueuePair(::net_util::NetQueuePairError),
    #[error("Failed to : {0}")]
    QueueAddUsed(virtio_queue::Error),
    #[error("Failed to : {0}")]
    QueueIterator(virtio_queue::Error),
}

#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize, PartialEq, Eq)]
pub struct TokenBucketConfig {
    pub size: u64,
    pub one_time_burst: Option<u64>,
    pub refill_time: u64,
}

/// Convert an absolute address into an address space (GuestMemory)
/// to a host pointer and verify that the provided size define a valid
/// range within a single memory region.
/// Return None if it is out of bounds or if addr+size overlaps a single region.
pub fn get_host_address_range<M: GuestMemory + ?Sized>(
    mem: &M,
    addr: GuestAddress,
    size: usize,
) -> Option<*mut u8> {
    if mem.check_range(addr, size) {
        Some(mem.get_host_address(addr).unwrap())
    } else {
        None
    }
}
