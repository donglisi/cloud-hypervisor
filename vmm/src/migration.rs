// Copyright © 2020 Intel Corporation
//
// SPDX-License-Identifier: Apache-2.0

#[cfg(all(target_arch = "x86_64", feature = "guest_debug"))]
use crate::coredump::GuestDebuggableError;
use crate::{vm::VmSnapshot};
use anyhow::anyhow;
use std::path::PathBuf;
use vm_migration::{MigratableError, Snapshot};

pub const SNAPSHOT_STATE_FILE: &str = "state.json";
pub const SNAPSHOT_CONFIG_FILE: &str = "config.json";

pub fn url_to_path(url: &str) -> std::result::Result<PathBuf, MigratableError> {
    let path: PathBuf = url
        .strip_prefix("file://")
        .ok_or_else(|| {
            MigratableError::MigrateSend(anyhow!("Could not extract path from URL: {}", url))
        })
        .map(|s| s.into())?;

    if !path.is_dir() {
        return Err(MigratableError::MigrateSend(anyhow!(
            "Destination is not a directory"
        )));
    }

    Ok(path)
}

#[cfg(all(target_arch = "x86_64", feature = "guest_debug"))]
pub fn url_to_file(url: &str) -> std::result::Result<PathBuf, GuestDebuggableError> {
    let file: PathBuf = url
        .strip_prefix("file://")
        .ok_or_else(|| {
            GuestDebuggableError::Coredump(anyhow!("Could not extract file from URL: {}", url))
        })
        .map(|s| s.into())?;

    Ok(file)
}

pub fn get_vm_snapshot(snapshot: &Snapshot) -> std::result::Result<VmSnapshot, MigratableError> {
    if let Some(snapshot_data) = snapshot.snapshot_data.as_ref() {
        return snapshot_data.to_state();
    }

    Err(MigratableError::Restore(anyhow!(
        "Could not find VM config snapshot section"
    )))
}
