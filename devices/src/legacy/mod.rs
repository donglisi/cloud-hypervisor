// Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
//
// Portions Copyright 2017 The Chromium OS Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE-BSD-3-Clause file.

#[cfg(target_arch = "x86_64")]
mod debug_port;
#[cfg(target_arch = "aarch64")]
mod rtc_pl031;
mod serial;
#[cfg(target_arch = "aarch64")]
mod uart_pl011;

#[cfg(target_arch = "x86_64")]
pub use self::debug_port::DebugPort;
pub use self::serial::Serial;

#[cfg(target_arch = "aarch64")]
pub use self::rtc_pl031::Rtc;
#[cfg(target_arch = "aarch64")]
pub use self::uart_pl011::Pl011;
