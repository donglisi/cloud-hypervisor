// Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
//
// Portions Copyright 2017 The Chromium OS Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE-BSD-3-Clause file.

use std::collections::VecDeque;
use std::sync::{Arc, Barrier};
use std::{io, result};
use versionize::{VersionMap, Versionize, VersionizeResult};
use versionize_derive::Versionize;
use vm_device::interrupt::InterruptSourceGroup;
use vm_device::BusDevice;
use vmm_sys_util::errno::Result;

const LOOP_SIZE: usize = 0x40;

const DATA: u8 = 0;
const IER: u8 = 1;
const IIR: u8 = 2;
const LCR: u8 = 3;
const MCR: u8 = 4;
const LSR: u8 = 5;
const MSR: u8 = 6;
const SCR: u8 = 7;

const DLAB_LOW: u8 = 0;
const DLAB_HIGH: u8 = 1;

const IER_RECV_BIT: u8 = 0x1;
const IER_THR_BIT: u8 = 0x2;
const IER_FIFO_BITS: u8 = 0x0f;

const IIR_FIFO_BITS: u8 = 0xc0;
const IIR_NONE_BIT: u8 = 0x1;
const IIR_THR_BIT: u8 = 0x2;
const IIR_RECV_BIT: u8 = 0x4;

const LCR_DLAB_BIT: u8 = 0x80;

const LSR_DATA_BIT: u8 = 0x1;
const LSR_EMPTY_BIT: u8 = 0x20;
const LSR_IDLE_BIT: u8 = 0x40;

const MCR_LOOP_BIT: u8 = 0x10;

const DEFAULT_INTERRUPT_IDENTIFICATION: u8 = IIR_NONE_BIT; // no pending interrupt
const DEFAULT_LINE_STATUS: u8 = LSR_EMPTY_BIT | LSR_IDLE_BIT; // THR empty and line is idle
const DEFAULT_LINE_CONTROL: u8 = 0x3; // 8-bits per character
const DEFAULT_MODEM_CONTROL: u8 = 0x8; // Auxiliary output 2
const DEFAULT_MODEM_STATUS: u8 = 0x20 | 0x10 | 0x80; // data ready, clear to send, carrier detect
const DEFAULT_BAUD_DIVISOR: u16 = 12; // 9600 bps

/// Emulates serial COM ports commonly seen on x86 I/O ports 0x3f8/0x2f8/0x3e8/0x2e8.
///
/// This can optionally write the guest's output to a Write trait object. To send input to the
/// guest, use `queue_input_bytes`.
pub struct Serial {
    interrupt_enable: u8,
    interrupt_identification: u8,
    line_control: u8,
    line_status: u8,
    modem_control: u8,
    modem_status: u8,
    scratch: u8,
    baud_divisor: u16,
    in_buffer: VecDeque<u8>,
    interrupt: Arc<dyn InterruptSourceGroup>,
    out: Option<Box<dyn io::Write + Send>>,
}

#[derive(Versionize)]
pub struct SerialState {
    interrupt_enable: u8,
    interrupt_identification: u8,
    line_control: u8,
    line_status: u8,
    modem_control: u8,
    modem_status: u8,
    scratch: u8,
    baud_divisor: u16,
    in_buffer: Vec<u8>,
}

impl Serial {
    pub fn new(
        interrupt: Arc<dyn InterruptSourceGroup>,
        out: Option<Box<dyn io::Write + Send>>,
    ) -> Serial {
        let (
            interrupt_enable,
            interrupt_identification,
            line_control,
            line_status,
            modem_control,
            modem_status,
            scratch,
            baud_divisor,
            in_buffer,
        ) = {
            (
                0,
                DEFAULT_INTERRUPT_IDENTIFICATION,
                DEFAULT_LINE_CONTROL,
                DEFAULT_LINE_STATUS,
                DEFAULT_MODEM_CONTROL,
                DEFAULT_MODEM_STATUS,
                0,
                DEFAULT_BAUD_DIVISOR,
                VecDeque::new(),
            )
        };

        Serial {
            interrupt_enable,
            interrupt_identification,
            line_control,
            line_status,
            modem_control,
            modem_status,
            scratch,
            baud_divisor,
            in_buffer,
            interrupt,
            out,
        }
    }

    /// Constructs a Serial port ready for output.
    pub fn new_out(
        interrupt: Arc<dyn InterruptSourceGroup>,
        out: Box<dyn io::Write + Send>,
    ) -> Serial {
        Self::new(interrupt, Some(out))
    }

    /// Constructs a Serial port with no connected output.
    pub fn new_sink(
        interrupt: Arc<dyn InterruptSourceGroup>,
    ) -> Serial {
        Self::new(interrupt, None)
    }

    pub fn set_out(&mut self, out: Option<Box<dyn io::Write + Send>>) {
        self.out = out;
    }

    /// Queues raw bytes for the guest to read and signals the interrupt if the line status would
    /// change.
    pub fn queue_input_bytes(&mut self, c: &[u8]) -> Result<()> {
        if !self.is_loop() {
            self.in_buffer.extend(c);
            self.recv_data()?;
        }
        Ok(())
    }

    pub fn flush_output(&mut self) -> result::Result<(), io::Error> {
        if let Some(out) = self.out.as_mut() {
            out.flush()?;
        }
        Ok(())
    }

    fn is_dlab_set(&self) -> bool {
        (self.line_control & LCR_DLAB_BIT) != 0
    }

    fn is_recv_intr_enabled(&self) -> bool {
        (self.interrupt_enable & IER_RECV_BIT) != 0
    }

    fn is_thr_intr_enabled(&self) -> bool {
        (self.interrupt_enable & IER_THR_BIT) != 0
    }

    fn is_loop(&self) -> bool {
        (self.modem_control & MCR_LOOP_BIT) != 0
    }

    fn add_intr_bit(&mut self, bit: u8) {
        self.interrupt_identification &= !IIR_NONE_BIT;
        self.interrupt_identification |= bit;
    }

    fn del_intr_bit(&mut self, bit: u8) {
        self.interrupt_identification &= !bit;
        if self.interrupt_identification == 0x0 {
            self.interrupt_identification = IIR_NONE_BIT;
        }
    }

    fn thr_empty(&mut self) -> Result<()> {
        if self.is_thr_intr_enabled() {
            self.add_intr_bit(IIR_THR_BIT);
            self.trigger_interrupt()?
        }
        Ok(())
    }

    fn recv_data(&mut self) -> Result<()> {
        if self.is_recv_intr_enabled() {
            self.add_intr_bit(IIR_RECV_BIT);
            self.trigger_interrupt()?
        }
        self.line_status |= LSR_DATA_BIT;
        Ok(())
    }

    fn trigger_interrupt(&mut self) -> result::Result<(), io::Error> {
        self.interrupt.trigger(0)
    }

    fn iir_reset(&mut self) {
        self.interrupt_identification = DEFAULT_INTERRUPT_IDENTIFICATION;
    }

    fn handle_write(&mut self, offset: u8, v: u8) -> Result<()> {
        match offset {
            DLAB_LOW if self.is_dlab_set() => {
                self.baud_divisor = (self.baud_divisor & 0xff00) | u16::from(v)
            }
            DLAB_HIGH if self.is_dlab_set() => {
                self.baud_divisor = (self.baud_divisor & 0x00ff) | ((u16::from(v)) << 8)
            }
            DATA => {
                if self.is_loop() {
                    if self.in_buffer.len() < LOOP_SIZE {
                        self.in_buffer.push_back(v);
                        self.recv_data()?;
                    }
                } else {
                    if let Some(out) = self.out.as_mut() {
                        out.write_all(&[v])?;
                        out.flush()?;
                    }
                    self.thr_empty()?;
                }
            }
            IER => self.interrupt_enable = v & IER_FIFO_BITS,
            LCR => self.line_control = v,
            MCR => self.modem_control = v,
            SCR => self.scratch = v,
            _ => {}
        }
        Ok(())
    }
}

impl BusDevice for Serial {
    fn read(&mut self, _base: u64, offset: u64, data: &mut [u8]) {
        if data.len() != 1 {
            return;
        }

        data[0] = match offset as u8 {
            DLAB_LOW if self.is_dlab_set() => self.baud_divisor as u8,
            DLAB_HIGH if self.is_dlab_set() => (self.baud_divisor >> 8) as u8,
            DATA => {
                self.del_intr_bit(IIR_RECV_BIT);
                if self.in_buffer.len() <= 1 {
                    self.line_status &= !LSR_DATA_BIT;
                }
                self.in_buffer.pop_front().unwrap_or_default()
            }
            IER => self.interrupt_enable,
            IIR => {
                let v = self.interrupt_identification | IIR_FIFO_BITS;
                self.iir_reset();
                v
            }
            LCR => self.line_control,
            MCR => self.modem_control,
            LSR => self.line_status,
            MSR => self.modem_status,
            SCR => self.scratch,
            _ => 0,
        };
    }

    fn write(&mut self, _base: u64, offset: u64, data: &[u8]) -> Option<Arc<Barrier>> {
        if data.len() != 1 {
            return None;
        }

        self.handle_write(offset as u8, data[0]).ok();

        None
    }
}
