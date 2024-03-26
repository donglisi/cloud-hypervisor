// Copyright 2021 Arm Limited (or its affiliates). All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//! ARM PrimeCell UART(PL011)
//!
//! This module implements an ARM PrimeCell UART(PL011).
//!

use crate::{read_le_u32, write_le_u32};
use std::collections::VecDeque;
use std::fmt;
use std::sync::{Arc, Barrier};
use std::time::Instant;
use std::{io, result};
use versionize::{VersionMap, Versionize, VersionizeResult};
use versionize_derive::Versionize;
use vm_device::interrupt::InterruptSourceGroup;
use vm_device::BusDevice;

/* Registers */
const UARTDR: u64 = 0;
const UARTRSR_UARTECR: u64 = 1;
const UARTFR: u64 = 6;
const UARTILPR: u64 = 8;
const UARTIBRD: u64 = 9;
const UARTFBRD: u64 = 10;
const UARTLCR_H: u64 = 11;
const UARTCR: u64 = 12;
const UARTIFLS: u64 = 13;
const UARTIMSC: u64 = 14;
const UARTRIS: u64 = 15;
const UARTMIS: u64 = 16;
const UARTICR: u64 = 17;
const UARTDMACR: u64 = 18;
const UARTDEBUG: u64 = 0x3c0;

const PL011_INT_TX: u32 = 0x20;
const PL011_INT_RX: u32 = 0x10;

const PL011_FLAG_RXFF: u32 = 0x40;
const PL011_FLAG_RXFE: u32 = 0x10;

const PL011_ID: [u8; 8] = [0x11, 0x10, 0x14, 0x00, 0x0d, 0xf0, 0x05, 0xb1];
// We are only interested in the margins.
const AMBA_ID_LOW: u64 = 0x3f8;
const AMBA_ID_HIGH: u64 = 0x401;

#[derive(Debug)]
pub enum Error {
    BadWriteOffset(u64),
    DmaNotImplemented,
    InterruptFailure(io::Error),
    WriteAllFailure(io::Error),
    FlushFailure(io::Error),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            Error::BadWriteOffset(offset) => write!(f, "pl011_write: Bad Write Offset: {offset}"),
            Error::DmaNotImplemented => write!(f, "pl011: DMA not implemented."),
            Error::InterruptFailure(e) => write!(f, "Failed to trigger interrupt: {e}"),
            Error::WriteAllFailure(e) => write!(f, "Failed to write: {e}"),
            Error::FlushFailure(e) => write!(f, "Failed to flush: {e}"),
        }
    }
}

type Result<T> = result::Result<T, Error>;

/// A PL011 device following the PL011 specification.
pub struct Pl011 {
    flags: u32,
    lcr: u32,
    rsr: u32,
    cr: u32,
    dmacr: u32,
    debug: u32,
    int_enabled: u32,
    int_level: u32,
    read_fifo: VecDeque<u8>,
    ilpr: u32,
    ibrd: u32,
    fbrd: u32,
    ifl: u32,
    read_count: u32,
    read_trigger: u32,
    irq: Arc<dyn InterruptSourceGroup>,
    out: Option<Box<dyn io::Write + Send>>,
    timestamp: std::time::Instant,
}

#[derive(Versionize)]
pub struct Pl011State {
    flags: u32,
    lcr: u32,
    rsr: u32,
    cr: u32,
    dmacr: u32,
    debug: u32,
    int_enabled: u32,
    int_level: u32,
    read_fifo: Vec<u8>,
    ilpr: u32,
    ibrd: u32,
    fbrd: u32,
    ifl: u32,
    read_count: u32,
    read_trigger: u32,
}

impl Pl011 {
    /// Constructs an AMBA PL011 UART device.
    pub fn new(
        irq: Arc<dyn InterruptSourceGroup>,
        out: Option<Box<dyn io::Write + Send>>,
        timestamp: Instant,
        state: Option<Pl011State>,
    ) -> Self {
        let (
            flags,
            lcr,
            rsr,
            cr,
            dmacr,
            debug,
            int_enabled,
            int_level,
            read_fifo,
            ilpr,
            ibrd,
            fbrd,
            ifl,
            read_count,
            read_trigger,
        ) = if let Some(state) = state {
            (
                state.flags,
                state.lcr,
                state.rsr,
                state.cr,
                state.dmacr,
                state.debug,
                state.int_enabled,
                state.int_level,
                state.read_fifo.into(),
                state.ilpr,
                state.ibrd,
                state.fbrd,
                state.ifl,
                state.read_count,
                state.read_trigger,
            )
        } else {
            (
                0x90,
                0,
                0,
                0x300,
                0,
                0,
                0,
                0,
                VecDeque::new(),
                0,
                0,
                0,
                0x12,
                0,
                1,
            )
        };

        Self {
            flags,
            lcr,
            rsr,
            cr,
            dmacr,
            debug,
            int_enabled,
            int_level,
            read_fifo,
            ilpr,
            ibrd,
            fbrd,
            ifl,
            read_count,
            read_trigger,
            irq,
            out,
            timestamp,
        }
    }

    pub fn set_out(&mut self, out: Option<Box<dyn io::Write + Send>>) {
        self.out = out;
    }

    /// Queues raw bytes for the guest to read and signals the interrupt
    pub fn queue_input_bytes(&mut self, c: &[u8]) -> vmm_sys_util::errno::Result<()> {
        self.read_fifo.extend(c);
        self.read_count += c.len() as u32;
        self.flags &= !PL011_FLAG_RXFE;

        if ((self.lcr & 0x10) == 0) || (self.read_count == 16) {
            self.flags |= PL011_FLAG_RXFF;
        }

        if self.read_count >= self.read_trigger {
            self.int_level |= PL011_INT_RX;
            self.trigger_interrupt()?;
        }

        Ok(())
    }

    pub fn flush_output(&mut self) -> result::Result<(), io::Error> {
        if let Some(out) = self.out.as_mut() {
            out.flush()?;
        }
        Ok(())
    }

    fn pl011_get_baudrate(&self) -> u32 {
        if self.fbrd == 0 {
            return 0;
        }

        let clk = 24_000_000; // We set the APB_PLCK to 24M in device tree
        (clk / ((self.ibrd << 6) + self.fbrd)) << 2
    }

    fn pl011_trace_baudrate_change(&self) {
        debug!(
            "=== New baudrate: {:#?} (clk: {:#?}Hz, ibrd: {:#?}, fbrd: {:#?}) ===",
            self.pl011_get_baudrate(),
            24_000_000, // We set the APB_PLCK to 24M in device tree
            self.ibrd,
            self.fbrd
        );
    }

    fn pl011_set_read_trigger(&mut self) {
        self.read_trigger = 1;
    }

    fn handle_write(&mut self, offset: u64, val: u32) -> Result<()> {
        match offset >> 2 {
            UARTDR => {
                self.int_level |= PL011_INT_TX;
                if let Some(out) = self.out.as_mut() {
                    out.write_all(&[val.to_le_bytes()[0]])
                        .map_err(Error::WriteAllFailure)?;
                    out.flush().map_err(Error::FlushFailure)?;
                }
            }
            UARTRSR_UARTECR => {
                self.rsr = 0;
            }
            UARTFR => { /* Writes to Flag register are ignored.*/ }
            UARTILPR => {
                self.ilpr = val;
            }
            UARTIBRD => {
                self.ibrd = val;
                self.pl011_trace_baudrate_change();
            }
            UARTFBRD => {
                self.fbrd = val;
                self.pl011_trace_baudrate_change();
            }
            UARTLCR_H => {
                /* Reset the FIFO state on FIFO enable or disable */
                if ((self.lcr ^ val) & 0x10) != 0 {
                    self.read_count = 0;
                }
                self.lcr = val;
                self.pl011_set_read_trigger();
            }
            UARTCR => {
                self.cr = val;
            }
            UARTIFLS => {
                self.ifl = val;
                self.pl011_set_read_trigger();
            }
            UARTIMSC => {
                self.int_enabled = val;
                self.trigger_interrupt().map_err(Error::InterruptFailure)?;
            }
            UARTICR => {
                self.int_level &= !val;
                self.trigger_interrupt().map_err(Error::InterruptFailure)?;
            }
            UARTDMACR => {
                self.dmacr = val;
                if (val & 3) != 0 {
                    return Err(Error::DmaNotImplemented);
                }
            }
            UARTDEBUG => {
                self.debug = val;
                self.handle_debug();
            }
            off => {
                debug!("PL011: Bad write offset, offset: {}", off);
                return Err(Error::BadWriteOffset(off));
            }
        }
        Ok(())
    }

    fn handle_debug(&self) {
        let elapsed = self.timestamp.elapsed();

        match self.debug {
            0x00..=0x1f => warn!(
                "[Debug I/O port: Firmware code: 0x{:x}] {}.{:>06} seconds",
                self.debug,
                elapsed.as_secs(),
                elapsed.as_micros()
            ),
            0x20..=0x3f => warn!(
                "[Debug I/O port: Bootloader code: 0x{:x}] {}.{:>06} seconds",
                self.debug,
                elapsed.as_secs(),
                elapsed.as_micros()
            ),
            0x40..=0x5f => warn!(
                "[Debug I/O port: Kernel code: 0x{:x}] {}.{:>06} seconds",
                self.debug,
                elapsed.as_secs(),
                elapsed.as_micros()
            ),
            0x60..=0x7f => warn!(
                "[Debug I/O port: Userspace code: 0x{:x}] {}.{:>06} seconds",
                self.debug,
                elapsed.as_secs(),
                elapsed.as_micros()
            ),
            _ => {}
        }
    }

    fn trigger_interrupt(&mut self) -> result::Result<(), io::Error> {
        self.irq.trigger(0)
    }
}

impl BusDevice for Pl011 {
    fn read(&mut self, _base: u64, offset: u64, data: &mut [u8]) {
        let mut read_ok = true;
        let v = if (AMBA_ID_LOW..AMBA_ID_HIGH).contains(&(offset >> 2)) {
            let index = ((offset - 0xfe0) >> 2) as usize;
            u32::from(PL011_ID[index])
        } else {
            match offset >> 2 {
                UARTDR => {
                    self.flags &= !PL011_FLAG_RXFF;
                    let c: u32 = self.read_fifo.pop_front().unwrap_or_default().into();
                    if self.read_count > 0 {
                        self.read_count -= 1;
                    }
                    if self.read_count == 0 {
                        self.flags |= PL011_FLAG_RXFE;
                    }
                    if self.read_count == (self.read_trigger - 1) {
                        self.int_level &= !PL011_INT_RX;
                    }
                    self.rsr = c >> 8;
                    c
                }
                UARTRSR_UARTECR => self.rsr,
                UARTFR => self.flags,
                UARTILPR => self.ilpr,
                UARTIBRD => self.ibrd,
                UARTFBRD => self.fbrd,
                UARTLCR_H => self.lcr,
                UARTCR => self.cr,
                UARTIFLS => self.ifl,
                UARTIMSC => self.int_enabled,
                UARTRIS => self.int_level,
                UARTMIS => self.int_level & self.int_enabled,
                UARTDMACR => self.dmacr,
                UARTDEBUG => self.debug,
                _ => {
                    read_ok = false;
                    0
                }
            }
        };

        if read_ok && data.len() <= 4 {
            write_le_u32(data, v);
        } else {
            warn!(
                "Invalid PL011 read: offset {}, data length {}",
                offset,
                data.len()
            );
        }
    }

    fn write(&mut self, _base: u64, offset: u64, data: &[u8]) -> Option<Arc<Barrier>> {
        if data.len() <= 4 {
            let v = read_le_u32(data);
            if let Err(e) = self.handle_write(offset, v) {
                warn!("Failed to write to PL011 device: {}", e);
            }
        } else {
            warn!(
                "Invalid PL011 write: offset {}, data length {}",
                offset,
                data.len()
            );
        }

        None
    }
}
