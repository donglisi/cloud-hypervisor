// Copyright © 2020, Oracle and/or its affiliates.
//
// Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
//
// Portions Copyright 2017 The Chromium OS Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE-BSD-3-Clause file.
//
// Copyright © 2019 Intel Corporation
//
// SPDX-License-Identifier: Apache-2.0 AND BSD-3-Clause
//

use crate::config::{
    ValidationError, VmConfig,
};
use crate::config::{PayloadConfig};
#[cfg(all(target_arch = "x86_64", feature = "guest_debug"))]
use crate::coredump::{
    CpuElf64Writable, DumpState, Elf64Writable, GuestDebuggable, GuestDebuggableError, NoteDescType,
};
use crate::cpu;
use crate::device_manager::{DeviceManager, DeviceManagerError, PtyPair};
use crate::device_tree::DeviceTree;
#[cfg(feature = "guest_debug")]
use crate::gdb::{Debuggable, DebuggableError, GdbRequestPayload, GdbResponsePayload};
#[cfg(feature = "igvm")]
use crate::igvm::igvm_loader;
use crate::memory_manager::{
    Error as MemoryManagerError, MemoryManager,
};
#[cfg(all(target_arch = "x86_64", feature = "guest_debug"))]
use crate::migration::url_to_file;
use crate::GuestMemoryMmap;
use arch::get_host_cpu_phys_bits;
#[cfg(target_arch = "x86_64")]
use arch::layout::{KVM_IDENTITY_MAP_START, KVM_TSS_START};
#[cfg(feature = "tdx")]
use arch::x86_64::tdx::TdvfSection;
use arch::EntryPoint;
#[cfg(target_arch = "aarch64")]
use devices::interrupt_controller;
#[cfg(all(target_arch = "aarch64", feature = "guest_debug"))]
use gdbstub_arch::aarch64::reg::AArch64CoreRegs as CoreRegs;
#[cfg(all(target_arch = "x86_64", feature = "guest_debug"))]
use gdbstub_arch::x86::reg::X86_64CoreRegs as CoreRegs;
use hypervisor::{HypervisorVmError, VmOps};
use libc::{termios, SIGWINCH};
use linux_loader::cmdline::Cmdline;
#[cfg(all(target_arch = "x86_64", feature = "guest_debug"))]
use linux_loader::elf;
#[cfg(target_arch = "x86_64")]
use linux_loader::loader::elf::PvhBootCapability::PvhEntryPresent;
#[cfg(target_arch = "aarch64")]
use linux_loader::loader::pe::Error::InvalidImageMagicNumber;
use linux_loader::loader::KernelLoader;
use serde::{Deserialize, Serialize};
use std::cmp;
use std::collections::HashMap;
use std::convert::TryInto;
use std::fs::File;
use std::io::{self, Seek, SeekFrom};
#[cfg(all(target_arch = "x86_64", feature = "guest_debug"))]
use std::mem::size_of;
use std::num::Wrapping;
use std::ops::Deref;
use std::sync::{Arc, Mutex, RwLock};
use std::{result, str, thread};
use thiserror::Error;
use vm_device::Bus;
#[cfg(feature = "tdx")]
use vm_memory::{Address, ByteValued, GuestMemoryRegion, ReadVolatile};
use vm_memory::{
    Bytes, GuestAddress, GuestAddressSpace, GuestMemory, GuestMemoryAtomic,
};
use vmm_sys_util::eventfd::EventFd;

/// Errors associated with VM management
#[derive(Debug, Error)]
pub enum Error {
    #[error("Cannot open kernel file: {0}")]
    KernelFile(#[source] io::Error),

    #[error("Cannot open initramfs file: {0}")]
    InitramfsFile(#[source] io::Error),

    #[error("Cannot load the kernel into memory: {0}")]
    KernelLoad(#[source] linux_loader::loader::Error),

    #[error("Cannot load the initramfs into memory")]
    InitramfsLoad,

    #[error("Cannot load the kernel command line in memory: {0}")]
    LoadCmdLine(#[source] linux_loader::loader::Error),

    #[error("Cannot modify the kernel command line: {0}")]
    CmdLineInsertStr(#[source] linux_loader::cmdline::Error),

    #[error("Cannot create the kernel command line: {0}")]
    CmdLineCreate(#[source] linux_loader::cmdline::Error),

    #[error("Cannot configure system: {0}")]
    ConfigureSystem(#[source] arch::Error),

    #[cfg(target_arch = "aarch64")]
    #[error("Cannot enable interrupt controller: {0:?}")]
    EnableInterruptController(interrupt_controller::Error),

    #[error("VM state is poisoned")]
    PoisonedState,

    #[error("Error from device manager: {0:?}")]
    DeviceManager(DeviceManagerError),

    #[error("No device with id {0:?} to remove")]
    NoDeviceToRemove(String),

    #[error("Cannot spawn a signal handler thread: {0}")]
    SignalHandlerSpawn(#[source] io::Error),

    #[error("Failed to join on threads: {0:?}")]
    ThreadCleanup(std::boxed::Box<dyn std::any::Any + std::marker::Send>),

    #[error("VM config is missing")]
    VmMissingConfig,

    #[error("VM is not created")]
    VmNotCreated,

    #[error("VM is already created")]
    VmAlreadyCreated,

    #[error("VM is not running")]
    VmNotRunning,

    #[error("Cannot clone EventFd: {0}")]
    EventFdClone(#[source] io::Error),

    #[error("invalid VM state transition: {0:?} to {1:?}")]
    InvalidStateTransition(VmState, VmState),

    #[error("Error from CPU manager: {0}")]
    CpuManager(#[source] cpu::Error),

    #[error("Memory manager error: {0:?}")]
    MemoryManager(MemoryManagerError),

    #[error("Eventfd write error: {0}")]
    EventfdError(#[source] std::io::Error),

    #[error("Invalid restore source URL")]
    InvalidRestoreSourceUrl,

    #[error("Failed to validate config: {0}")]
    ConfigValidation(#[source] ValidationError),

    #[error("Too many virtio-vsock devices")]
    TooManyVsockDevices,

    #[error("Invalid NUMA configuration")]
    InvalidNumaConfig,

    #[error("Failed resizing a memory zone")]
    ResizeZone,

    #[error("Cannot activate virtio devices: {0:?}")]
    ActivateVirtioDevices(DeviceManagerError),

    #[error("Error triggering power button: {0:?}")]
    PowerButton(DeviceManagerError),

    #[error("Kernel lacks PVH header")]
    KernelMissingPvhHeader,

    #[error("Failed to allocate firmware RAM: {0:?}")]
    AllocateFirmwareMemory(MemoryManagerError),

    #[error("Error manipulating firmware file: {0}")]
    FirmwareFile(#[source] std::io::Error),

    #[error("Firmware too big")]
    FirmwareTooLarge,

    #[error("Failed to copy firmware to memory: {0}")]
    FirmwareLoad(#[source] vm_memory::GuestMemoryError),

    #[cfg(feature = "sev_snp")]
    #[error("Error enabling SEV-SNP VM: {0}")]
    InitializeSevSnpVm(#[source] hypervisor::HypervisorVmError),

    #[cfg(feature = "tdx")]
    #[error("Error performing I/O on TDX firmware file: {0}")]
    LoadTdvf(#[source] std::io::Error),

    #[cfg(feature = "tdx")]
    #[error("Error performing I/O on the TDX payload file: {0}")]
    LoadPayload(#[source] std::io::Error),

    #[cfg(feature = "tdx")]
    #[error("Error parsing TDVF: {0}")]
    ParseTdvf(#[source] arch::x86_64::tdx::TdvfError),

    #[cfg(feature = "tdx")]
    #[error("Error populating TDX HOB: {0}")]
    PopulateHob(#[source] arch::x86_64::tdx::TdvfError),

    #[cfg(feature = "tdx")]
    #[error("Error allocating TDVF memory: {0:?}")]
    AllocatingTdvfMemory(crate::memory_manager::Error),

    #[cfg(feature = "tdx")]
    #[error("Error enabling TDX VM: {0}")]
    InitializeTdxVm(#[source] hypervisor::HypervisorVmError),

    #[cfg(feature = "tdx")]
    #[error("Error enabling TDX memory region: {0}")]
    InitializeTdxMemoryRegion(#[source] hypervisor::HypervisorVmError),

    #[cfg(feature = "tdx")]
    #[error("Error finalizing TDX VM: {0}")]
    FinalizeTdx(#[source] hypervisor::HypervisorVmError),

    #[cfg(feature = "tdx")]
    #[error("TDX firmware missing")]
    TdxFirmwareMissing,

    #[cfg(feature = "tdx")]
    #[error("Invalid TDX payload type")]
    InvalidPayloadType,

    #[cfg(feature = "guest_debug")]
    #[error("Error debugging VM: {0:?}")]
    Debug(DebuggableError),

    #[error("Error spawning kernel loading thread")]
    KernelLoadThreadSpawn(std::io::Error),

    #[error("Error joining kernel loading thread")]
    KernelLoadThreadJoin(std::boxed::Box<dyn std::any::Any + std::marker::Send>),

    #[error("Payload configuration is not bootable")]
    InvalidPayload,

    #[cfg(all(target_arch = "x86_64", feature = "guest_debug"))]
    #[error("Error coredumping VM: {0:?}")]
    Coredump(GuestDebuggableError),

    #[cfg(feature = "igvm")]
    #[error("Cannot open igvm file: {0}")]
    IgvmFile(#[source] io::Error),

    #[cfg(feature = "igvm")]
    #[error("Cannot load the igvm into memory: {0}")]
    IgvmLoad(#[source] igvm_loader::Error),
}
pub type Result<T> = result::Result<T, Error>;

#[derive(Clone, Copy, Debug, Deserialize, Serialize, PartialEq, Eq)]
pub enum VmState {
    Created,
    Running,
    Shutdown,
    Paused,
    BreakPoint,
}

impl VmState {
    fn valid_transition(self, new_state: VmState) -> Result<()> {
        match self {
            VmState::Created => match new_state {
                VmState::Created => Err(Error::InvalidStateTransition(self, new_state)),
                VmState::Running | VmState::Paused | VmState::BreakPoint | VmState::Shutdown => {
                    Ok(())
                }
            },

            VmState::Running => match new_state {
                VmState::Created | VmState::Running => {
                    Err(Error::InvalidStateTransition(self, new_state))
                }
                VmState::Paused | VmState::Shutdown | VmState::BreakPoint => Ok(()),
            },

            VmState::Shutdown => match new_state {
                VmState::Paused | VmState::Created | VmState::Shutdown | VmState::BreakPoint => {
                    Err(Error::InvalidStateTransition(self, new_state))
                }
                VmState::Running => Ok(()),
            },

            VmState::Paused => match new_state {
                VmState::Created | VmState::Paused | VmState::BreakPoint => {
                    Err(Error::InvalidStateTransition(self, new_state))
                }
                VmState::Running | VmState::Shutdown => Ok(()),
            },
            VmState::BreakPoint => match new_state {
                VmState::Created | VmState::Running => Ok(()),
                _ => Err(Error::InvalidStateTransition(self, new_state)),
            },
        }
    }
}

struct VmOpsHandler {
    memory: GuestMemoryAtomic<GuestMemoryMmap>,
    #[cfg(target_arch = "x86_64")]
    io_bus: Arc<Bus>,
    mmio_bus: Arc<Bus>,
}

impl VmOps for VmOpsHandler {
    fn guest_mem_write(&self, gpa: u64, buf: &[u8]) -> result::Result<usize, HypervisorVmError> {
        self.memory
            .memory()
            .write(buf, GuestAddress(gpa))
            .map_err(|e| HypervisorVmError::GuestMemWrite(e.into()))
    }

    fn guest_mem_read(&self, gpa: u64, buf: &mut [u8]) -> result::Result<usize, HypervisorVmError> {
        self.memory
            .memory()
            .read(buf, GuestAddress(gpa))
            .map_err(|e| HypervisorVmError::GuestMemRead(e.into()))
    }

    fn mmio_read(&self, gpa: u64, data: &mut [u8]) -> result::Result<(), HypervisorVmError> {
        if let Err(vm_device::BusError::MissingAddressRange) = self.mmio_bus.read(gpa, data) {
            info!("Guest MMIO read to unregistered address 0x{:x}", gpa);
        }
        Ok(())
    }

    fn mmio_write(&self, gpa: u64, data: &[u8]) -> result::Result<(), HypervisorVmError> {
        match self.mmio_bus.write(gpa, data) {
            Err(vm_device::BusError::MissingAddressRange) => {
                info!("Guest MMIO write to unregistered address 0x{:x}", gpa);
            }
            Ok(Some(barrier)) => {
                info!("Waiting for barrier");
                barrier.wait();
                info!("Barrier released");
            }
            _ => {}
        };
        Ok(())
    }

    #[cfg(target_arch = "x86_64")]
    fn pio_read(&self, port: u64, data: &mut [u8]) -> result::Result<(), HypervisorVmError> {
        if let Err(vm_device::BusError::MissingAddressRange) = self.io_bus.read(port, data) {
            info!("Guest PIO read to unregistered address 0x{:x}", port);
        }
        Ok(())
    }

    #[cfg(target_arch = "x86_64")]
    fn pio_write(&self, port: u64, data: &[u8]) -> result::Result<(), HypervisorVmError> {
        match self.io_bus.write(port, data) {
            Err(vm_device::BusError::MissingAddressRange) => {
                info!("Guest PIO write to unregistered address 0x{:x}", port);
            }
            Ok(Some(barrier)) => {
                info!("Waiting for barrier");
                barrier.wait();
                info!("Barrier released");
            }
            _ => {}
        };
        Ok(())
    }
}

pub fn physical_bits(hypervisor: &Arc<dyn hypervisor::Hypervisor>, max_phys_bits: u8) -> u8 {
    let host_phys_bits = get_host_cpu_phys_bits(hypervisor);

    cmp::min(host_phys_bits, max_phys_bits)
}

pub struct Vm {
    #[cfg(feature = "tdx")]
    kernel: Option<File>,
    initramfs: Option<File>,
    threads: Vec<thread::JoinHandle<()>>,
    device_manager: Arc<Mutex<DeviceManager>>,
    config: Arc<Mutex<VmConfig>>,
    state: RwLock<VmState>,
    cpu_manager: Arc<Mutex<cpu::CpuManager>>,
    memory_manager: Arc<Mutex<MemoryManager>>,
    stop_on_boot: bool,
    load_payload_handle: Option<thread::JoinHandle<Result<EntryPoint>>>,
}

impl Vm {
    pub const HANDLED_SIGNALS: [i32; 1] = [SIGWINCH];

    #[allow(clippy::too_many_arguments)]
    pub fn new_from_memory_manager(
        config: Arc<Mutex<VmConfig>>,
        memory_manager: Arc<Mutex<MemoryManager>>,
        vm: Arc<dyn hypervisor::Vm>,
        exit_evt: EventFd,
        reset_evt: EventFd,
        #[cfg(feature = "guest_debug")] vm_debug_evt: EventFd,
        hypervisor: Arc<dyn hypervisor::Hypervisor>,
        serial_pty: Option<PtyPair>,
        original_termios: Arc<Mutex<Option<termios>>>,
    ) -> Result<Self> {
        let load_payload_handle = Self::load_payload_async(&memory_manager, &config)?;

        info!("Booting VM from config: {:?}", &config);

        #[cfg(feature = "tdx")]
        let tdx_enabled = config.lock().unwrap().is_tdx_enabled();
        #[cfg(feature = "sev_snp")]
        let sev_snp_enabled = config.lock().unwrap().is_sev_snp_enabled();
        #[cfg(feature = "tdx")]
        let force_iommu = tdx_enabled;
        #[cfg(feature = "sev_snp")]
        let force_iommu = sev_snp_enabled;

        #[cfg(feature = "guest_debug")]
        let stop_on_boot = config.lock().unwrap().gdb;
        #[cfg(not(feature = "guest_debug"))]
        let stop_on_boot = false;

        let memory = memory_manager.lock().unwrap().guest_memory();
        #[cfg(target_arch = "x86_64")]
        let io_bus = Arc::new(Bus::new());
        let mmio_bus = Arc::new(Bus::new());

        let vm_ops: Arc<dyn VmOps> = Arc::new(VmOpsHandler {
            memory,
            #[cfg(target_arch = "x86_64")]
            io_bus: io_bus.clone(),
            mmio_bus: mmio_bus.clone(),
        });

        let cpus_config = { &config.lock().unwrap().cpus.clone() };
        let cpu_manager = cpu::CpuManager::new(
            cpus_config,
            vm.clone(),
            exit_evt.try_clone().map_err(Error::EventFdClone)?,
            reset_evt.try_clone().map_err(Error::EventFdClone)?,
            #[cfg(feature = "guest_debug")]
            vm_debug_evt,
            &hypervisor,
            vm_ops,
            #[cfg(feature = "tdx")]
            tdx_enabled,
            #[cfg(feature = "sev_snp")]
            sev_snp_enabled,
        )
        .map_err(Error::CpuManager)?;

        #[cfg(target_arch = "x86_64")]
        cpu_manager
            .lock()
            .unwrap()
            .populate_cpuid(
                &memory_manager,
                &hypervisor,
                #[cfg(feature = "tdx")]
                tdx_enabled,
            )
            .map_err(Error::CpuManager)?;

        // Loading the igvm file is pushed down here because
        // igvm parser needs cpu_manager to retrieve cpuid leaf.
        // For the regular case, we can start loading early, but for
        // igvm case we have to wait until cpu_manager is created.
        // Currently, Microsoft Hypervisor does not provide any
        // Hypervisor specific common cpuid, we need to call get_cpuid_values
        // per cpuid through cpu_manager.
        #[cfg(feature = "igvm")]
        let load_payload_handle = if snapshot.is_none() {
            Self::load_payload_async(&memory_manager, &config, &cpu_manager)?
        } else {
            None
        };
        // The initial TDX configuration must be done before the vCPUs are
        // created
        #[cfg(feature = "tdx")]
        if tdx_enabled {
            let cpuid = cpu_manager.lock().unwrap().common_cpuid();
            let max_vcpus = cpu_manager.lock().unwrap().max_vcpus() as u32;
            vm.tdx_init(&cpuid, max_vcpus)
                .map_err(Error::InitializeTdxVm)?;
        }

        cpu_manager
            .lock()
            .unwrap()
            .create_boot_vcpus()
            .map_err(Error::CpuManager)?;

        // This initial SEV-SNP configuration must be done immediately after
        // vCPUs are created. As part of this initialization we are
        // transitioning the guest into secure state.
        #[cfg(feature = "sev_snp")]
        if sev_snp_enabled {
            vm.sev_snp_init().map_err(Error::InitializeSevSnpVm)?;
        }

        #[cfg(feature = "tdx")]
        let dynamic = !tdx_enabled;
        #[cfg(not(feature = "tdx"))]
        let dynamic = true;

        let device_manager = DeviceManager::new(
            #[cfg(target_arch = "x86_64")]
            io_bus,
            mmio_bus,
            vm.clone(),
            config.clone(),
            memory_manager.clone(),
            cpu_manager.clone(),
            exit_evt.try_clone().map_err(Error::EventFdClone)?,
            dynamic,
        )
        .map_err(Error::DeviceManager)?;

        device_manager
            .lock()
            .unwrap()
            .create_devices(
                serial_pty,
                original_termios,
            )
            .map_err(Error::DeviceManager)?;

        #[cfg(feature = "tdx")]
        let kernel = config
            .lock()
            .unwrap()
            .payload
            .as_ref()
            .map(|p| p.kernel.as_ref().map(File::open))
            .unwrap_or_default()
            .transpose()
            .map_err(Error::KernelFile)?;

        let initramfs = config
            .lock()
            .unwrap()
            .payload
            .as_ref()
            .map(|p| p.initramfs.as_ref().map(File::open))
            .unwrap_or_default()
            .transpose()
            .map_err(Error::InitramfsFile)?;

        let vm_state = VmState::Created;

        Ok(Vm {
            #[cfg(feature = "tdx")]
            kernel,
            initramfs,
            device_manager,
            config,
            threads: Vec::with_capacity(1),
            state: RwLock::new(vm_state),
            cpu_manager,
            memory_manager,
            stop_on_boot,
            load_payload_handle,
        })
    }

    #[allow(clippy::too_many_arguments)]
    pub fn new(
        vm_config: Arc<Mutex<VmConfig>>,
        exit_evt: EventFd,
        reset_evt: EventFd,
        #[cfg(feature = "guest_debug")] vm_debug_evt: EventFd,
        hypervisor: Arc<dyn hypervisor::Hypervisor>,
        serial_pty: Option<PtyPair>,
        original_termios: Arc<Mutex<Option<termios>>>,
    ) -> Result<Self> {
        let vm = Self::create_hypervisor_vm(
            &hypervisor,
            #[cfg(feature = "tdx")]
            tdx_enabled,
            #[cfg(feature = "sev_snp")]
            sev_snp_enabled,
        )?;

        let phys_bits = physical_bits(&hypervisor, vm_config.lock().unwrap().cpus.max_phys_bits);

        let memory_manager = {
            #[cfg(target_arch = "x86_64")]
            let sgx_epc_config = vm_config.lock().unwrap().sgx_epc.clone();

            MemoryManager::new(
                vm.clone(),
                &vm_config.lock().unwrap().memory.clone(),
                None,
                phys_bits,
                #[cfg(feature = "tdx")]
                tdx_enabled,
                #[cfg(target_arch = "x86_64")]
                sgx_epc_config,
            )
            .map_err(Error::MemoryManager)?
        };

        Vm::new_from_memory_manager(
            vm_config,
            memory_manager,
            vm,
            exit_evt,
            reset_evt,
            #[cfg(feature = "guest_debug")]
            vm_debug_evt,
            hypervisor,
            serial_pty,
            original_termios,
        )
    }

    pub fn create_hypervisor_vm(
        hypervisor: &Arc<dyn hypervisor::Hypervisor>,
        #[cfg(feature = "tdx")] tdx_enabled: bool,
        #[cfg(feature = "sev_snp")] sev_snp_enabled: bool,
    ) -> Result<Arc<dyn hypervisor::Vm>> {
        hypervisor.check_required_extensions().unwrap();

        let vm = hypervisor.create_vm().unwrap();

        #[cfg(target_arch = "x86_64")]
        {
            vm.set_identity_map_address(KVM_IDENTITY_MAP_START.0)
                .unwrap();
            vm.set_tss_address(KVM_TSS_START.0 as usize).unwrap();
            vm.enable_split_irq().unwrap();
        }

        Ok(vm)
    }

    fn load_initramfs(&mut self, guest_mem: &GuestMemoryMmap) -> Result<arch::InitramfsConfig> {
        let initramfs = self.initramfs.as_mut().unwrap();
        let size: usize = initramfs
            .seek(SeekFrom::End(0))
            .map_err(|_| Error::InitramfsLoad)?
            .try_into()
            .unwrap();
        initramfs.rewind().map_err(|_| Error::InitramfsLoad)?;

        let address =
            arch::initramfs_load_addr(guest_mem, size).map_err(|_| Error::InitramfsLoad)?;
        let address = GuestAddress(address);

        guest_mem
            .read_volatile_from(address, initramfs, size)
            .map_err(|_| Error::InitramfsLoad)?;

        info!("Initramfs loaded: address = 0x{:x}", address.0);
        Ok(arch::InitramfsConfig { address, size })
    }

    pub fn generate_cmdline(
        payload: &PayloadConfig,
        #[cfg(target_arch = "aarch64")] device_manager: &Arc<Mutex<DeviceManager>>,
    ) -> Result<Cmdline> {
        let mut cmdline = Cmdline::new(arch::CMDLINE_MAX_SIZE).map_err(Error::CmdLineCreate)?;
        if let Some(s) = payload.cmdline.as_ref() {
            cmdline.insert_str(s).map_err(Error::CmdLineInsertStr)?;
        }

        #[cfg(target_arch = "aarch64")]
        for entry in device_manager.lock().unwrap().cmdline_additions() {
            cmdline.insert_str(entry).map_err(Error::CmdLineInsertStr)?;
        }
        Ok(cmdline)
    }

    #[cfg(target_arch = "aarch64")]
    fn load_kernel(
        firmware: Option<File>,
        kernel: Option<File>,
        memory_manager: Arc<Mutex<MemoryManager>>,
    ) -> Result<EntryPoint> {
        let guest_memory = memory_manager.lock().as_ref().unwrap().guest_memory();
        let mem = guest_memory.memory();
        let entry_addr = match (firmware, kernel) {
            (None, Some(mut kernel)) => {
                match linux_loader::loader::pe::PE::load(
                    mem.deref(),
                    Some(arch::layout::KERNEL_START),
                    &mut kernel,
                    None,
                ) {
                    Ok(entry_addr) => entry_addr.kernel_load,
                    // Try to load the binary as kernel PE file at first.
                    // If failed, retry to load it as UEFI binary.
                    // As the UEFI binary is formatless, it must be the last option to try.
                    Err(linux_loader::loader::Error::Pe(InvalidImageMagicNumber)) => {
                        arch::layout::UEFI_START
                    }
                    Err(e) => {
                        return Err(Error::KernelLoad(e));
                    }
                }
            }
            _ => return Err(Error::InvalidPayload),
        };

        Ok(EntryPoint { entry_addr })
    }

    #[cfg(target_arch = "x86_64")]
    fn load_kernel(
        mut kernel: File,
        cmdline: Option<Cmdline>,
        memory_manager: Arc<Mutex<MemoryManager>>,
    ) -> Result<EntryPoint> {
        info!("Loading kernel");

        let mem = {
            let guest_memory = memory_manager.lock().as_ref().unwrap().guest_memory();
            guest_memory.memory()
        };
        let entry_addr = linux_loader::loader::elf::Elf::load(
            mem.deref(),
            None,
            &mut kernel,
            Some(arch::layout::HIGH_RAM_START),
        )
        .map_err(Error::KernelLoad)?;

        if let Some(cmdline) = cmdline {
            linux_loader::loader::load_cmdline(mem.deref(), arch::layout::CMDLINE_START, &cmdline)
                .map_err(Error::LoadCmdLine)?;
        }

        if let PvhEntryPresent(entry_addr) = entry_addr.pvh_boot_cap {
            // Use the PVH kernel entry point to boot the guest
            info!("Kernel loaded: entry_addr = 0x{:x}", entry_addr.0);
            Ok(EntryPoint { entry_addr })
        } else {
            Err(Error::KernelMissingPvhHeader)
        }
    }

    #[cfg(target_arch = "x86_64")]
    fn load_payload(
        payload: &PayloadConfig,
        memory_manager: Arc<Mutex<MemoryManager>>,
        #[cfg(feature = "igvm")] cpu_manager: Arc<Mutex<cpu::CpuManager>>,
    ) -> Result<EntryPoint> {
        #[cfg(feature = "igvm")]
        if let Some(_igvm_file) = &payload.igvm {
            let igvm = File::open(_igvm_file).map_err(Error::IgvmFile)?;
            return Self::load_igvm(igvm, memory_manager, cpu_manager);
        }
        match (
            &payload.firmware,
            &payload.kernel,
            &payload.initramfs,
            &payload.cmdline,
        ) {
            (Some(firmware), None, None, None) => {
                let firmware = File::open(firmware).map_err(Error::FirmwareFile)?;
                Self::load_kernel(firmware, None, memory_manager)
            }
            (None, Some(kernel), _, _) => {
                let kernel = File::open(kernel).map_err(Error::KernelFile)?;
                let cmdline = Self::generate_cmdline(payload)?;
                Self::load_kernel(kernel, Some(cmdline), memory_manager)
            }
            _ => Err(Error::InvalidPayload),
        }
    }

    #[cfg(target_arch = "aarch64")]
    fn load_payload(
        payload: &PayloadConfig,
        memory_manager: Arc<Mutex<MemoryManager>>,
    ) -> Result<EntryPoint> {
        match (&payload.firmware, &payload.kernel) {
            (Some(firmware), None) => {
                let firmware = File::open(firmware).map_err(Error::FirmwareFile)?;
                Self::load_kernel(Some(firmware), None, memory_manager)
            }
            (None, Some(kernel)) => {
                let kernel = File::open(kernel).map_err(Error::KernelFile)?;
                Self::load_kernel(None, Some(kernel), memory_manager)
            }
            _ => Err(Error::InvalidPayload),
        }
    }

    fn load_payload_async(
        memory_manager: &Arc<Mutex<MemoryManager>>,
        config: &Arc<Mutex<VmConfig>>,
        #[cfg(feature = "igvm")] cpu_manager: &Arc<Mutex<cpu::CpuManager>>,
    ) -> Result<Option<thread::JoinHandle<Result<EntryPoint>>>> {
        // Kernel with TDX is loaded in a different manner
        #[cfg(feature = "tdx")]
        if config.lock().unwrap().is_tdx_enabled() {
            return Ok(None);
        }

        config
            .lock()
            .unwrap()
            .payload
            .as_ref()
            .map(|payload| {
                let memory_manager = memory_manager.clone();
                let payload = payload.clone();
                #[cfg(feature = "igvm")]
                let cpu_manager = cpu_manager.clone();

                std::thread::Builder::new()
                    .name("payload_loader".into())
                    .spawn(move || {
                        Self::load_payload(
                            &payload,
                            memory_manager,
                            #[cfg(feature = "igvm")]
                            cpu_manager,
                        )
                    })
                    .map_err(Error::KernelLoadThreadSpawn)
            })
            .transpose()
    }

    #[cfg(target_arch = "x86_64")]
    fn configure_system(&mut self) -> Result<()> {
        info!("Configuring system");
        let mem = self.memory_manager.lock().unwrap().boot_guest_memory();

        let initramfs_config = match self.initramfs {
            Some(_) => Some(self.load_initramfs(&mem)?),
            None => None,
        };

        let boot_vcpus = self.cpu_manager.lock().unwrap().boot_vcpus();
        let sgx_epc_region = self
            .memory_manager
            .lock()
            .unwrap()
            .sgx_epc_region()
            .as_ref()
            .cloned();

        let topology = self.cpu_manager.lock().unwrap().get_vcpu_topology();

        arch::configure_system(
            &mem,
            arch::layout::CMDLINE_START,
            &initramfs_config,
            boot_vcpus,
            sgx_epc_region,
            topology,
        )
        .map_err(Error::ConfigureSystem)?;
        Ok(())
    }

    #[cfg(target_arch = "aarch64")]
    fn configure_system(&mut self) -> Result<()> {
        let cmdline = Self::generate_cmdline(
            self.config.lock().unwrap().payload.as_ref().unwrap(),
            &self.device_manager,
        )?;
        let vcpu_mpidrs = self.cpu_manager.lock().unwrap().get_mpidrs();
        let vcpu_topology = self.cpu_manager.lock().unwrap().get_vcpu_topology();
        let mem = self.memory_manager.lock().unwrap().boot_guest_memory();
        let initramfs_config = match self.initramfs {
            Some(_) => Some(self.load_initramfs(&mem)?),
            None => None,
        };

        let device_info = &self
            .device_manager
            .lock()
            .unwrap()
            .get_device_info()
            .clone();

        let vgic = self
            .device_manager
            .lock()
            .unwrap()
            .get_interrupt_controller()
            .unwrap()
            .lock()
            .unwrap()
            .get_vgic()
            .map_err(|_| {
                Error::ConfigureSystem(arch::Error::PlatformSpecific(
                    arch::aarch64::Error::SetupGic,
                ))
            })?;

        // PMU interrupt sticks to PPI, so need to be added by 16 to get real irq number.
        let pmu_supported = self
            .cpu_manager
            .lock()
            .unwrap()
            .init_pmu(arch::aarch64::fdt::AARCH64_PMU_IRQ + 16)
            .map_err(|_| {
                Error::ConfigureSystem(arch::Error::PlatformSpecific(
                    arch::aarch64::Error::VcpuInitPmu,
                ))
            })?;

        arch::configure_system(
            &mem,
            cmdline.as_cstring().unwrap().to_str().unwrap(),
            vcpu_mpidrs,
            vcpu_topology,
            device_info,
            &initramfs_config,
            &vgic,
            pmu_supported,
        )
        .map_err(Error::ConfigureSystem)?;

        Ok(())
    }

    pub fn serial_pty(&self) -> Option<PtyPair> {
        self.device_manager.lock().unwrap().serial_pty()
    }

    pub fn shutdown(&mut self) -> Result<()> {
        let mut state = self.state.try_write().map_err(|_| Error::PoisonedState)?;
        let new_state = VmState::Shutdown;

        state.valid_transition(new_state)?;

        self.cpu_manager
            .lock()
            .unwrap()
            .shutdown()
            .map_err(Error::CpuManager)?;

        // Wait for all the threads to finish
        for thread in self.threads.drain(..) {
            thread.join().map_err(Error::ThreadCleanup)?
        }
        *state = new_state;

        Ok(())
    }

    pub fn counters(&self) -> Result<HashMap<String, HashMap<&'static str, Wrapping<u64>>>> {
        Ok(self.device_manager.lock().unwrap().counters())
    }

    #[cfg(feature = "tdx")]
    fn extract_tdvf_sections(&mut self) -> Result<(Vec<TdvfSection>, bool)> {
        use arch::x86_64::tdx::*;

        let firmware_path = self
            .config
            .lock()
            .unwrap()
            .payload
            .as_ref()
            .unwrap()
            .firmware
            .clone()
            .ok_or(Error::TdxFirmwareMissing)?;
        // The TDVF file contains a table of section as well as code
        let mut firmware_file = File::open(firmware_path).map_err(Error::LoadTdvf)?;

        // For all the sections allocate some RAM backing them
        parse_tdvf_sections(&mut firmware_file).map_err(Error::ParseTdvf)
    }

    #[cfg(feature = "tdx")]
    fn hob_memory_resources(
        mut sorted_sections: Vec<TdvfSection>,
        guest_memory: &GuestMemoryMmap,
    ) -> Vec<(u64, u64, bool)> {
        let mut list = Vec::new();

        let mut current_section = sorted_sections.pop();

        // RAM regions interleaved with TDVF sections
        let mut next_start_addr = 0;
        for region in guest_memory.iter() {
            let region_start = region.start_addr().0;
            let region_end = region.last_addr().0;
            if region_start > next_start_addr {
                next_start_addr = region_start;
            }

            loop {
                let (start, size, ram) = if let Some(section) = &current_section {
                    if section.address <= next_start_addr {
                        (section.address, section.size, false)
                    } else {
                        let last_addr = std::cmp::min(section.address - 1, region_end);
                        (next_start_addr, last_addr - next_start_addr + 1, true)
                    }
                } else {
                    (next_start_addr, region_end - next_start_addr + 1, true)
                };

                list.push((start, size, ram));

                if !ram {
                    current_section = sorted_sections.pop();
                }

                next_start_addr = start + size;

                if region_start > next_start_addr {
                    next_start_addr = region_start;
                }

                if next_start_addr > region_end {
                    break;
                }
            }
        }

        // Once all the interleaved sections have been processed, let's simply
        // pull the remaining ones.
        if let Some(section) = current_section {
            list.push((section.address, section.size, false));
        }
        while let Some(section) = sorted_sections.pop() {
            list.push((section.address, section.size, false));
        }

        list
    }

    #[cfg(feature = "tdx")]
    fn populate_tdx_sections(
        &mut self,
        sections: &[TdvfSection],
        guid_found: bool,
    ) -> Result<Option<u64>> {
        use arch::x86_64::tdx::*;
        // Get the memory end *before* we start adding TDVF ram regions
        let boot_guest_memory = self
            .memory_manager
            .lock()
            .as_ref()
            .unwrap()
            .boot_guest_memory();
        for section in sections {
            // No need to allocate if the section falls within guest RAM ranges
            if boot_guest_memory.address_in_range(GuestAddress(section.address)) {
                info!(
                    "Not allocating TDVF Section: {:x?} since it is already part of guest RAM",
                    section
                );
                continue;
            }

            info!("Allocating TDVF Section: {:x?}", section);
            self.memory_manager
                .lock()
                .unwrap()
                .add_ram_region(GuestAddress(section.address), section.size as usize)
                .map_err(Error::AllocatingTdvfMemory)?;
        }

        // The TDVF file contains a table of section as well as code
        let firmware_path = self
            .config
            .lock()
            .unwrap()
            .payload
            .as_ref()
            .unwrap()
            .firmware
            .clone()
            .ok_or(Error::TdxFirmwareMissing)?;
        let mut firmware_file = File::open(firmware_path).map_err(Error::LoadTdvf)?;

        // The guest memory at this point now has all the required regions so it
        // is safe to copy from the TDVF file into it.
        let guest_memory = self.memory_manager.lock().as_ref().unwrap().guest_memory();
        let mem = guest_memory.memory();
        let mut payload_info = None;
        let mut hob_offset = None;
        for section in sections {
            info!("Populating TDVF Section: {:x?}", section);
            match section.r#type {
                TdvfSectionType::Bfv | TdvfSectionType::Cfv => {
                    info!("Copying section to guest memory");
                    firmware_file
                        .seek(SeekFrom::Start(section.data_offset as u64))
                        .map_err(Error::LoadTdvf)?;
                    mem.read_volatile_from(
                        GuestAddress(section.address),
                        &mut firmware_file,
                        section.data_size as usize,
                    )
                    .unwrap();
                }
                TdvfSectionType::TdHob => {
                    hob_offset = Some(section.address);
                }
                TdvfSectionType::Payload => {
                    info!("Copying payload to guest memory");
                    if let Some(payload_file) = self.kernel.as_mut() {
                        let payload_size = payload_file
                            .seek(SeekFrom::End(0))
                            .map_err(Error::LoadPayload)?;

                        payload_file
                            .seek(SeekFrom::Start(0x1f1))
                            .map_err(Error::LoadPayload)?;

                        let mut payload_header = linux_loader::bootparam::setup_header::default();
                        payload_file
                            .read_volatile(&mut payload_header.as_bytes())
                            .unwrap();

                        if payload_header.header != 0x5372_6448 {
                            return Err(Error::InvalidPayloadType);
                        }

                        if (payload_header.version < 0x0200)
                            || ((payload_header.loadflags & 0x1) == 0x0)
                        {
                            return Err(Error::InvalidPayloadType);
                        }

                        payload_file.rewind().map_err(Error::LoadPayload)?;
                        mem.read_volatile_from(
                            GuestAddress(section.address),
                            payload_file,
                            payload_size as usize,
                        )
                        .unwrap();

                        // Create the payload info that will be inserted into
                        // the HOB.
                        payload_info = Some(PayloadInfo {
                            image_type: PayloadImageType::BzImage,
                            entry_point: section.address,
                        });
                    }
                }
                TdvfSectionType::PayloadParam => {
                    info!("Copying payload parameters to guest memory");
                    let cmdline = Self::generate_cmdline(
                        self.config.lock().unwrap().payload.as_ref().unwrap(),
                    )?;
                    mem.write_slice(
                        cmdline.as_cstring().unwrap().as_bytes_with_nul(),
                        GuestAddress(section.address),
                    )
                    .unwrap();
                }
                _ => {}
            }
        }

        // Generate HOB
        let mut hob = TdHob::start(hob_offset.unwrap());

        let mut sorted_sections = sections.to_vec();
        sorted_sections.retain(|section| matches!(section.r#type, TdvfSectionType::TempMem));

        sorted_sections.sort_by_key(|section| section.address);
        sorted_sections.reverse();

        for (start, size, ram) in Vm::hob_memory_resources(sorted_sections, &boot_guest_memory) {
            hob.add_memory_resource(&mem, start, size, ram, guid_found)
                .map_err(Error::PopulateHob)?;
        }

        // MMIO regions
        hob.add_mmio_resource(
            &mem,
            arch::layout::MEM_32BIT_DEVICES_START.raw_value(),
            arch::layout::APIC_START.raw_value()
                - arch::layout::MEM_32BIT_DEVICES_START.raw_value(),
        )
        .map_err(Error::PopulateHob)?;
        let start_of_device_area = self
            .memory_manager
            .lock()
            .unwrap()
            .start_of_device_area()
            .raw_value();
        let end_of_device_area = self
            .memory_manager
            .lock()
            .unwrap()
            .end_of_device_area()
            .raw_value();
        hob.add_mmio_resource(
            &mem,
            start_of_device_area,
            end_of_device_area - start_of_device_area,
        )
        .map_err(Error::PopulateHob)?;

        // If a payload info has been created, let's insert it into the HOB.
        if let Some(payload_info) = payload_info {
            hob.add_payload(&mem, payload_info)
                .map_err(Error::PopulateHob)?;
        }

        hob.finish(&mem).map_err(Error::PopulateHob)?;

        Ok(hob_offset)
    }

    #[cfg(feature = "tdx")]
    fn init_tdx_memory(&mut self, sections: &[TdvfSection]) -> Result<()> {
        let guest_memory = self.memory_manager.lock().as_ref().unwrap().guest_memory();
        let mem = guest_memory.memory();

        for section in sections {
            self.vm
                .tdx_init_memory_region(
                    mem.get_host_address(GuestAddress(section.address)).unwrap() as u64,
                    section.address,
                    section.size,
                    /* TDVF_SECTION_ATTRIBUTES_EXTENDMR */
                    section.attributes == 1,
                )
                .map_err(Error::InitializeTdxMemoryRegion)?;
        }

        Ok(())
    }

    fn entry_point(&mut self) -> Result<Option<EntryPoint>> {
        self.load_payload_handle
            .take()
            .map(|handle| handle.join().map_err(Error::KernelLoadThreadJoin)?)
            .transpose()
    }

    pub fn boot(&mut self) -> Result<()> {
        info!("Booting VM");
        let current_state = self.get_state()?;

        let new_state = if self.stop_on_boot {
            VmState::BreakPoint
        } else {
            VmState::Running
        };
        current_state.valid_transition(new_state)?;

        // Load kernel synchronously or if asynchronous then wait for load to
        // finish.
        let entry_point = self.entry_point()?;

        #[cfg(feature = "tdx")]
        let tdx_enabled = self.config.lock().unwrap().is_tdx_enabled();

        // Configure the vcpus that have been created
        let vcpus = self.cpu_manager.lock().unwrap().vcpus();
        for vcpu in vcpus {
            let guest_memory = &self.memory_manager.lock().as_ref().unwrap().guest_memory();
            let boot_setup = entry_point.map(|e| (e, guest_memory));
            self.cpu_manager
                .lock()
                .unwrap()
                .configure_vcpu(vcpu, boot_setup)
                .map_err(Error::CpuManager)?;
        }

        #[cfg(feature = "tdx")]
        let (sections, guid_found) = if tdx_enabled {
            self.extract_tdvf_sections()?
        } else {
            (Vec::new(), false)
        };

        // Configuring the TDX regions requires that the vCPUs are created.
        #[cfg(feature = "tdx")]
        let hob_address = if tdx_enabled {
            // TDX sections are written to memory.
            self.populate_tdx_sections(&sections, guid_found)?
        } else {
            None
        };

        // Configure shared state based on loaded kernel
        entry_point
            .map(|_| {
                self.configure_system()
            })
            .transpose()?;

        #[cfg(target_arch = "x86_64")]
        // Note: For x86, always call this function before invoking start boot vcpus.
        // Otherwise guest would fail to boot because we haven't created the
        // userspace mappings to update the hypervisor about the memory mappings.
        // These mappings must be created before we start the vCPU threads for
        // the very first time.
        self.memory_manager
            .lock()
            .unwrap()
            .allocate_address_space()
            .map_err(Error::MemoryManager)?;

        #[cfg(feature = "tdx")]
        if let Some(hob_address) = hob_address {
            // With the HOB address extracted the vCPUs can have
            // their TDX state configured.
            self.cpu_manager
                .lock()
                .unwrap()
                .initialize_tdx(hob_address)
                .map_err(Error::CpuManager)?;
            // Let the hypervisor know which memory ranges are shared with the
            // guest. This prevents the guest from ignoring/discarding memory
            // regions provided by the host.
            self.init_tdx_memory(&sections)?;
            // With TDX memory and CPU state configured TDX setup is complete
            self.vm.tdx_finalize().map_err(Error::FinalizeTdx)?;
        }

        self.cpu_manager
            .lock()
            .unwrap()
            .start_boot_vcpus(new_state == VmState::BreakPoint)
            .map_err(Error::CpuManager)?;

        let mut state = self.state.try_write().map_err(|_| Error::PoisonedState)?;
        *state = new_state;
        Ok(())
    }

    pub fn restore(&mut self) -> Result<()> {

        #[cfg(target_arch = "x86_64")]
        // Note: For x86, always call this function before invoking start boot vcpus.
        // Otherwise guest would fail to boot because we haven't created the
        // userspace mappings to update the hypervisor about the memory mappings.
        // These mappings must be created before we start the vCPU threads for
        // the very first time for the restored VM.
        self.memory_manager
            .lock()
            .unwrap()
            .allocate_address_space()
            .map_err(Error::MemoryManager)?;

        // Now we can start all vCPUs from here.
        self.cpu_manager
            .lock()
            .unwrap()
            .start_restored_vcpus()
            .map_err(Error::CpuManager)?;

        Ok(())
    }

    /// Gets a thread-safe reference counted pointer to the VM configuration.
    pub fn get_config(&self) -> Arc<Mutex<VmConfig>> {
        Arc::clone(&self.config)
    }

    /// Get the VM state. Returns an error if the state is poisoned.
    pub fn get_state(&self) -> Result<VmState> {
        self.state
            .try_read()
            .map_err(|_| Error::PoisonedState)
            .map(|state| *state)
    }

    pub fn device_tree(&self) -> Arc<Mutex<DeviceTree>> {
        self.device_manager.lock().unwrap().device_tree()
    }
}

#[derive(Serialize, Deserialize)]
pub struct VmSnapshot {
    #[cfg(all(feature = "kvm", target_arch = "x86_64"))]
    pub clock: Option<hypervisor::ClockData>,
    #[cfg(all(feature = "kvm", target_arch = "x86_64"))]
    pub common_cpuid: Vec<hypervisor::arch::x86::CpuIdEntry>,
}

pub const VM_SNAPSHOT_ID: &str = "vm";

#[cfg(feature = "guest_debug")]
impl Debuggable for Vm {
    fn set_guest_debug(
        &self,
        cpu_id: usize,
        addrs: &[GuestAddress],
        singlestep: bool,
    ) -> std::result::Result<(), DebuggableError> {
        self.cpu_manager
            .lock()
            .unwrap()
            .set_guest_debug(cpu_id, addrs, singlestep)
    }

    fn debug_pause(&mut self) -> std::result::Result<(), DebuggableError> {
        if *self.state.read().unwrap() == VmState::Running {
            self.pause().map_err(DebuggableError::Pause)?;
        }

        let mut state = self
            .state
            .try_write()
            .map_err(|_| DebuggableError::PoisonedState)?;
        *state = VmState::BreakPoint;
        Ok(())
    }

    fn debug_resume(&mut self) -> std::result::Result<(), DebuggableError> {
        if *self.state.read().unwrap() == VmState::BreakPoint {
            self.resume().map_err(DebuggableError::Pause)?;
        }

        Ok(())
    }

    fn read_regs(&self, cpu_id: usize) -> std::result::Result<CoreRegs, DebuggableError> {
        self.cpu_manager.lock().unwrap().read_regs(cpu_id)
    }

    fn write_regs(
        &self,
        cpu_id: usize,
        regs: &CoreRegs,
    ) -> std::result::Result<(), DebuggableError> {
        self.cpu_manager.lock().unwrap().write_regs(cpu_id, regs)
    }

    fn read_mem(
        &self,
        guest_memory: &GuestMemoryAtomic<GuestMemoryMmap>,
        cpu_id: usize,
        vaddr: GuestAddress,
        len: usize,
    ) -> std::result::Result<Vec<u8>, DebuggableError> {
        self.cpu_manager
            .lock()
            .unwrap()
            .read_mem(guest_memory, cpu_id, vaddr, len)
    }

    fn write_mem(
        &self,
        guest_memory: &GuestMemoryAtomic<GuestMemoryMmap>,
        cpu_id: usize,
        vaddr: &GuestAddress,
        data: &[u8],
    ) -> std::result::Result<(), DebuggableError> {
        self.cpu_manager
            .lock()
            .unwrap()
            .write_mem(guest_memory, cpu_id, vaddr, data)
    }

    fn active_vcpus(&self) -> usize {
        let active_vcpus = self.cpu_manager.lock().unwrap().active_vcpus();
        if active_vcpus > 0 {
            active_vcpus
        } else {
            // The VM is not booted yet. Report boot_vcpus() instead.
            self.cpu_manager.lock().unwrap().boot_vcpus() as usize
        }
    }
}

#[cfg(feature = "guest_debug")]
pub const UINT16_MAX: u32 = 65535;

#[cfg(all(target_arch = "x86_64", feature = "guest_debug"))]
impl Elf64Writable for Vm {}

#[cfg(all(target_arch = "x86_64", feature = "guest_debug"))]
impl GuestDebuggable for Vm {
    fn coredump(&mut self, destination_url: &str) -> std::result::Result<(), GuestDebuggableError> {

        let mut resume = false;

        #[cfg(feature = "tdx")]
        {
            if let Some(ref platform) = self.config.lock().unwrap().platform {
                if platform.tdx {
                    return Err(GuestDebuggableError::Coredump(anyhow!(
                        "Coredump not possible with TDX VM"
                    )));
                }
            }
        }

        match self.get_state().unwrap() {
            VmState::Running => {
                self.pause().map_err(GuestDebuggableError::Pause)?;
                resume = true;
            }
            VmState::Paused => {}
            _ => {
                return Err(GuestDebuggableError::Coredump(anyhow!(
                    "Trying to coredump while VM is not running or paused"
                )));
            }
        }

        let coredump_state = self.get_dump_state(destination_url)?;

        self.write_header(&coredump_state)?;
        self.write_note(&coredump_state)?;
        self.write_loads(&coredump_state)?;

        self.cpu_manager
            .lock()
            .unwrap()
            .cpu_write_elf64_note(&coredump_state)?;
        self.cpu_manager
            .lock()
            .unwrap()
            .cpu_write_vmm_note(&coredump_state)?;

        self.memory_manager
            .lock()
            .unwrap()
            .coredump_iterate_save_mem(&coredump_state)?;

        if resume {
            self.resume().map_err(GuestDebuggableError::Resume)?;
        }

        Ok(())
    }
}

#[cfg(all(feature = "kvm", target_arch = "x86_64"))]
#[cfg(test)]
mod tests {
    use super::*;

    fn test_vm_state_transitions(state: VmState) {
        match state {
            VmState::Created => {
                // Check the transitions from Created
                assert!(state.valid_transition(VmState::Created).is_err());
                assert!(state.valid_transition(VmState::Running).is_ok());
                assert!(state.valid_transition(VmState::Shutdown).is_ok());
                assert!(state.valid_transition(VmState::Paused).is_ok());
                assert!(state.valid_transition(VmState::BreakPoint).is_ok());
            }
            VmState::Running => {
                // Check the transitions from Running
                assert!(state.valid_transition(VmState::Created).is_err());
                assert!(state.valid_transition(VmState::Running).is_err());
                assert!(state.valid_transition(VmState::Shutdown).is_ok());
                assert!(state.valid_transition(VmState::Paused).is_ok());
                assert!(state.valid_transition(VmState::BreakPoint).is_ok());
            }
            VmState::Shutdown => {
                // Check the transitions from Shutdown
                assert!(state.valid_transition(VmState::Created).is_err());
                assert!(state.valid_transition(VmState::Running).is_ok());
                assert!(state.valid_transition(VmState::Shutdown).is_err());
                assert!(state.valid_transition(VmState::Paused).is_err());
                assert!(state.valid_transition(VmState::BreakPoint).is_err());
            }
            VmState::Paused => {
                // Check the transitions from Paused
                assert!(state.valid_transition(VmState::Created).is_err());
                assert!(state.valid_transition(VmState::Running).is_ok());
                assert!(state.valid_transition(VmState::Shutdown).is_ok());
                assert!(state.valid_transition(VmState::Paused).is_err());
                assert!(state.valid_transition(VmState::BreakPoint).is_err());
            }
            VmState::BreakPoint => {
                // Check the transitions from Breakpoint
                assert!(state.valid_transition(VmState::Created).is_ok());
                assert!(state.valid_transition(VmState::Running).is_ok());
                assert!(state.valid_transition(VmState::Shutdown).is_err());
                assert!(state.valid_transition(VmState::Paused).is_err());
                assert!(state.valid_transition(VmState::BreakPoint).is_err());
            }
        }
    }

    #[test]
    fn test_vm_created_transitions() {
        test_vm_state_transitions(VmState::Created);
    }

    #[test]
    fn test_vm_running_transitions() {
        test_vm_state_transitions(VmState::Running);
    }

    #[test]
    fn test_vm_shutdown_transitions() {
        test_vm_state_transitions(VmState::Shutdown);
    }

    #[test]
    fn test_vm_paused_transitions() {
        test_vm_state_transitions(VmState::Paused);
    }

    #[cfg(feature = "tdx")]
    #[test]
    fn test_hob_memory_resources() {
        // Case 1: Two TDVF sections in the middle of the RAM
        let sections = vec![
            TdvfSection {
                address: 0xc000,
                size: 0x1000,
                ..Default::default()
            },
            TdvfSection {
                address: 0x1000,
                size: 0x4000,
                ..Default::default()
            },
        ];
        let guest_ranges: Vec<(GuestAddress, usize)> = vec![(GuestAddress(0), 0x1000_0000)];
        let expected = vec![
            (0, 0x1000, true),
            (0x1000, 0x4000, false),
            (0x5000, 0x7000, true),
            (0xc000, 0x1000, false),
            (0xd000, 0x0fff_3000, true),
        ];
        assert_eq!(
            expected,
            Vm::hob_memory_resources(
                sections,
                &GuestMemoryMmap::from_ranges(&guest_ranges).unwrap()
            )
        );

        // Case 2: Two TDVF sections with no conflict with the RAM
        let sections = vec![
            TdvfSection {
                address: 0x1000_1000,
                size: 0x1000,
                ..Default::default()
            },
            TdvfSection {
                address: 0,
                size: 0x1000,
                ..Default::default()
            },
        ];
        let guest_ranges: Vec<(GuestAddress, usize)> = vec![(GuestAddress(0x1000), 0x1000_0000)];
        let expected = vec![
            (0, 0x1000, false),
            (0x1000, 0x1000_0000, true),
            (0x1000_1000, 0x1000, false),
        ];
        assert_eq!(
            expected,
            Vm::hob_memory_resources(
                sections,
                &GuestMemoryMmap::from_ranges(&guest_ranges).unwrap()
            )
        );

        // Case 3: Two TDVF sections with partial conflicts with the RAM
        let sections = vec![
            TdvfSection {
                address: 0x1000_0000,
                size: 0x2000,
                ..Default::default()
            },
            TdvfSection {
                address: 0,
                size: 0x2000,
                ..Default::default()
            },
        ];
        let guest_ranges: Vec<(GuestAddress, usize)> = vec![(GuestAddress(0x1000), 0x1000_0000)];
        let expected = vec![
            (0, 0x2000, false),
            (0x2000, 0x0fff_e000, true),
            (0x1000_0000, 0x2000, false),
        ];
        assert_eq!(
            expected,
            Vm::hob_memory_resources(
                sections,
                &GuestMemoryMmap::from_ranges(&guest_ranges).unwrap()
            )
        );

        // Case 4: Two TDVF sections with no conflict before the RAM and two
        // more additional sections with no conflict after the RAM.
        let sections = vec![
            TdvfSection {
                address: 0x2000_1000,
                size: 0x1000,
                ..Default::default()
            },
            TdvfSection {
                address: 0x2000_0000,
                size: 0x1000,
                ..Default::default()
            },
            TdvfSection {
                address: 0x1000,
                size: 0x1000,
                ..Default::default()
            },
            TdvfSection {
                address: 0,
                size: 0x1000,
                ..Default::default()
            },
        ];
        let guest_ranges: Vec<(GuestAddress, usize)> = vec![(GuestAddress(0x4000), 0x1000_0000)];
        let expected = vec![
            (0, 0x1000, false),
            (0x1000, 0x1000, false),
            (0x4000, 0x1000_0000, true),
            (0x2000_0000, 0x1000, false),
            (0x2000_1000, 0x1000, false),
        ];
        assert_eq!(
            expected,
            Vm::hob_memory_resources(
                sections,
                &GuestMemoryMmap::from_ranges(&guest_ranges).unwrap()
            )
        );

        // Case 5: One TDVF section overriding the entire RAM
        let sections = vec![TdvfSection {
            address: 0,
            size: 0x2000_0000,
            ..Default::default()
        }];
        let guest_ranges: Vec<(GuestAddress, usize)> = vec![(GuestAddress(0x1000), 0x1000_0000)];
        let expected = vec![(0, 0x2000_0000, false)];
        assert_eq!(
            expected,
            Vm::hob_memory_resources(
                sections,
                &GuestMemoryMmap::from_ranges(&guest_ranges).unwrap()
            )
        );

        // Case 6: Two TDVF sections with no conflict with 2 RAM regions
        let sections = vec![
            TdvfSection {
                address: 0x1000_2000,
                size: 0x2000,
                ..Default::default()
            },
            TdvfSection {
                address: 0,
                size: 0x2000,
                ..Default::default()
            },
        ];
        let guest_ranges: Vec<(GuestAddress, usize)> = vec![
            (GuestAddress(0x2000), 0x1000_0000),
            (GuestAddress(0x1000_4000), 0x1000_0000),
        ];
        let expected = vec![
            (0, 0x2000, false),
            (0x2000, 0x1000_0000, true),
            (0x1000_2000, 0x2000, false),
            (0x1000_4000, 0x1000_0000, true),
        ];
        assert_eq!(
            expected,
            Vm::hob_memory_resources(
                sections,
                &GuestMemoryMmap::from_ranges(&guest_ranges).unwrap()
            )
        );

        // Case 7: Two TDVF sections with partial conflicts with 2 RAM regions
        let sections = vec![
            TdvfSection {
                address: 0x1000_0000,
                size: 0x4000,
                ..Default::default()
            },
            TdvfSection {
                address: 0,
                size: 0x4000,
                ..Default::default()
            },
        ];
        let guest_ranges: Vec<(GuestAddress, usize)> = vec![
            (GuestAddress(0x1000), 0x1000_0000),
            (GuestAddress(0x1000_3000), 0x1000_0000),
        ];
        let expected = vec![
            (0, 0x4000, false),
            (0x4000, 0x0fff_c000, true),
            (0x1000_0000, 0x4000, false),
            (0x1000_4000, 0x0fff_f000, true),
        ];
        assert_eq!(
            expected,
            Vm::hob_memory_resources(
                sections,
                &GuestMemoryMmap::from_ranges(&guest_ranges).unwrap()
            )
        );
    }
}

#[cfg(target_arch = "aarch64")]
#[cfg(test)]
mod tests {
    use super::*;
    use crate::GuestMemoryMmap;
    use arch::aarch64::fdt::create_fdt;
    use arch::aarch64::layout;
    use arch::{DeviceType, MmioDeviceInfo};
    use devices::gic::Gic;

    const LEN: u64 = 4096;

    #[test]
    fn test_create_fdt_with_devices() {
        let regions = vec![(layout::RAM_START, (layout::FDT_MAX_SIZE + 0x1000) as usize)];
        let mem = GuestMemoryMmap::from_ranges(&regions).expect("Cannot initialize memory");

        let dev_info: HashMap<(DeviceType, std::string::String), MmioDeviceInfo> = [
            (
                (DeviceType::Serial, DeviceType::Serial.to_string()),
                MmioDeviceInfo {
                    addr: 0x00,
                    len: LEN,
                    irq: 33,
                },
            ),
            (
                (DeviceType::Virtio(1), "virtio".to_string()),
                MmioDeviceInfo {
                    addr: LEN,
                    len: LEN,
                    irq: 34,
                },
            ),
            (
                (DeviceType::Rtc, "rtc".to_string()),
                MmioDeviceInfo {
                    addr: 2 * LEN,
                    len: LEN,
                    irq: 35,
                },
            ),
        ]
        .iter()
        .cloned()
        .collect();

        let hv = hypervisor::new().unwrap();
        let vm = hv.create_vm().unwrap();
        let gic = vm
            .create_vgic(Gic::create_default_config(1))
            .expect("Cannot create gic");
        assert!(create_fdt(
            &mem,
            "console=tty0",
            vec![0],
            Some((0, 0, 0)),
            &dev_info,
            &gic,
            &None,
            &Vec::new(),
            &BTreeMap::new(),
            None,
            true,
        )
        .is_ok())
    }
}

#[cfg(all(feature = "kvm", target_arch = "x86_64"))]
#[test]
pub fn test_vm() {
    use hypervisor::VmExit;
    use vm_memory::{Address, GuestMemory, GuestMemoryRegion};
    // This example based on https://lwn.net/Articles/658511/
    let code = [
        0xba, 0xf8, 0x03, /* mov $0x3f8, %dx */
        0x00, 0xd8, /* add %bl, %al */
        0x04, b'0', /* add $'0', %al */
        0xee, /* out %al, (%dx) */
        0xb0, b'\n', /* mov $'\n', %al */
        0xee,  /* out %al, (%dx) */
        0xf4,  /* hlt */
    ];

    let mem_size = 0x1000;
    let load_addr = GuestAddress(0x1000);
    let mem = GuestMemoryMmap::from_ranges(&[(load_addr, mem_size)]).unwrap();

    let hv = hypervisor::new().unwrap();
    let vm = hv.create_vm().expect("new VM creation failed");

    for (index, region) in mem.iter().enumerate() {
        let mem_region = vm.make_user_memory_region(
            index as u32,
            region.start_addr().raw_value(),
            region.len(),
            region.as_ptr() as u64,
            false,
            false,
        );

        vm.create_user_memory_region(mem_region)
            .expect("Cannot configure guest memory");
    }
    mem.write_slice(&code, load_addr)
        .expect("Writing code to memory failed");

    let vcpu = vm.create_vcpu(0, None).expect("new Vcpu failed");

    let mut vcpu_sregs = vcpu.get_sregs().expect("get sregs failed");
    vcpu_sregs.cs.base = 0;
    vcpu_sregs.cs.selector = 0;
    vcpu.set_sregs(&vcpu_sregs).expect("set sregs failed");

    let mut vcpu_regs = vcpu.get_regs().expect("get regs failed");
    vcpu_regs.rip = 0x1000;
    vcpu_regs.rax = 2;
    vcpu_regs.rbx = 3;
    vcpu_regs.rflags = 2;
    vcpu.set_regs(&vcpu_regs).expect("set regs failed");

    loop {
        match vcpu.run().expect("run failed") {
            VmExit::IoOut(addr, data) => {
                println!(
                    "IO out -- addr: {:#x} data [{:?}]",
                    addr,
                    str::from_utf8(data).unwrap()
                );
            }
            VmExit::Reset => {
                println!("HLT");
                break;
            }
            r => panic!("unexpected exit reason: {r:?}"),
        }
    }
}
