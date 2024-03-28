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

pub const ACPI_X2APIC_PROCESSOR: u8 = 9;
pub const ACPI_APIC_IO: u8 = 1;
pub const ACPI_APIC_XRUPT_OVERRIDE: u8 = 2;
pub const ACPI_APIC_GENERIC_REDISTRIBUTOR: u8 = 14;
pub const ACPI_APIC_GENERIC_TRANSLATOR: u8 = 15;
pub const ACPI_APIC_GENERIC_CPU_INTERFACE: u8 = 11;
pub const ACPI_APIC_GENERIC_DISTRIBUTOR: u8 = 12;
use crate::config::CpusConfig;
#[cfg(all(target_arch = "x86_64", feature = "guest_debug"))]
use crate::coredump::{
    CpuElf64Writable, CpuSegment, CpuState as DumpCpusState, DumpState, Elf64Writable,
    GuestDebuggableError, NoteDescType, X86_64ElfPrStatus, X86_64UserRegs, COREDUMP_NAME_SIZE,
    NT_PRSTATUS,
};
#[cfg(feature = "guest_debug")]
use crate::gdb::{get_raw_tid, Debuggable, DebuggableError};
#[cfg(target_arch = "x86_64")]
use crate::memory_manager::MemoryManager;
#[cfg(target_arch = "x86_64")]
use crate::vm::physical_bits;
use crate::GuestMemoryMmap;
use anyhow::anyhow;
#[cfg(all(target_arch = "aarch64", feature = "guest_debug"))]
use arch::aarch64::regs;
use arch::EntryPoint;
use devices::interrupt_controller::InterruptController;
#[cfg(all(target_arch = "aarch64", feature = "guest_debug"))]
use gdbstub_arch::aarch64::reg::AArch64CoreRegs as CoreRegs;
#[cfg(all(target_arch = "x86_64", feature = "guest_debug"))]
use gdbstub_arch::x86::reg::{X86SegmentRegs, X86_64CoreRegs as CoreRegs};
#[cfg(all(target_arch = "aarch64", feature = "guest_debug"))]
use hypervisor::aarch64::StandardRegisters;
#[cfg(all(target_arch = "x86_64", feature = "guest_debug"))]
use hypervisor::arch::x86::msr_index;
#[cfg(target_arch = "x86_64")]
use hypervisor::arch::x86::CpuIdEntry;
#[cfg(all(target_arch = "x86_64", feature = "guest_debug"))]
use hypervisor::arch::x86::MsrEntry;
#[cfg(all(target_arch = "x86_64", feature = "guest_debug"))]
use hypervisor::arch::x86::{SpecialRegisters, StandardRegisters};
#[cfg(target_arch = "aarch64")]
use hypervisor::kvm::kvm_bindings;
#[cfg(all(target_arch = "aarch64", feature = "kvm"))]
use hypervisor::kvm::kvm_ioctls::Cap;
#[cfg(feature = "tdx")]
use hypervisor::kvm::{TdxExitDetails, TdxExitStatus};
#[cfg(target_arch = "x86_64")]
use hypervisor::CpuVendor;
use hypervisor::{HypervisorCpuError, HypervisorType, VmExit, VmOps};
use libc::{c_void, siginfo_t};
#[cfg(all(target_arch = "x86_64", feature = "guest_debug"))]
use linux_loader::elf::Elf64_Nhdr;
use std::collections::BTreeMap;
#[cfg(all(target_arch = "x86_64", feature = "guest_debug"))]
use std::io::Write;
#[cfg(all(target_arch = "x86_64", feature = "guest_debug"))]
use std::mem::size_of;
use std::os::unix::thread::JoinHandleExt;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Barrier, Mutex};
use std::{cmp, io, result, thread};
use thiserror::Error;
use vm_device::BusDevice;
#[cfg(all(target_arch = "x86_64", feature = "guest_debug"))]
use vm_memory::ByteValued;
#[cfg(feature = "guest_debug")]
use vm_memory::{Bytes, GuestAddressSpace};
use vm_memory::{GuestAddress, GuestMemoryAtomic};
use vmm_sys_util::eventfd::EventFd;
use vmm_sys_util::signal::{register_signal_handler, SIGRTMIN};
use zerocopy::AsBytes;
#[cfg(all(target_arch = "aarch64", feature = "guest_debug"))]
/// Extract the specified bits of a 64-bit integer.
/// For example, to extrace 2 bits from offset 1 (zero based) of `6u64`,
/// following expression should return 3 (`0b11`):
/// `extract_bits_64!(0b0000_0110u64, 1, 2)`
///
macro_rules! extract_bits_64 {
    ($value: tt, $offset: tt, $length: tt) => {
        ($value >> $offset) & (!0u64 >> (64 - $length))
    };
}

#[cfg(all(target_arch = "aarch64", feature = "guest_debug"))]
macro_rules! extract_bits_64_without_offset {
    ($value: tt, $length: tt) => {
        $value & (!0u64 >> (64 - $length))
    };
}

pub const CPU_MANAGER_ACPI_SIZE: usize = 0xc;

#[derive(Debug, Error)]
pub enum Error {
    #[error("Error creating vCPU: {0}")]
    VcpuCreate(#[source] anyhow::Error),

    #[error("Error running bCPU: {0}")]
    VcpuRun(#[source] anyhow::Error),

    #[error("Error spawning vCPU thread: {0}")]
    VcpuSpawn(#[source] io::Error),

    #[error("Error generating common CPUID: {0}")]
    CommonCpuId(#[source] arch::Error),

    #[error("Error configuring vCPU: {0}")]
    VcpuConfiguration(#[source] arch::Error),

    #[error("Still pending removed vcpu")]
    VcpuPendingRemovedVcpu,

    #[cfg(target_arch = "aarch64")]
    #[error("Error fetching preferred target: {0}")]
    VcpuArmPreferredTarget(#[source] hypervisor::HypervisorVmError),

    #[cfg(target_arch = "aarch64")]
    #[error("Error initialising vCPU: {0}")]
    VcpuArmInit(#[source] hypervisor::HypervisorCpuError),

    #[error("Failed to join on vCPU threads: {0:?}")]
    ThreadCleanup(std::boxed::Box<dyn std::any::Any + std::marker::Send>),

    #[error("Error adding CpuManager to MMIO bus: {0}")]
    BusError(#[source] vm_device::BusError),

    #[error("Requested vCPUs exceed maximum")]
    DesiredVCpuCountExceedsMax,

    #[error("Error starting vCPU after restore: {0}")]
    StartRestoreVcpu(#[source] anyhow::Error),

    #[error("Unexpected VmExit")]
    UnexpectedVmExit,

    #[error("Failed to allocate MMIO address for CpuManager")]
    AllocateMmmioAddress,

    #[cfg(feature = "tdx")]
    #[error("Error initializing TDX: {0}")]
    InitializeTdx(#[source] hypervisor::HypervisorCpuError),

    #[cfg(target_arch = "aarch64")]
    #[error("Error initializing PMU: {0}")]
    InitPmu(#[source] hypervisor::HypervisorCpuError),

    #[cfg(feature = "guest_debug")]
    #[error("Error during CPU debug: {0}")]
    CpuDebug(#[source] hypervisor::HypervisorCpuError),

    #[cfg(feature = "guest_debug")]
    #[error("Error translating virtual address: {0}")]
    TranslateVirtualAddress(#[source] anyhow::Error),

    #[cfg(target_arch = "x86_64")]
    #[error("Error setting up AMX: {0}")]
    AmxEnable(#[source] anyhow::Error),

    #[error("Maximum number of vCPUs exceeds host limit")]
    MaximumVcpusExceeded,

    #[cfg(feature = "sev_snp")]
    #[error("Failed to set sev control register: {0}")]
    SetSevControlRegister(#[source] hypervisor::HypervisorCpuError),
}
pub type Result<T> = result::Result<T, Error>;

#[cfg(target_arch = "x86_64")]
#[allow(dead_code)]
#[repr(packed)]
#[derive(AsBytes)]
struct LocalX2Apic {
    pub r#type: u8,
    pub length: u8,
    pub _reserved: u16,
    pub apic_id: u32,
    pub flags: u32,
    pub processor_id: u32,
}

#[allow(dead_code)]
#[repr(packed)]
#[derive(Default, AsBytes)]
struct Ioapic {
    pub r#type: u8,
    pub length: u8,
    pub ioapic_id: u8,
    _reserved: u8,
    pub apic_address: u32,
    pub gsi_base: u32,
}

#[cfg(target_arch = "aarch64")]
#[allow(dead_code)]
#[repr(packed)]
#[derive(AsBytes)]
struct GicC {
    pub r#type: u8,
    pub length: u8,
    pub reserved0: u16,
    pub cpu_interface_number: u32,
    pub uid: u32,
    pub flags: u32,
    pub parking_version: u32,
    pub performance_interrupt: u32,
    pub parked_address: u64,
    pub base_address: u64,
    pub gicv_base_address: u64,
    pub gich_base_address: u64,
    pub vgic_interrupt: u32,
    pub gicr_base_address: u64,
    pub mpidr: u64,
    pub proc_power_effi_class: u8,
    pub reserved1: u8,
    pub spe_overflow_interrupt: u16,
}

#[cfg(target_arch = "aarch64")]
#[allow(dead_code)]
#[repr(packed)]
#[derive(AsBytes)]
struct GicD {
    pub r#type: u8,
    pub length: u8,
    pub reserved0: u16,
    pub gic_id: u32,
    pub base_address: u64,
    pub global_irq_base: u32,
    pub version: u8,
    pub reserved1: [u8; 3],
}

#[cfg(target_arch = "aarch64")]
#[allow(dead_code)]
#[repr(packed)]
#[derive(AsBytes)]
struct GicR {
    pub r#type: u8,
    pub length: u8,
    pub reserved: u16,
    pub base_address: u64,
    pub range_length: u32,
}

#[cfg(target_arch = "aarch64")]
#[allow(dead_code)]
#[repr(packed)]
#[derive(AsBytes)]
struct GicIts {
    pub r#type: u8,
    pub length: u8,
    pub reserved0: u16,
    pub translation_id: u32,
    pub base_address: u64,
    pub reserved1: u32,
}

#[cfg(target_arch = "aarch64")]
#[allow(dead_code)]
#[repr(packed)]
#[derive(AsBytes)]
struct ProcessorHierarchyNode {
    pub r#type: u8,
    pub length: u8,
    pub reserved: u16,
    pub flags: u32,
    pub parent: u32,
    pub acpi_processor_id: u32,
    pub num_private_resources: u32,
}

#[allow(dead_code)]
#[repr(packed)]
#[derive(Default, AsBytes)]
struct InterruptSourceOverride {
    pub r#type: u8,
    pub length: u8,
    pub bus: u8,
    pub source: u8,
    pub gsi: u32,
    pub flags: u16,
}

#[cfg(all(target_arch = "x86_64", feature = "guest_debug"))]
macro_rules! round_up {
    ($n:expr,$d:expr) => {
        (($n / ($d + 1)) + 1) * $d
    };
}

/// A wrapper around creating and using a kvm-based VCPU.
pub struct Vcpu {
    // The hypervisor abstracted CPU.
    vcpu: Arc<dyn hypervisor::Vcpu>,
    id: u8,
    #[cfg(target_arch = "aarch64")]
    mpidr: u64,
    #[cfg(target_arch = "x86_64")]
    vendor: CpuVendor,
}

impl Vcpu {
    /// Constructs a new VCPU for `vm`.
    ///
    /// # Arguments
    ///
    /// * `id` - Represents the CPU number between [0, max vcpus).
    /// * `vm` - The virtual machine this vcpu will get attached to.
    /// * `vm_ops` - Optional object for exit handling.
    /// * `cpu_vendor` - CPU vendor as reported by __cpuid(0x0)
    pub fn new(
        id: u8,
        apic_id: u8,
        vm: &Arc<dyn hypervisor::Vm>,
        vm_ops: Option<Arc<dyn VmOps>>,
        #[cfg(target_arch = "x86_64")] cpu_vendor: CpuVendor,
    ) -> Result<Self> {
        let vcpu = vm
            .create_vcpu(apic_id, vm_ops)
            .map_err(|e| Error::VcpuCreate(e.into()))?;
        // Initially the cpuid per vCPU is the one supported by this VM.
        Ok(Vcpu {
            vcpu,
            id,
            #[cfg(target_arch = "aarch64")]
            mpidr: 0,
            #[cfg(target_arch = "x86_64")]
            vendor: cpu_vendor,
        })
    }

    /// Configures a vcpu and should be called once per vcpu when created.
    ///
    /// # Arguments
    ///
    /// * `kernel_entry_point` - Kernel entry point address in guest memory and boot protocol used.
    /// * `guest_memory` - Guest memory.
    /// * `cpuid` - (x86_64) CpuId, wrapper over the `kvm_cpuid2` structure.
    pub fn configure(
        &mut self,
        #[cfg(target_arch = "aarch64")] vm: &Arc<dyn hypervisor::Vm>,
        boot_setup: Option<(EntryPoint, &GuestMemoryAtomic<GuestMemoryMmap>)>,
        #[cfg(target_arch = "x86_64")] cpuid: Vec<CpuIdEntry>,
        #[cfg(target_arch = "x86_64")] kvm_hyperv: bool,
        #[cfg(target_arch = "x86_64")] topology: Option<(u8, u8, u8)>,
    ) -> Result<()> {
        #[cfg(target_arch = "aarch64")]
        {
            self.init(vm)?;
            self.mpidr = arch::configure_vcpu(&self.vcpu, self.id, boot_setup)
                .map_err(Error::VcpuConfiguration)?;
        }
        info!("Configuring vCPU: cpu_id = {}", self.id);
        #[cfg(target_arch = "x86_64")]
        arch::configure_vcpu(
            &self.vcpu,
            self.id,
            boot_setup,
            cpuid,
            kvm_hyperv,
            self.vendor,
            topology,
        )
        .map_err(Error::VcpuConfiguration)?;

        Ok(())
    }

    /// Gets the MPIDR register value.
    #[cfg(target_arch = "aarch64")]
    pub fn get_mpidr(&self) -> u64 {
        self.mpidr
    }

    /// Initializes an aarch64 specific vcpu for booting Linux.
    #[cfg(target_arch = "aarch64")]
    pub fn init(&self, vm: &Arc<dyn hypervisor::Vm>) -> Result<()> {
        let mut kvi: kvm_bindings::kvm_vcpu_init = kvm_bindings::kvm_vcpu_init::default();

        // This reads back the kernel's preferred target type.
        vm.get_preferred_target(&mut kvi)
            .map_err(Error::VcpuArmPreferredTarget)?;
        // We already checked that the capability is supported.
        kvi.features[0] |= 1 << kvm_bindings::KVM_ARM_VCPU_PSCI_0_2;
        if vm
            .as_any()
            .downcast_ref::<hypervisor::kvm::KvmVm>()
            .unwrap()
            .check_extension(Cap::ArmPmuV3)
        {
            kvi.features[0] |= 1 << kvm_bindings::KVM_ARM_VCPU_PMU_V3;
        }
        // Non-boot cpus are powered off initially.
        if self.id > 0 {
            kvi.features[0] |= 1 << kvm_bindings::KVM_ARM_VCPU_POWER_OFF;
        }
        self.vcpu.vcpu_init(&kvi).map_err(Error::VcpuArmInit)
    }

    /// Runs the VCPU until it exits, returning the reason.
    ///
    /// Note that the state of the VCPU and associated VM must be setup first for this to do
    /// anything useful.
    pub fn run(&self) -> std::result::Result<VmExit, HypervisorCpuError> {
        self.vcpu.run()
    }

    #[cfg(feature = "sev_snp")]
    pub fn set_sev_control_register(&self, vmsa_pfn: u64) -> Result<()> {
        self.vcpu
            .set_sev_control_register(vmsa_pfn)
            .map_err(Error::SetSevControlRegister)
    }
}

pub struct CpuManager {
    config: CpusConfig,
    #[cfg_attr(target_arch = "aarch64", allow(dead_code))]
    interrupt_controller: Option<Arc<Mutex<dyn InterruptController>>>,
    #[cfg(target_arch = "x86_64")]
    cpuid: Vec<CpuIdEntry>,
    #[cfg_attr(target_arch = "aarch64", allow(dead_code))]
    vm: Arc<dyn hypervisor::Vm>,
    vcpus_kill_signalled: Arc<AtomicBool>,
    vcpus_pause_signalled: Arc<AtomicBool>,
    exit_evt: EventFd,
    #[cfg_attr(target_arch = "aarch64", allow(dead_code))]
    reset_evt: EventFd,
    #[cfg(feature = "guest_debug")]
    vm_debug_evt: EventFd,
    vcpu_states: Vec<VcpuState>,
    selected_cpu: u8,
    vcpus: Vec<Arc<Mutex<Vcpu>>>,
    vm_ops: Arc<dyn VmOps>,
    #[cfg_attr(target_arch = "aarch64", allow(dead_code))]
    acpi_address: Option<GuestAddress>,
    affinity: BTreeMap<u8, Vec<usize>>,
    dynamic: bool,
    hypervisor: Arc<dyn hypervisor::Hypervisor>,
    #[cfg(feature = "sev_snp")]
    sev_snp_enabled: bool,
}

const CPU_ENABLE_FLAG: usize = 0;
const CPU_INSERTING_FLAG: usize = 1;
const CPU_REMOVING_FLAG: usize = 2;
const CPU_EJECT_FLAG: usize = 3;

const CPU_STATUS_OFFSET: u64 = 4;
const CPU_SELECTION_OFFSET: u64 = 0;

impl BusDevice for CpuManager {
    fn read(&mut self, _base: u64, offset: u64, data: &mut [u8]) {
        // The Linux kernel, quite reasonably, doesn't zero the memory it gives us.
        data.fill(0);

        match offset {
            CPU_SELECTION_OFFSET => {
                data[0] = self.selected_cpu;
            }
            CPU_STATUS_OFFSET => {
                if self.selected_cpu < self.max_vcpus() {
                    let state = &self.vcpu_states[usize::from(self.selected_cpu)];
                    if state.active() {
                        data[0] |= 1 << CPU_ENABLE_FLAG;
                    }
                    if state.inserting {
                        data[0] |= 1 << CPU_INSERTING_FLAG;
                    }
                    if state.removing {
                        data[0] |= 1 << CPU_REMOVING_FLAG;
                    }
                } else {
                    warn!("Out of range vCPU id: {}", self.selected_cpu);
                }
            }
            _ => {
                warn!(
                    "Unexpected offset for accessing CPU manager device: {:#}",
                    offset
                );
            }
        }
    }

    fn write(&mut self, _base: u64, offset: u64, data: &[u8]) -> Option<Arc<Barrier>> {
        match offset {
            CPU_SELECTION_OFFSET => {
                self.selected_cpu = data[0];
            }
            CPU_STATUS_OFFSET => {
                if self.selected_cpu < self.max_vcpus() {
                    let state = &mut self.vcpu_states[usize::from(self.selected_cpu)];
                    // The ACPI code writes back a 1 to acknowledge the insertion
                    if (data[0] & (1 << CPU_INSERTING_FLAG) == 1 << CPU_INSERTING_FLAG)
                        && state.inserting
                    {
                        state.inserting = false;
                    }
                    // Ditto for removal
                    if (data[0] & (1 << CPU_REMOVING_FLAG) == 1 << CPU_REMOVING_FLAG)
                        && state.removing
                    {
                        state.removing = false;
                    }
                    // Trigger removal of vCPU
                    if data[0] & (1 << CPU_EJECT_FLAG) == 1 << CPU_EJECT_FLAG {
                        if let Err(e) = self.remove_vcpu(self.selected_cpu) {
                            error!("Error removing vCPU: {:?}", e);
                        }
                    }
                } else {
                    warn!("Out of range vCPU id: {}", self.selected_cpu);
                }
            }
            _ => {
                warn!(
                    "Unexpected offset for accessing CPU manager device: {:#}",
                    offset
                );
            }
        }
        None
    }
}

#[derive(Default)]
struct VcpuState {
    inserting: bool,
    removing: bool,
    pending_removal: Arc<AtomicBool>,
    handle: Option<thread::JoinHandle<()>>,
    kill: Arc<AtomicBool>,
    vcpu_run_interrupted: Arc<AtomicBool>,
    paused: Arc<AtomicBool>,
}

impl VcpuState {
    fn active(&self) -> bool {
        self.handle.is_some()
    }

    fn signal_thread(&self) {
        if let Some(handle) = self.handle.as_ref() {
            loop {
                // SAFETY: FFI call with correct arguments
                unsafe {
                    libc::pthread_kill(handle.as_pthread_t() as _, SIGRTMIN());
                }
                if self.vcpu_run_interrupted.load(Ordering::SeqCst) {
                    break;
                } else {
                    // This is more effective than thread::yield_now() at
                    // avoiding a priority inversion with the vCPU thread
                    thread::sleep(std::time::Duration::from_millis(1));
                }
            }
        }
    }

    fn join_thread(&mut self) -> Result<()> {
        if let Some(handle) = self.handle.take() {
            handle.join().map_err(Error::ThreadCleanup)?
        }

        Ok(())
    }

    fn unpark_thread(&self) {
        if let Some(handle) = self.handle.as_ref() {
            handle.thread().unpark()
        }
    }
}

impl CpuManager {
    #[allow(unused_variables)]
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        config: &CpusConfig,
        vm: Arc<dyn hypervisor::Vm>,
        exit_evt: EventFd,
        reset_evt: EventFd,
        #[cfg(feature = "guest_debug")] vm_debug_evt: EventFd,
        hypervisor: &Arc<dyn hypervisor::Hypervisor>,
        vm_ops: Arc<dyn VmOps>,
        #[cfg(feature = "tdx")] tdx_enabled: bool,
        #[cfg(feature = "sev_snp")] sev_snp_enabled: bool,
    ) -> Result<Arc<Mutex<CpuManager>>> {
        if u32::from(config.max_vcpus) > hypervisor.get_max_vcpus() {
            return Err(Error::MaximumVcpusExceeded);
        }

        let mut vcpu_states = Vec::with_capacity(usize::from(config.max_vcpus));
        vcpu_states.resize_with(usize::from(config.max_vcpus), VcpuState::default);
        let hypervisor_type = hypervisor.hypervisor_type();
        #[cfg(target_arch = "x86_64")]
        let cpu_vendor = hypervisor.get_cpu_vendor();

        #[cfg(target_arch = "x86_64")]
        if config.features.amx {
            const ARCH_GET_XCOMP_GUEST_PERM: usize = 0x1024;
            const ARCH_REQ_XCOMP_GUEST_PERM: usize = 0x1025;
            const XFEATURE_XTILEDATA: usize = 18;
            const XFEATURE_XTILEDATA_MASK: usize = 1 << XFEATURE_XTILEDATA;

            // SAFETY: the syscall is only modifying kernel internal
            // data structures that the kernel is itself expected to safeguard.
            let amx_tile = unsafe {
                libc::syscall(
                    libc::SYS_arch_prctl,
                    ARCH_REQ_XCOMP_GUEST_PERM,
                    XFEATURE_XTILEDATA,
                )
            };

            if amx_tile != 0 {
                return Err(Error::AmxEnable(anyhow!("Guest AMX usage not supported")));
            } else {
                let mask: usize = 0;
                // SAFETY: the mask being modified (not marked mutable as it is
                // modified in unsafe only which is permitted) isn't in use elsewhere.
                let result = unsafe {
                    libc::syscall(libc::SYS_arch_prctl, ARCH_GET_XCOMP_GUEST_PERM, &mask)
                };
                if result != 0 || (mask & XFEATURE_XTILEDATA_MASK) != XFEATURE_XTILEDATA_MASK {
                    return Err(Error::AmxEnable(anyhow!("Guest AMX usage not supported")));
                }
            }
        }

        let proximity_domain_per_cpu: BTreeMap<u8, u32> = {
            let cpu_list = Vec::new();
            cpu_list
        }
        .into_iter()
        .collect();

        let affinity = if let Some(cpu_affinity) = config.affinity.as_ref() {
            cpu_affinity
                .iter()
                .map(|a| (a.vcpu, a.host_cpus.clone()))
                .collect()
        } else {
            BTreeMap::new()
        };

        #[cfg(feature = "tdx")]
        let dynamic = !tdx_enabled;
        #[cfg(not(feature = "tdx"))]
        let dynamic = true;

        Ok(Arc::new(Mutex::new(CpuManager {
            config: config.clone(),
            interrupt_controller: None,
            #[cfg(target_arch = "x86_64")]
            cpuid: Vec::new(),
            vm,
            vcpus_kill_signalled: Arc::new(AtomicBool::new(false)),
            vcpus_pause_signalled: Arc::new(AtomicBool::new(false)),
            vcpu_states,
            exit_evt,
            reset_evt,
            #[cfg(feature = "guest_debug")]
            vm_debug_evt,
            selected_cpu: 0,
            vcpus: Vec::with_capacity(usize::from(config.max_vcpus)),
            vm_ops,
            acpi_address: None,
            affinity,
            dynamic,
            hypervisor: hypervisor.clone(),
            #[cfg(feature = "sev_snp")]
            sev_snp_enabled,
        })))
    }

    #[cfg(target_arch = "x86_64")]
    pub fn populate_cpuid(
        &mut self,
        memory_manager: &Arc<Mutex<MemoryManager>>,
        hypervisor: &Arc<dyn hypervisor::Hypervisor>,
        #[cfg(feature = "tdx")] tdx: bool,
    ) -> Result<()> {
        let sgx_epc_sections = memory_manager
            .lock()
            .unwrap()
            .sgx_epc_region()
            .as_ref()
            .map(|sgx_epc_region| sgx_epc_region.epc_sections().values().cloned().collect());

        self.cpuid = {
            let phys_bits = physical_bits(hypervisor, self.config.max_phys_bits);
            arch::generate_common_cpuid(
                hypervisor,
                &arch::CpuidConfig {
                    sgx_epc_sections,
                    phys_bits,
                    kvm_hyperv: self.config.kvm_hyperv,
                    #[cfg(feature = "tdx")]
                    tdx,
                    amx: self.config.features.amx,
                },
            )
            .map_err(Error::CommonCpuId)?
        };

        Ok(())
    }

    fn create_vcpu(&mut self, cpu_id: u8) -> Result<Arc<Mutex<Vcpu>>> {
        info!("Creating vCPU: cpu_id = {}", cpu_id);

        #[cfg(target_arch = "x86_64")]
        let topology = self.get_vcpu_topology();
        #[cfg(target_arch = "x86_64")]
        let x2apic_id = arch::x86_64::get_x2apic_id(cpu_id as u32, topology);
        #[cfg(target_arch = "aarch64")]
        let x2apic_id = cpu_id as u32;

        let vcpu = Vcpu::new(
            cpu_id,
            x2apic_id as u8,
            &self.vm,
            Some(self.vm_ops.clone()),
            #[cfg(target_arch = "x86_64")]
            self.hypervisor.get_cpu_vendor(),
        )?;

        let vcpu = Arc::new(Mutex::new(vcpu));

        // Adding vCPU to the CpuManager's vCPU list.
        self.vcpus.push(vcpu.clone());

        Ok(vcpu)
    }

    pub fn configure_vcpu(
        &self,
        vcpu: Arc<Mutex<Vcpu>>,
        boot_setup: Option<(EntryPoint, &GuestMemoryAtomic<GuestMemoryMmap>)>,
    ) -> Result<()> {
        let mut vcpu = vcpu.lock().unwrap();

        #[cfg(feature = "sev_snp")]
        if self.sev_snp_enabled {
            if let Some((kernel_entry_point, _)) = boot_setup {
                vcpu.set_sev_control_register(
                    kernel_entry_point.entry_addr.0 / crate::igvm::HV_PAGE_SIZE,
                )?;
            }

            // Traditional way to configure vcpu doesn't work for SEV-SNP guests.
            // All the vCPU configuration for SEV-SNP guest is provided via VMSA.
            return Ok(());
        }

        #[cfg(target_arch = "x86_64")]
        assert!(!self.cpuid.is_empty());

        #[cfg(target_arch = "x86_64")]
        let topology = self.config.topology.clone().map_or_else(
            || {
                #[cfg(feature = "mshv")]
                if matches!(self.hypervisor.hypervisor_type(), HypervisorType::Mshv) {
                    return Some((1, self.boot_vcpus(), 1));
                }
                None
            },
            |t| Some((t.threads_per_core, t.cores_per_die, t.dies_per_package)),
        );
        #[cfg(target_arch = "x86_64")]
        vcpu.configure(
            boot_setup,
            self.cpuid.clone(),
            self.config.kvm_hyperv,
            topology,
        )?;

        #[cfg(target_arch = "aarch64")]
        vcpu.configure(&self.vm, boot_setup)?;

        Ok(())
    }

    /// Only create new vCPUs if there aren't any inactive ones to reuse
    fn create_vcpus(
        &mut self,
        desired_vcpus: u8,
    ) -> Result<Vec<Arc<Mutex<Vcpu>>>> {
        let mut vcpus: Vec<Arc<Mutex<Vcpu>>> = vec![];
        info!(
            "Request to create new vCPUs: desired = {}, max = {}, allocated = {}, present = {}",
            desired_vcpus,
            self.config.max_vcpus,
            self.vcpus.len(),
            self.present_vcpus()
        );

        if desired_vcpus > self.config.max_vcpus {
            return Err(Error::DesiredVCpuCountExceedsMax);
        }

        // Only create vCPUs in excess of all the allocated vCPUs.
        for cpu_id in self.vcpus.len() as u8..desired_vcpus {
            vcpus.push(self.create_vcpu(
                cpu_id
            )?);
        }

        Ok(vcpus)
    }

    #[cfg(target_arch = "aarch64")]
    pub fn init_pmu(&self, irq: u32) -> Result<bool> {
        for cpu in self.vcpus.iter() {
            let cpu = cpu.lock().unwrap();
            // Check if PMU attr is available, if not, log the information.
            if cpu.vcpu.has_pmu_support() {
                cpu.vcpu.init_pmu(irq).map_err(Error::InitPmu)?;
            } else {
                debug!(
                    "PMU attribute is not supported in vCPU{}, skip PMU init!",
                    cpu.id
                );
                return Ok(false);
            }
        }

        Ok(true)
    }

    pub fn vcpus(&self) -> Vec<Arc<Mutex<Vcpu>>> {
        self.vcpus.clone()
    }

    fn start_vcpu(
        &mut self,
        vcpu: Arc<Mutex<Vcpu>>,
        vcpu_id: u8,
        vcpu_thread_barrier: Arc<Barrier>,
        inserting: bool,
    ) -> Result<()> {
        let reset_evt = self.reset_evt.try_clone().unwrap();
        let exit_evt = self.exit_evt.try_clone().unwrap();
        #[cfg(feature = "kvm")]
        let hypervisor_type = self.hypervisor.hypervisor_type();
        #[cfg(feature = "guest_debug")]
        let vm_debug_evt = self.vm_debug_evt.try_clone().unwrap();
        let panic_exit_evt = self.exit_evt.try_clone().unwrap();
        let vcpu_kill_signalled = self.vcpus_kill_signalled.clone();
        let vcpu_pause_signalled = self.vcpus_pause_signalled.clone();

        let vcpu_kill = self.vcpu_states[usize::from(vcpu_id)].kill.clone();
        let vcpu_run_interrupted = self.vcpu_states[usize::from(vcpu_id)]
            .vcpu_run_interrupted
            .clone();
        let panic_vcpu_run_interrupted = vcpu_run_interrupted.clone();
        let vcpu_paused = self.vcpu_states[usize::from(vcpu_id)].paused.clone();

        // Prepare the CPU set the current vCPU is expected to run onto.
        let cpuset = self.affinity.get(&vcpu_id).map(|host_cpus| {
            // SAFETY: all zeros is a valid pattern
            let mut cpuset: libc::cpu_set_t = unsafe { std::mem::zeroed() };
            // SAFETY: FFI call, trivially safe
            unsafe { libc::CPU_ZERO(&mut cpuset) };
            for host_cpu in host_cpus {
                // SAFETY: FFI call, trivially safe
                unsafe { libc::CPU_SET(*host_cpu, &mut cpuset) };
            }
            cpuset
        });

        #[cfg(target_arch = "x86_64")]
        let interrupt_controller_clone = self.interrupt_controller.as_ref().cloned();

        info!("Starting vCPU: cpu_id = {}", vcpu_id);

        let handle = Some(
            thread::Builder::new()
                .name(format!("vcpu{vcpu_id}"))
                .spawn(move || {
                    // Schedule the thread to run on the expected CPU set
                    if let Some(cpuset) = cpuset.as_ref() {
                        // SAFETY: FFI call with correct arguments
                        let ret = unsafe {
                            libc::sched_setaffinity(
                                0,
                                std::mem::size_of::<libc::cpu_set_t>(),
                                cpuset as *const libc::cpu_set_t,
                            )
                        };

                        if ret != 0 {
                            error!(
                                "Failed scheduling the vCPU {} on the expected CPU set: {}",
                                vcpu_id,
                                io::Error::last_os_error()
                            );
                            return;
                        }
                    }

                    extern "C" fn handle_signal(_: i32, _: *mut siginfo_t, _: *mut c_void) {}
                    // This uses an async signal safe handler to kill the vcpu handles.
                    register_signal_handler(SIGRTMIN(), handle_signal)
                        .expect("Failed to register vcpu signal handler");
                    // Block until all CPUs are ready.
                    vcpu_thread_barrier.wait();

                    std::panic::catch_unwind(move || {
                        loop {
                            // If we are being told to pause, we park the thread
                            // until the pause boolean is toggled.
                            // The resume operation is responsible for toggling
                            // the boolean and unpark the thread.
                            // We enter a loop because park() could spuriously
                            // return. We will then park() again unless the
                            // pause boolean has been toggled.

                            // Need to use Ordering::SeqCst as we have multiple
                            // loads and stores to different atomics and we need
                            // to see them in a consistent order in all threads

                            if vcpu_pause_signalled.load(Ordering::SeqCst) {
                                // As a pause can be caused by PIO & MMIO exits then we need to ensure they are
                                // completed by returning to KVM_RUN. From the kernel docs:
                                //
                                // For KVM_EXIT_IO, KVM_EXIT_MMIO, KVM_EXIT_OSI, KVM_EXIT_PAPR, KVM_EXIT_XEN,
                                // KVM_EXIT_EPR, KVM_EXIT_X86_RDMSR and KVM_EXIT_X86_WRMSR the corresponding
                                // operations are complete (and guest state is consistent) only after userspace
                                // has re-entered the kernel with KVM_RUN.  The kernel side will first finish
                                // incomplete operations and then check for pending signals.
                                // The pending state of the operation is not preserved in state which is
                                // visible to userspace, thus userspace should ensure that the operation is
                                // completed before performing a live migration.  Userspace can re-enter the
                                // guest with an unmasked signal pending or with the immediate_exit field set
                                // to complete pending operations without allowing any further instructions
                                // to be executed.

                                #[cfg(feature = "kvm")]
                                if matches!(hypervisor_type, HypervisorType::Kvm) {
                                    vcpu.lock().as_ref().unwrap().vcpu.set_immediate_exit(true);
                                    if !matches!(vcpu.lock().unwrap().run(), Ok(VmExit::Ignore)) {
                                        error!("Unexpected VM exit on \"immediate_exit\" run");
                                        break;
                                    }
                                    vcpu.lock().as_ref().unwrap().vcpu.set_immediate_exit(false);
                                }

                                vcpu_run_interrupted.store(true, Ordering::SeqCst);

                                vcpu_paused.store(true, Ordering::SeqCst);
                                while vcpu_pause_signalled.load(Ordering::SeqCst) {
                                    thread::park();
                                }
                                vcpu_run_interrupted.store(false, Ordering::SeqCst);
                            }

                            // We've been told to terminate
                            if vcpu_kill_signalled.load(Ordering::SeqCst)
                                || vcpu_kill.load(Ordering::SeqCst)
                            {
                                vcpu_run_interrupted.store(true, Ordering::SeqCst);
                                break;
                            }

                            #[cfg(feature = "tdx")]
                            let mut vcpu = vcpu.lock().unwrap();
                            #[cfg(not(feature = "tdx"))]
                            let vcpu = vcpu.lock().unwrap();
                            // vcpu.run() returns false on a triple-fault so trigger a reset
                            match vcpu.run() {
                                Ok(run) => match run {
                                    #[cfg(feature = "kvm")]
                                    VmExit::Debug => {
                                        info!("VmExit::Debug");
                                        #[cfg(feature = "guest_debug")]
                                        {
                                            vcpu_pause_signalled.store(true, Ordering::SeqCst);
                                            let raw_tid = get_raw_tid(vcpu_id as usize);
                                            vm_debug_evt.write(raw_tid as u64).unwrap();
                                        }
                                    }
                                    #[cfg(target_arch = "x86_64")]
                                    VmExit::IoapicEoi(vector) => {
                                        if let Some(interrupt_controller) =
                                            &interrupt_controller_clone
                                        {
                                            interrupt_controller
                                                .lock()
                                                .unwrap()
                                                .end_of_interrupt(vector);
                                        }
                                    }
                                    VmExit::Ignore => {}
                                    VmExit::Hyperv => {}
                                    VmExit::Reset => {
                                        info!("VmExit::Reset");
                                        vcpu_run_interrupted.store(true, Ordering::SeqCst);
                                        reset_evt.write(1).unwrap();
                                        break;
                                    }
                                    VmExit::Shutdown => {
                                        info!("VmExit::Shutdown");
                                        vcpu_run_interrupted.store(true, Ordering::SeqCst);
                                        exit_evt.write(1).unwrap();
                                        break;
                                    }
                                    #[cfg(feature = "tdx")]
                                    VmExit::Tdx => {
                                        if let Some(vcpu) = Arc::get_mut(&mut vcpu.vcpu) {
                                            match vcpu.get_tdx_exit_details() {
                                                Ok(details) => match details {
                                                    TdxExitDetails::GetQuote => warn!("TDG_VP_VMCALL_GET_QUOTE not supported"),
                                                    TdxExitDetails::SetupEventNotifyInterrupt => {
                                                        warn!("TDG_VP_VMCALL_SETUP_EVENT_NOTIFY_INTERRUPT not supported")
                                                    }
                                                },
                                                Err(e) => error!("Unexpected TDX VMCALL: {}", e),
                                            }
                                            vcpu.set_tdx_status(TdxExitStatus::InvalidOperand);
                                        } else {
                                            // We should never reach this code as
                                            // this means the design from the code
                                            // is wrong.
                                            unreachable!("Couldn't get a mutable reference from Arc<dyn Vcpu> as there are multiple instances");
                                        }
                                    }
                                    _ => {
                                        error!(
                                            "VCPU generated error: {:?}",
                                            Error::UnexpectedVmExit
                                        );
                                        vcpu_run_interrupted.store(true, Ordering::SeqCst);
                                        exit_evt.write(1).unwrap();
                                        break;
                                    }
                                },

                                Err(e) => {
                                    error!("VCPU generated error: {:?}", Error::VcpuRun(e.into()));
                                    vcpu_run_interrupted.store(true, Ordering::SeqCst);
                                    exit_evt.write(1).unwrap();
                                    break;
                                }
                            }

                            // We've been told to terminate
                            if vcpu_kill_signalled.load(Ordering::SeqCst)
                                || vcpu_kill.load(Ordering::SeqCst)
                            {
                                vcpu_run_interrupted.store(true, Ordering::SeqCst);
                                break;
                            }
                        }
                    })
                    .or_else(|_| {
                        panic_vcpu_run_interrupted.store(true, Ordering::SeqCst);
                        error!("vCPU thread panicked");
                        panic_exit_evt.write(1)
                    })
                    .ok();
                })
                .map_err(Error::VcpuSpawn)?,
        );

        // On hot plug calls into this function entry_point is None. It is for
        // those hotplug CPU additions that we need to set the inserting flag.
        self.vcpu_states[usize::from(vcpu_id)].handle = handle;
        self.vcpu_states[usize::from(vcpu_id)].inserting = inserting;

        Ok(())
    }

    /// Start up as many vCPUs threads as needed to reach `desired_vcpus`
    fn activate_vcpus(
        &mut self,
        desired_vcpus: u8,
        inserting: bool,
        paused: Option<bool>,
    ) -> Result<()> {
        if desired_vcpus > self.config.max_vcpus {
            return Err(Error::DesiredVCpuCountExceedsMax);
        }

        let vcpu_thread_barrier = Arc::new(Barrier::new(
            (desired_vcpus - self.present_vcpus() + 1) as usize,
        ));

        if let Some(paused) = paused {
            self.vcpus_pause_signalled.store(paused, Ordering::SeqCst);
        }

        info!(
            "Starting vCPUs: desired = {}, allocated = {}, present = {}, paused = {}",
            desired_vcpus,
            self.vcpus.len(),
            self.present_vcpus(),
            self.vcpus_pause_signalled.load(Ordering::SeqCst)
        );

        // This reuses any inactive vCPUs as well as any that were newly created
        for vcpu_id in self.present_vcpus()..desired_vcpus {
            let vcpu = Arc::clone(&self.vcpus[vcpu_id as usize]);
            self.start_vcpu(vcpu, vcpu_id, vcpu_thread_barrier.clone(), inserting)?;
        }

        // Unblock all CPU threads.
        vcpu_thread_barrier.wait();
        Ok(())
    }

    fn mark_vcpus_for_removal(&mut self, desired_vcpus: u8) {
        // Mark vCPUs for removal, actual removal happens on ejection
        for cpu_id in desired_vcpus..self.present_vcpus() {
            self.vcpu_states[usize::from(cpu_id)].removing = true;
            self.vcpu_states[usize::from(cpu_id)]
                .pending_removal
                .store(true, Ordering::SeqCst);
        }
    }

    pub fn check_pending_removed_vcpu(&mut self) -> bool {
        for state in self.vcpu_states.iter() {
            if state.active() && state.pending_removal.load(Ordering::SeqCst) {
                return true;
            }
        }
        false
    }

    fn remove_vcpu(&mut self, cpu_id: u8) -> Result<()> {
        info!("Removing vCPU: cpu_id = {}", cpu_id);
        let state = &mut self.vcpu_states[usize::from(cpu_id)];
        state.kill.store(true, Ordering::SeqCst);
        state.signal_thread();
        state.join_thread()?;
        state.handle = None;

        // Once the thread has exited, clear the "kill" so that it can reused
        state.kill.store(false, Ordering::SeqCst);
        state.pending_removal.store(false, Ordering::SeqCst);

        Ok(())
    }

    pub fn create_boot_vcpus(
        &mut self,
    ) -> Result<Vec<Arc<Mutex<Vcpu>>>> {
        self.create_vcpus(self.boot_vcpus())
    }

    // Starts all the vCPUs that the VM is booting with. Blocks until all vCPUs are running.
    pub fn start_boot_vcpus(&mut self, paused: bool) -> Result<()> {
        self.activate_vcpus(self.boot_vcpus(), false, Some(paused))
    }

    pub fn start_restored_vcpus(&mut self) -> Result<()> {
        self.activate_vcpus(self.vcpus.len() as u8, false, Some(true))
            .map_err(|e| {
                Error::StartRestoreVcpu(anyhow!("Failed to start restored vCPUs: {:#?}", e))
            })?;

        Ok(())
    }

    pub fn resize(&mut self, desired_vcpus: u8) -> Result<bool> {
        if desired_vcpus.cmp(&self.present_vcpus()) == cmp::Ordering::Equal {
            return Ok(false);
        }

        if !self.dynamic {
            return Ok(false);
        }

        if self.check_pending_removed_vcpu() {
            return Err(Error::VcpuPendingRemovedVcpu);
        }

        match desired_vcpus.cmp(&self.present_vcpus()) {
            cmp::Ordering::Greater => {
                let vcpus = self.create_vcpus(desired_vcpus)?;
                for vcpu in vcpus {
                    self.configure_vcpu(vcpu, None)?
                }
                self.activate_vcpus(desired_vcpus, true, None)?;
                Ok(true)
            }
            cmp::Ordering::Less => {
                self.mark_vcpus_for_removal(desired_vcpus);
                Ok(true)
            }
            _ => Ok(false),
        }
    }

    pub fn shutdown(&mut self) -> Result<()> {
        // Tell the vCPUs to stop themselves next time they go through the loop
        self.vcpus_kill_signalled.store(true, Ordering::SeqCst);

        // Toggle the vCPUs pause boolean
        self.vcpus_pause_signalled.store(false, Ordering::SeqCst);

        // Unpark all the VCPU threads.
        for state in self.vcpu_states.iter() {
            state.unpark_thread();
        }

        // Signal to the spawned threads (vCPUs and console signal handler). For the vCPU threads
        // this will interrupt the KVM_RUN ioctl() allowing the loop to check the boolean set
        // above.
        for state in self.vcpu_states.iter() {
            state.signal_thread();
        }

        // Wait for all the threads to finish. This removes the state from the vector.
        for mut state in self.vcpu_states.drain(..) {
            state.join_thread()?;
        }

        Ok(())
    }

    #[cfg(feature = "tdx")]
    pub fn initialize_tdx(&self, hob_address: u64) -> Result<()> {
        for vcpu in &self.vcpus {
            vcpu.lock()
                .unwrap()
                .vcpu
                .tdx_init(hob_address)
                .map_err(Error::InitializeTdx)?;
        }
        Ok(())
    }

    pub fn boot_vcpus(&self) -> u8 {
        self.config.boot_vcpus
    }

    pub fn max_vcpus(&self) -> u8 {
        self.config.max_vcpus
    }

    #[cfg(target_arch = "x86_64")]
    pub fn common_cpuid(&self) -> Vec<CpuIdEntry> {
        assert!(!self.cpuid.is_empty());
        self.cpuid.clone()
    }

    fn present_vcpus(&self) -> u8 {
        self.vcpu_states
            .iter()
            .fold(0, |acc, state| acc + state.active() as u8)
    }

    #[cfg(target_arch = "aarch64")]
    pub fn get_mpidrs(&self) -> Vec<u64> {
        self.vcpus
            .iter()
            .map(|cpu| cpu.lock().unwrap().get_mpidr())
            .collect()
    }

    pub fn get_vcpu_topology(&self) -> Option<(u8, u8, u8)> {
        self.config
            .topology
            .clone()
            .map(|t| (t.threads_per_core, t.cores_per_die, t.packages))
    }

    #[cfg(feature = "guest_debug")]
    fn get_regs(&self, cpu_id: u8) -> Result<StandardRegisters> {
        self.vcpus[usize::from(cpu_id)]
            .lock()
            .unwrap()
            .vcpu
            .get_regs()
            .map_err(Error::CpuDebug)
    }

    #[cfg(feature = "guest_debug")]
    fn set_regs(&self, cpu_id: u8, regs: &StandardRegisters) -> Result<()> {
        self.vcpus[usize::from(cpu_id)]
            .lock()
            .unwrap()
            .vcpu
            .set_regs(regs)
            .map_err(Error::CpuDebug)
    }

    #[cfg(all(target_arch = "x86_64", feature = "guest_debug"))]
    fn get_sregs(&self, cpu_id: u8) -> Result<SpecialRegisters> {
        self.vcpus[usize::from(cpu_id)]
            .lock()
            .unwrap()
            .vcpu
            .get_sregs()
            .map_err(Error::CpuDebug)
    }

    #[cfg(all(target_arch = "x86_64", feature = "guest_debug"))]
    fn set_sregs(&self, cpu_id: u8, sregs: &SpecialRegisters) -> Result<()> {
        self.vcpus[usize::from(cpu_id)]
            .lock()
            .unwrap()
            .vcpu
            .set_sregs(sregs)
            .map_err(Error::CpuDebug)
    }

    #[cfg(all(target_arch = "x86_64", feature = "guest_debug"))]
    fn translate_gva(
        &self,
        _guest_memory: &GuestMemoryAtomic<GuestMemoryMmap>,
        cpu_id: u8,
        gva: u64,
    ) -> Result<u64> {
        let (gpa, _) = self.vcpus[usize::from(cpu_id)]
            .lock()
            .unwrap()
            .vcpu
            .translate_gva(gva, /* flags: unused */ 0)
            .map_err(|e| Error::TranslateVirtualAddress(e.into()))?;
        Ok(gpa)
    }

    ///
    /// On AArch64, `translate_gva` API is not provided by KVM. We implemented
    /// it in VMM by walking through translation tables.
    ///
    /// Address translation is big topic, here we only focus the scenario that
    /// happens in VMM while debugging kernel. This `translate_gva`
    /// implementation is restricted to:
    /// - Exception Level 1
    /// - Translate high address range only (kernel space)
    ///
    /// This implementation supports following Arm-v8a features related to
    /// address translation:
    /// - FEAT_LPA
    /// - FEAT_LVA
    /// - FEAT_LPA2
    ///
    #[cfg(all(target_arch = "aarch64", feature = "guest_debug"))]
    fn translate_gva(
        &self,
        guest_memory: &GuestMemoryAtomic<GuestMemoryMmap>,
        cpu_id: u8,
        gva: u64,
    ) -> Result<u64> {
        let tcr_el1: u64 = self.vcpus[usize::from(cpu_id)]
            .lock()
            .unwrap()
            .vcpu
            .get_sys_reg(regs::TCR_EL1)
            .map_err(|e| Error::TranslateVirtualAddress(e.into()))?;
        let ttbr1_el1: u64 = self.vcpus[usize::from(cpu_id)]
            .lock()
            .unwrap()
            .vcpu
            .get_sys_reg(regs::TTBR1_EL1)
            .map_err(|e| Error::TranslateVirtualAddress(e.into()))?;
        let id_aa64mmfr0_el1: u64 = self.vcpus[usize::from(cpu_id)]
            .lock()
            .unwrap()
            .vcpu
            .get_sys_reg(regs::ID_AA64MMFR0_EL1)
            .map_err(|e| Error::TranslateVirtualAddress(e.into()))?;

        // Bit 55 of the VA determines the range, high (0xFFFxxx...)
        // or low (0x000xxx...).
        let high_range = extract_bits_64!(gva, 55, 1);
        if high_range == 0 {
            info!("VA (0x{:x}) range is not supported!", gva);
            return Ok(gva);
        }

        // High range size offset
        let tsz = extract_bits_64!(tcr_el1, 16, 6);
        // Granule size
        let tg = extract_bits_64!(tcr_el1, 30, 2);
        // Indication of 48-bits (0) or 52-bits (1) for FEAT_LPA2
        let ds = extract_bits_64!(tcr_el1, 59, 1);

        if tsz == 0 {
            info!("VA translation is not ready!");
            return Ok(gva);
        }

        // VA size is determined by TCR_BL1.T1SZ
        let va_size = 64 - tsz;
        // Number of bits in VA consumed in each level of translation
        let stride = match tg {
            3 => 13, // 64KB granule size
            1 => 11, // 16KB granule size
            _ => 9,  // 4KB, default
        };
        // Starting level of walking
        let mut level = 4 - (va_size - 4) / stride;

        // PA or IPA size is determined
        let tcr_ips = extract_bits_64!(tcr_el1, 32, 3);
        let pa_range = extract_bits_64_without_offset!(id_aa64mmfr0_el1, 4);
        // The IPA size in TCR_BL1 and PA Range in ID_AA64MMFR0_EL1 should match.
        // To be safe, we use the minimum value if they are different.
        let pa_range = std::cmp::min(tcr_ips, pa_range);
        // PA size in bits
        let pa_size = match pa_range {
            0 => 32,
            1 => 36,
            2 => 40,
            3 => 42,
            4 => 44,
            5 => 48,
            6 => 52,
            _ => {
                return Err(Error::TranslateVirtualAddress(anyhow!(format!(
                    "PA range not supported {pa_range}"
                ))))
            }
        };

        let indexmask_grainsize = (!0u64) >> (64 - (stride + 3));
        let mut indexmask = (!0u64) >> (64 - (va_size - (stride * (4 - level))));
        // If FEAT_LPA2 is present, the translation table descriptor holds
        // 50 bits of the table address of next level.
        // Otherwise, it is 48 bits.
        let descaddrmask = if ds == 1 {
            !0u64 >> (64 - 50) // mask with 50 least significant bits
        } else {
            !0u64 >> (64 - 48) // mask with 48 least significant bits
        };
        let descaddrmask = descaddrmask & !indexmask_grainsize;

        // Translation table base address
        let mut descaddr: u64 = extract_bits_64_without_offset!(ttbr1_el1, 48);
        // In the case of FEAT_LPA and FEAT_LPA2, the initial translation table
        // address bits [48:51] comes from TTBR1_EL1 bits [2:5].
        if pa_size == 52 {
            descaddr |= extract_bits_64!(ttbr1_el1, 2, 4) << 48;
        }

        // Loop through tables of each level
        loop {
            // Table offset for current level
            let table_offset: u64 = (gva >> (stride * (4 - level))) & indexmask;
            descaddr |= table_offset;
            descaddr &= !7u64;

            let mut buf = [0; 8];
            guest_memory
                .memory()
                .read(&mut buf, GuestAddress(descaddr))
                .map_err(|e| Error::TranslateVirtualAddress(e.into()))?;
            let descriptor = u64::from_le_bytes(buf);

            descaddr = descriptor & descaddrmask;
            // In the case of FEAT_LPA, the next-level translation table address
            // bits [48:51] comes from bits [12:15] of the current descriptor.
            // For FEAT_LPA2, the next-level translation table address
            // bits [50:51] comes from bits [8:9] of the current descriptor,
            // bits [48:49] comes from bits [48:49] of the descriptor which was
            // handled previously.
            if pa_size == 52 {
                if ds == 1 {
                    // FEAT_LPA2
                    descaddr |= extract_bits_64!(descriptor, 8, 2) << 50;
                } else {
                    // FEAT_LPA
                    descaddr |= extract_bits_64!(descriptor, 12, 4) << 48;
                }
            }

            if (descriptor & 2) != 0 && (level < 3) {
                // This is a table entry. Go down to next level.
                level += 1;
                indexmask = indexmask_grainsize;
                continue;
            }

            break;
        }

        // We have reached either:
        // - a page entry at level 3 or
        // - a block entry at level 1 or 2
        let page_size = 1u64 << ((stride * (4 - level)) + 3);
        descaddr &= !(page_size - 1);
        descaddr |= gva & (page_size - 1);

        Ok(descaddr)
    }

    pub(crate) fn set_acpi_address(&mut self, acpi_address: GuestAddress) {
        self.acpi_address = Some(acpi_address);
    }

    pub(crate) fn set_interrupt_controller(
        &mut self,
        interrupt_controller: Arc<Mutex<dyn InterruptController>>,
    ) {
        self.interrupt_controller = Some(interrupt_controller);
    }
}
