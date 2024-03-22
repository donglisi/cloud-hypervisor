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
    ConsoleOutputMode,
    VmConfig,
};
use crate::cpu::{CpuManager, CPU_MANAGER_ACPI_SIZE};
use crate::device_tree::{DeviceNode, DeviceTree};
use crate::interrupt::LegacyUserspaceInterruptManager;
use crate::interrupt::MsiInterruptManager;
use crate::memory_manager::{Error as MemoryManagerError, MemoryManager, MEMORY_MANAGER_ACPI_SIZE};
use crate::serial_manager::{Error as SerialManagerError, SerialManager};
use crate::{device_node, DEVICE_MANAGER_SNAPSHOT_ID};
use acpi_tables::sdt::GenericAddress;
use acpi_tables::{aml, Aml};
use arch::layout;
#[cfg(target_arch = "x86_64")]
use arch::layout::{APIC_START, IOAPIC_SIZE, IOAPIC_START};
#[cfg(target_arch = "aarch64")]
use arch::{DeviceType, MmioDeviceInfo};
#[cfg(target_arch = "x86_64")]
use devices::debug_console::DebugConsole;
#[cfg(target_arch = "aarch64")]
use devices::gic;
#[cfg(target_arch = "x86_64")]
use devices::ioapic;
#[cfg(target_arch = "aarch64")]
use devices::legacy::Pl011;
use devices::{
    interrupt_controller, interrupt_controller::InterruptController, AcpiNotificationFlags,
};
use libc::{
    cfmakeraw, isatty, tcgetattr, tcsetattr, termios,
    TCSANOW,
};
use pci::{
    DeviceRelocation, PciBarRegionType, PciBdf, PciDevice,
};
use serde::{Deserialize, Serialize};
use std::collections::{HashMap};
use std::fs::{read_link, File, OpenOptions};
use std::io::{self, stdout};
use std::mem::zeroed;
use std::num::Wrapping;
use std::os::unix::fs::OpenOptionsExt;
use std::os::unix::io::{AsRawFd, FromRawFd, RawFd};
use std::path::PathBuf;
use std::result;
use std::sync::{Arc, Mutex};
use std::time::Instant;
use vm_allocator::{AddressAllocator, SystemAllocator};
use vm_device::interrupt::{
    InterruptIndex, InterruptManager, LegacyIrqGroupConfig, MsiIrqGroupConfig,
};
use vm_device::{Bus, BusDevice, Resource};
use vm_memory::{Address, GuestAddress, GuestUsize};
#[cfg(target_arch = "x86_64")]
use vm_memory::{GuestAddressSpace, GuestMemory};
use vm_migration::{
    protocol::MemoryRangeTable, snapshot_from_id, versioned_state_from_id, Migratable,
    MigratableError, Pausable, Snapshot, SnapshotData, Snapshottable, Transportable,
};
use vmm_sys_util::eventfd::EventFd;
#[cfg(target_arch = "x86_64")]
use {devices::debug_console, devices::legacy::Serial};

#[cfg(target_arch = "aarch64")]
const MMIO_LEN: u64 = 0x1000;

// Singleton devices / devices the user cannot name
#[cfg(target_arch = "x86_64")]
const IOAPIC_DEVICE_NAME: &str = "__ioapic";
const SERIAL_DEVICE_NAME: &str = "__serial";
#[cfg(target_arch = "x86_64")]
const DEBUGCON_DEVICE_NAME: &str = "__debug_console";

/// Errors associated with device manager
#[derive(Debug)]
pub enum DeviceManagerError {
    /// Cannot create EventFd.
    EventFd(io::Error),

    /// Cannot open disk path
    Disk(io::Error),

    /// Virtio-fs device was created without a socket.
    NoVirtioFsSock,

    /// Cannot create tpm device
    CreateTpmDevice(anyhow::Error),

    /// Failed to convert Path to &str for the vDPA device.
    CreateVdpaConvertPath,

    /// Failed to parse disk image format
    DetectImageType(io::Error),

    /// Cannot create serial manager
    CreateSerialManager(SerialManagerError),

    /// Cannot spawn the serial manager thread
    SpawnSerialManager(SerialManagerError),

    /// Cannot open tap interface
    OpenTap(net_util::TapError),

    /// Cannot allocate IRQ.
    AllocateIrq,

    /// Cannot configure the IRQ.
    Irq(vmm_sys_util::errno::Error),

    /// Cannot allocate PCI BARs
    AllocateBars(pci::PciDeviceError),

    /// Could not free the BARs associated with a PCI device.
    FreePciBars(pci::PciDeviceError),

    /// Cannot register ioevent.
    RegisterIoevent(anyhow::Error),

    /// Cannot unregister ioevent.
    UnRegisterIoevent(anyhow::Error),

    /// Cannot add PCI device
    AddPciDevice(pci::PciRootError),

    /// Cannot open persistent memory file
    PmemFileOpen(io::Error),

    /// Cannot set persistent memory file size
    PmemFileSetLen(io::Error),

    /// Cannot find a memory range for persistent memory
    PmemRangeAllocation,

    /// Cannot find a memory range for virtio-fs
    FsRangeAllocation,

    /// Error creating serial output file
    SerialOutputFileOpen(io::Error),

    #[cfg(target_arch = "x86_64")]
    /// Error creating debug-console output file
    DebugconOutputFileOpen(io::Error),

    /// Error creating console output file
    ConsoleOutputFileOpen(io::Error),

    /// Error creating serial pty
    SerialPtyOpen(io::Error),

    /// Error creating console pty
    ConsolePtyOpen(io::Error),

    /// Error creating console pty
    DebugconPtyOpen(io::Error),

    /// Error setting pty raw mode
    SetPtyRaw(vmm_sys_util::errno::Error),

    /// Error getting pty peer
    GetPtyPeer(vmm_sys_util::errno::Error),

    /// Failed to memory map.
    Mmap(io::Error),

    /// Cannot add legacy device to Bus.
    BusError(vm_device::BusError),

    /// Failed to allocate IO port
    AllocateIoPort,

    /// Failed to allocate MMIO address
    AllocateMmioAddress,

    /// Failed to make hotplug notification
    HotPlugNotification(io::Error),

    /// Error from a memory manager operation
    MemoryManager(MemoryManagerError),

    /// Failed to create new interrupt source group.
    CreateInterruptGroup(io::Error),

    /// Failed to update interrupt source group.
    UpdateInterruptGroup(io::Error),

    /// Failed to create interrupt controller.
    CreateInterruptController(interrupt_controller::Error),

    /// Failed to create a new MmapRegion instance.
    NewMmapRegion(vm_memory::mmap::MmapRegionError),

    /// Failed to clone a File.
    CloneFile(io::Error),

    /// Failed to create socket file
    CreateSocketFile(io::Error),

    /// Failed to spawn the network backend
    SpawnNetBackend(io::Error),

    /// Failed to spawn the block backend
    SpawnBlockBackend(io::Error),

    /// Missing PCI bus.
    NoPciBus,

    /// Could not find an available device name.
    NoAvailableDeviceName,

    /// Missing PCI device.
    MissingPciDevice,

    /// Failed to remove a PCI device from the PCI bus.
    RemoveDeviceFromPciBus(pci::PciRootError),

    /// Failed to remove a bus device from the IO bus.
    RemoveDeviceFromIoBus(vm_device::BusError),

    /// Failed to remove a bus device from the MMIO bus.
    RemoveDeviceFromMmioBus(vm_device::BusError),

    /// Failed to find the device corresponding to a specific PCI b/d/f.
    UnknownPciBdf(u32),

    /// Not allowed to remove this type of device from the VM.
    RemovalNotAllowed(vm_virtio::VirtioDeviceType),

    /// Failed to find device corresponding to the given identifier.
    UnknownDeviceId(String),

    /// Failed to find an available PCI device ID.
    NextPciDeviceId(pci::PciRootError),

    /// Could not reserve the PCI device ID.
    GetPciDeviceId(pci::PciRootError),

    /// Could not give the PCI device ID back.
    PutPciDeviceId(pci::PciRootError),

    /// No disk path was specified when one was expected
    NoDiskPath,

    /// Cannot find a memory range for virtio-mem memory
    VirtioMemRangeAllocation,

    /// Trying to use a directory for pmem but no size specified
    PmemWithDirectorySizeMissing,

    /// Trying to use a size that is not multiple of 2MiB
    PmemSizeNotAligned,

    /// Could not find the node in the device tree.
    MissingNode,

    /// Resource was already found.
    ResourceAlreadyExists,

    /// Expected resources for virtio-pmem could not be found.
    MissingVirtioPmemResources,

    /// Missing PCI b/d/f from the DeviceNode.
    MissingDeviceNodePciBdf,

    /// No support for device passthrough
    NoDevicePassthroughSupport,

    /// No socket option support for console device
    NoSocketOptionSupportForConsoleDevice,

    /// Missing virtual IOMMU device
    MissingVirtualIommu,

    /// Failed to do power button notification
    PowerButtonNotification(io::Error),

    /// Failed to set O_DIRECT flag to file descriptor
    SetDirectIo,

    /// Cannot duplicate file descriptor
    DupFd(vmm_sys_util::errno::Error),

    /// Failed to DMA map virtio device.
    VirtioDmaMap(std::io::Error),

    /// Failed to DMA unmap virtio device.
    VirtioDmaUnmap(std::io::Error),

    /// Cannot hotplug device behind vIOMMU
    InvalidIommuHotplug,

    /// Invalid identifier as it is not unique.
    IdentifierNotUnique(String),

    /// Invalid identifier
    InvalidIdentifier(String),

    /// Failed retrieving device state from snapshot
    RestoreGetState(MigratableError),

    /// Cannot create a PvPanic device
    PvPanicCreate(devices::pvpanic::PvPanicError),
}

pub type DeviceManagerResult<T> = result::Result<T, DeviceManagerError>;

const DEVICE_MANAGER_ACPI_SIZE: usize = 0x10;

const TIOCSPTLCK: libc::c_int = 0x4004_5431;
const TIOCGTPEER: libc::c_int = 0x5441;

pub fn create_pty() -> io::Result<(File, File, PathBuf)> {
    // Try to use /dev/pts/ptmx first then fall back to /dev/ptmx
    // This is done to try and use the devpts filesystem that
    // could be available for use in the process's namespace first.
    // Ideally these are all the same file though but different
    // kernels could have things setup differently.
    // See https://www.kernel.org/doc/Documentation/filesystems/devpts.txt
    // for further details.

    let custom_flags = libc::O_NONBLOCK;
    let main = match OpenOptions::new()
        .read(true)
        .write(true)
        .custom_flags(custom_flags)
        .open("/dev/pts/ptmx")
    {
        Ok(f) => f,
        _ => OpenOptions::new()
            .read(true)
            .write(true)
            .custom_flags(custom_flags)
            .open("/dev/ptmx")?,
    };
    let mut unlock: libc::c_ulong = 0;
    // SAFETY: FFI call into libc, trivially safe
    unsafe { libc::ioctl(main.as_raw_fd(), TIOCSPTLCK as _, &mut unlock) };

    // SAFETY: FFI call into libc, trivially safe
    let sub_fd = unsafe {
        libc::ioctl(
            main.as_raw_fd(),
            TIOCGTPEER as _,
            libc::O_NOCTTY | libc::O_RDWR,
        )
    };
    if sub_fd == -1 {
        return vmm_sys_util::errno::errno_result().map_err(|e| e.into());
    }

    let proc_path = PathBuf::from(format!("/proc/self/fd/{sub_fd}"));
    let path = read_link(proc_path)?;

    // SAFETY: sub_fd is checked to be valid before being wrapped in File
    Ok((main, unsafe { File::from_raw_fd(sub_fd) }, path))
}


#[derive(Default)]
pub struct Console {
}

pub(crate) struct AddressManager {
    pub(crate) allocator: Arc<Mutex<SystemAllocator>>,
    #[cfg(target_arch = "x86_64")]
    pub(crate) io_bus: Arc<Bus>,
    pub(crate) mmio_bus: Arc<Bus>,
    pub(crate) vm: Arc<dyn hypervisor::Vm>,
    device_tree: Arc<Mutex<DeviceTree>>,
    pci_mmio32_allocators: Vec<Arc<Mutex<AddressAllocator>>>,
    pci_mmio64_allocators: Vec<Arc<Mutex<AddressAllocator>>>,
}

impl DeviceRelocation for AddressManager {
    fn move_bar(
        &self,
        old_base: u64,
        new_base: u64,
        len: u64,
        pci_dev: &mut dyn PciDevice,
        region_type: PciBarRegionType,
    ) -> std::result::Result<(), std::io::Error> {
        match region_type {
            PciBarRegionType::IoRegion => {
                #[cfg(target_arch = "x86_64")]
                {
                    // Update system allocator
                    self.allocator
                        .lock()
                        .unwrap()
                        .free_io_addresses(GuestAddress(old_base), len as GuestUsize);

                    self.allocator
                        .lock()
                        .unwrap()
                        .allocate_io_addresses(
                            Some(GuestAddress(new_base)),
                            len as GuestUsize,
                            None,
                        )
                        .ok_or_else(|| {
                            io::Error::new(io::ErrorKind::Other, "failed allocating new IO range")
                        })?;

                    // Update PIO bus
                    self.io_bus
                        .update_range(old_base, len, new_base, len)
                        .map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;
                }
                #[cfg(target_arch = "aarch64")]
                error!("I/O region is not supported");
            }
            PciBarRegionType::Memory32BitRegion | PciBarRegionType::Memory64BitRegion => {
                let allocators = if region_type == PciBarRegionType::Memory32BitRegion {
                    &self.pci_mmio32_allocators
                } else {
                    &self.pci_mmio64_allocators
                };

                // Find the specific allocator that this BAR was allocated from and use it for new one
                for allocator in allocators {
                    let allocator_base = allocator.lock().unwrap().base();
                    let allocator_end = allocator.lock().unwrap().end();

                    if old_base >= allocator_base.0 && old_base <= allocator_end.0 {
                        allocator
                            .lock()
                            .unwrap()
                            .free(GuestAddress(old_base), len as GuestUsize);

                        allocator
                            .lock()
                            .unwrap()
                            .allocate(Some(GuestAddress(new_base)), len as GuestUsize, Some(len))
                            .ok_or_else(|| {
                                io::Error::new(
                                    io::ErrorKind::Other,
                                    "failed allocating new MMIO range",
                                )
                            })?;

                        break;
                    }
                }

                // Update MMIO bus
                self.mmio_bus
                    .update_range(old_base, len, new_base, len)
                    .map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;
            }
        }

        // Update the device_tree resources associated with the device
        if let Some(id) = pci_dev.id() {
            if let Some(node) = self.device_tree.lock().unwrap().get_mut(&id) {
                let mut resource_updated = false;
                for resource in node.resources.iter_mut() {
                    if let Resource::PciBar { base, type_, .. } = resource {
                        if PciBarRegionType::from(*type_) == region_type && *base == old_base {
                            *base = new_base;
                            resource_updated = true;
                            break;
                        }
                    }
                }

                if !resource_updated {
                    return Err(io::Error::new(
                        io::ErrorKind::Other,
                        format!(
                            "Couldn't find a resource with base 0x{old_base:x} for device {id}"
                        ),
                    ));
                }
            } else {
                return Err(io::Error::new(
                    io::ErrorKind::Other,
                    format!("Couldn't find device {id} from device tree"),
                ));
            }
        }

        pci_dev.move_bar(old_base, new_base)
    }
}

#[derive(Serialize, Deserialize)]
struct DeviceManagerState {
    device_tree: DeviceTree,
    device_id_cnt: Wrapping<usize>,
}

#[derive(Debug)]
pub struct PtyPair {
    pub main: File,
    pub path: PathBuf,
}

impl Clone for PtyPair {
    fn clone(&self) -> Self {
        PtyPair {
            main: self.main.try_clone().unwrap(),
            path: self.path.clone(),
        }
    }
}

#[derive(Default)]
pub struct AcpiPlatformAddresses {
    pub pm_timer_address: Option<GenericAddress>,
    pub reset_reg_address: Option<GenericAddress>,
    pub sleep_control_reg_address: Option<GenericAddress>,
    pub sleep_status_reg_address: Option<GenericAddress>,
}

pub struct DeviceManager {
    // Manage address space related to devices
    address_manager: Arc<AddressManager>,

    // Console abstraction
    console: Arc<Console>,

    // serial PTY
    serial_pty: Option<Arc<Mutex<PtyPair>>>,

    // debug-console PTY
    debug_console_pty: Option<Arc<Mutex<PtyPair>>>,

    // Serial Manager
    serial_manager: Option<Arc<SerialManager>>,

    // To restore on exit.
    original_termios_opt: Arc<Mutex<Option<termios>>>,

    // Interrupt controller
    #[cfg(target_arch = "x86_64")]
    interrupt_controller: Option<Arc<Mutex<ioapic::Ioapic>>>,
    #[cfg(target_arch = "aarch64")]
    interrupt_controller: Option<Arc<Mutex<gic::Gic>>>,

    // Things to be added to the commandline (e.g. aarch64 early console)
    #[cfg(target_arch = "aarch64")]
    cmdline_additions: Vec<String>,

    // ACPI GED notification device
    ged_notification_device: Option<Arc<Mutex<devices::AcpiGedDevice>>>,

    // VM configuration
    config: Arc<Mutex<VmConfig>>,

    // Memory Manager
    memory_manager: Arc<Mutex<MemoryManager>>,

    // CPU Manager
    cpu_manager: Arc<Mutex<CpuManager>>,

    // List of bus devices
    // Let the DeviceManager keep strong references to the BusDevice devices.
    // This allows the IO and MMIO buses to be provided with Weak references,
    // which prevents cyclic dependencies.
    bus_devices: Vec<Arc<Mutex<dyn BusDevice>>>,

    // Counter to keep track of the consumed device IDs.
    device_id_cnt: Wrapping<usize>,

    #[cfg_attr(target_arch = "aarch64", allow(dead_code))]
    // MSI Interrupt Manager
    msi_interrupt_manager: Arc<dyn InterruptManager<GroupConfig = MsiIrqGroupConfig>>,

    #[cfg_attr(feature = "mshv", allow(dead_code))]
    // Legacy Interrupt Manager
    legacy_interrupt_manager: Option<Arc<dyn InterruptManager<GroupConfig = LegacyIrqGroupConfig>>>,

    // PCI information about devices attached to the paravirtualized IOMMU
    // It contains the virtual IOMMU PCI BDF along with the list of PCI BDF
    // representing the devices attached to the virtual IOMMU. This is useful
    // information for filling the ACPI VIOT table.
    iommu_attached_devices: Option<(PciBdf, Vec<PciBdf>)>,

    // Tree of devices, representing the dependencies between devices.
    // Useful for introspection, snapshot and restore.
    device_tree: Arc<Mutex<DeviceTree>>,

    // Exit event
    exit_evt: EventFd,
    reset_evt: EventFd,

    #[cfg(target_arch = "aarch64")]
    id_to_dev_info: HashMap<(DeviceType, String), MmioDeviceInfo>,

    acpi_address: GuestAddress,

    selected_segment: usize,

    // Start time of the VM
    timestamp: Instant,

    snapshot: Option<Snapshot>,
}

impl DeviceManager {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        #[cfg(target_arch = "x86_64")] io_bus: Arc<Bus>,
        mmio_bus: Arc<Bus>,
        vm: Arc<dyn hypervisor::Vm>,
        config: Arc<Mutex<VmConfig>>,
        memory_manager: Arc<Mutex<MemoryManager>>,
        cpu_manager: Arc<Mutex<CpuManager>>,
        exit_evt: EventFd,
        reset_evt: EventFd,
        timestamp: Instant,
        snapshot: Option<Snapshot>,
        dynamic: bool,
    ) -> DeviceManagerResult<Arc<Mutex<Self>>> {
        let (device_tree, device_id_cnt) = if let Some(snapshot) = snapshot.as_ref() {
            let state: DeviceManagerState = snapshot.to_state().unwrap();
            (
                Arc::new(Mutex::new(state.device_tree.clone())),
                state.device_id_cnt,
            )
        } else {
            (Arc::new(Mutex::new(DeviceTree::new())), Wrapping(0))
        };

        let num_pci_segments =
            if let Some(platform_config) = config.lock().unwrap().platform.as_ref() {
                platform_config.num_pci_segments
            } else {
                1
            };

        let create_mmio_allocators = |start, end, num_pci_segments, alignment| {
            // Start each PCI segment mmio range on an aligned boundary
            let pci_segment_mmio_size =
                (end - start + 1) / (alignment * num_pci_segments as u64) * alignment;

            let mut mmio_allocators = vec![];
            for i in 0..num_pci_segments as u64 {
                let mmio_start = start + i * pci_segment_mmio_size;
                let allocator = Arc::new(Mutex::new(
                    AddressAllocator::new(GuestAddress(mmio_start), pci_segment_mmio_size).unwrap(),
                ));
                mmio_allocators.push(allocator)
            }

            mmio_allocators
        };

        let start_of_mmio32_area = layout::MEM_32BIT_DEVICES_START.0;
        let end_of_mmio32_area = layout::MEM_32BIT_DEVICES_START.0 + layout::MEM_32BIT_DEVICES_SIZE;
        let pci_mmio32_allocators = create_mmio_allocators(
            start_of_mmio32_area,
            end_of_mmio32_area,
            num_pci_segments,
            4 << 10,
        );

        let start_of_mmio64_area = memory_manager.lock().unwrap().start_of_device_area().0;
        let end_of_mmio64_area = memory_manager.lock().unwrap().end_of_device_area().0;
        let pci_mmio64_allocators = create_mmio_allocators(
            start_of_mmio64_area,
            end_of_mmio64_area,
            num_pci_segments,
            4 << 30,
        );

        let address_manager = Arc::new(AddressManager {
            allocator: memory_manager.lock().unwrap().allocator(),
            #[cfg(target_arch = "x86_64")]
            io_bus,
            mmio_bus,
            vm: vm.clone(),
            device_tree: Arc::clone(&device_tree),
            pci_mmio32_allocators,
            pci_mmio64_allocators,
        });

        // First we create the MSI interrupt manager, the legacy one is created
        // later, after the IOAPIC device creation.
        // The reason we create the MSI one first is because the IOAPIC needs it,
        // and then the legacy interrupt manager needs an IOAPIC. So we're
        // handling a linear dependency chain:
        // msi_interrupt_manager <- IOAPIC <- legacy_interrupt_manager.
        let msi_interrupt_manager: Arc<dyn InterruptManager<GroupConfig = MsiIrqGroupConfig>> =
            Arc::new(MsiInterruptManager::new(
                Arc::clone(&address_manager.allocator),
                vm,
            ));

        let acpi_address = address_manager
            .allocator
            .lock()
            .unwrap()
            .allocate_platform_mmio_addresses(None, DEVICE_MANAGER_ACPI_SIZE as u64, None)
            .ok_or(DeviceManagerError::AllocateIoPort)?;

        if dynamic {
            let acpi_address = address_manager
                .allocator
                .lock()
                .unwrap()
                .allocate_platform_mmio_addresses(None, CPU_MANAGER_ACPI_SIZE as u64, None)
                .ok_or(DeviceManagerError::AllocateMmioAddress)?;

            address_manager
                .mmio_bus
                .insert(
                    cpu_manager.clone(),
                    acpi_address.0,
                    CPU_MANAGER_ACPI_SIZE as u64,
                )
                .map_err(DeviceManagerError::BusError)?;

            cpu_manager.lock().unwrap().set_acpi_address(acpi_address);
        }

        let device_manager = DeviceManager {
            address_manager: Arc::clone(&address_manager),
            console: Arc::new(Console::default()),
            interrupt_controller: None,
            #[cfg(target_arch = "aarch64")]
            cmdline_additions: Vec::new(),
            ged_notification_device: None,
            config,
            memory_manager,
            cpu_manager,
            bus_devices: Vec::new(),
            device_id_cnt,
            msi_interrupt_manager,
            legacy_interrupt_manager: None,
            iommu_attached_devices: None,
            device_tree,
            exit_evt,
            reset_evt,
            #[cfg(target_arch = "aarch64")]
            id_to_dev_info: HashMap::new(),
            acpi_address,
            selected_segment: 0,
            serial_pty: None,
            serial_manager: None,
            debug_console_pty: None,
            original_termios_opt: Arc::new(Mutex::new(None)),
            timestamp,
            snapshot,
        };

        let device_manager = Arc::new(Mutex::new(device_manager));

        address_manager
            .mmio_bus
            .insert(
                Arc::clone(&device_manager) as Arc<Mutex<dyn BusDevice>>,
                acpi_address.0,
                DEVICE_MANAGER_ACPI_SIZE as u64,
            )
            .map_err(DeviceManagerError::BusError)?;

        Ok(device_manager)
    }

    pub fn serial_pty(&self) -> Option<PtyPair> {
        self.serial_pty
            .as_ref()
            .map(|pty| pty.lock().unwrap().clone())
    }

    pub fn debug_console_pty(&self) -> Option<PtyPair> {
        self.debug_console_pty
            .as_ref()
            .map(|pty| pty.lock().unwrap().clone())
    }

    pub fn create_devices(
        &mut self,
        serial_pty: Option<PtyPair>,
        debug_console_pty: Option<PtyPair>,
        original_termios_opt: Arc<Mutex<Option<termios>>>,
    ) -> DeviceManagerResult<()> {

        let interrupt_controller = self.add_interrupt_controller()?;

        self.cpu_manager
            .lock()
            .unwrap()
            .set_interrupt_controller(interrupt_controller.clone());

        // Now we can create the legacy interrupt manager, which needs the freshly
        // formed IOAPIC device.
        let legacy_interrupt_manager: Arc<
            dyn InterruptManager<GroupConfig = LegacyIrqGroupConfig>,
        > = Arc::new(LegacyUserspaceInterruptManager::new(Arc::clone(
            &interrupt_controller,
        )));

        {
            if let Some(acpi_address) = self.memory_manager.lock().unwrap().acpi_address() {
                self.address_manager
                    .mmio_bus
                    .insert(
                        Arc::clone(&self.memory_manager) as Arc<Mutex<dyn BusDevice>>,
                        acpi_address.0,
                        MEMORY_MANAGER_ACPI_SIZE as u64,
                    )
                    .map_err(DeviceManagerError::BusError)?;
            }
        }

        #[cfg(target_arch = "x86_64")]
        self.add_legacy_devices(
            self.reset_evt
                .try_clone()
                .map_err(DeviceManagerError::EventFd)?,
        )?;

        #[cfg(target_arch = "aarch64")]
        self.add_legacy_devices(&legacy_interrupt_manager)?;

        {
            self.ged_notification_device = self.add_acpi_devices(
                &legacy_interrupt_manager,
                self.reset_evt
                    .try_clone()
                    .map_err(DeviceManagerError::EventFd)?,
                self.exit_evt
                    .try_clone()
                    .map_err(DeviceManagerError::EventFd)?,
            )?;
        }

        self.original_termios_opt = original_termios_opt;

        self.console = self.add_console_devices(
            &legacy_interrupt_manager,
            serial_pty,
            debug_console_pty,
        )?;

        self.legacy_interrupt_manager = Some(legacy_interrupt_manager);

        Ok(())
    }

    fn state(&self) -> DeviceManagerState {
        DeviceManagerState {
            device_tree: self.device_tree.lock().unwrap().clone(),
            device_id_cnt: self.device_id_cnt,
        }
    }

    #[cfg(target_arch = "aarch64")]
    /// Gets the information of the devices registered up to some point in time.
    pub fn get_device_info(&self) -> &HashMap<(DeviceType, String), MmioDeviceInfo> {
        &self.id_to_dev_info
    }

    #[cfg(target_arch = "aarch64")]
    fn add_interrupt_controller(
        &mut self,
    ) -> DeviceManagerResult<Arc<Mutex<dyn InterruptController>>> {
        let interrupt_controller: Arc<Mutex<gic::Gic>> = Arc::new(Mutex::new(
            gic::Gic::new(
                self.config.lock().unwrap().cpus.boot_vcpus,
                Arc::clone(&self.msi_interrupt_manager),
                self.address_manager.vm.clone(),
            )
            .map_err(DeviceManagerError::CreateInterruptController)?,
        ));

        self.interrupt_controller = Some(interrupt_controller.clone());

        // Restore the vGic if this is in the process of restoration
        let id = String::from(gic::GIC_SNAPSHOT_ID);
        if let Some(vgic_snapshot) = snapshot_from_id(self.snapshot.as_ref(), &id) {
            // PMU support is optional. Nothing should be impacted if the PMU initialization failed.
            if self
                .cpu_manager
                .lock()
                .unwrap()
                .init_pmu(arch::aarch64::fdt::AARCH64_PMU_IRQ + 16)
                .is_err()
            {
                info!("Failed to initialize PMU");
            }

            let vgic_state = vgic_snapshot
                .to_state()
                .map_err(DeviceManagerError::RestoreGetState)?;
            let saved_vcpu_states = self.cpu_manager.lock().unwrap().get_saved_states();
            interrupt_controller
                .lock()
                .unwrap()
                .restore_vgic(vgic_state, &saved_vcpu_states)
                .unwrap();
        }

        self.device_tree
            .lock()
            .unwrap()
            .insert(id.clone(), device_node!(id, interrupt_controller));

        Ok(interrupt_controller)
    }

    #[cfg(target_arch = "aarch64")]
    pub fn get_interrupt_controller(&mut self) -> Option<&Arc<Mutex<gic::Gic>>> {
        self.interrupt_controller.as_ref()
    }

    #[cfg(target_arch = "x86_64")]
    fn add_interrupt_controller(
        &mut self,
    ) -> DeviceManagerResult<Arc<Mutex<dyn InterruptController>>> {
        let id = String::from(IOAPIC_DEVICE_NAME);

        // Create IOAPIC
        let interrupt_controller = Arc::new(Mutex::new(
            ioapic::Ioapic::new(
                id.clone(),
                APIC_START,
                Arc::clone(&self.msi_interrupt_manager),
                versioned_state_from_id(self.snapshot.as_ref(), id.as_str())
                    .map_err(DeviceManagerError::RestoreGetState)?,
            )
            .map_err(DeviceManagerError::CreateInterruptController)?,
        ));

        self.interrupt_controller = Some(interrupt_controller.clone());

        self.address_manager
            .mmio_bus
            .insert(interrupt_controller.clone(), IOAPIC_START.0, IOAPIC_SIZE)
            .map_err(DeviceManagerError::BusError)?;

        self.bus_devices
            .push(Arc::clone(&interrupt_controller) as Arc<Mutex<dyn BusDevice>>);

        // Fill the device tree with a new node. In case of restore, we
        // know there is nothing to do, so we can simply override the
        // existing entry.
        self.device_tree
            .lock()
            .unwrap()
            .insert(id.clone(), device_node!(id, interrupt_controller));

        Ok(interrupt_controller)
    }

    fn add_acpi_devices(
        &mut self,
        interrupt_manager: &Arc<dyn InterruptManager<GroupConfig = LegacyIrqGroupConfig>>,
        reset_evt: EventFd,
        exit_evt: EventFd,
    ) -> DeviceManagerResult<Option<Arc<Mutex<devices::AcpiGedDevice>>>> {
        let vcpus_kill_signalled = self
            .cpu_manager
            .lock()
            .unwrap()
            .vcpus_kill_signalled()
            .clone();
        let shutdown_device = Arc::new(Mutex::new(devices::AcpiShutdownDevice::new(
            exit_evt,
            reset_evt,
            vcpus_kill_signalled,
        )));

        self.bus_devices
            .push(Arc::clone(&shutdown_device) as Arc<Mutex<dyn BusDevice>>);

        #[cfg(target_arch = "x86_64")]
        {
            let shutdown_pio_address: u16 = 0x600;

            self.address_manager
                .allocator
                .lock()
                .unwrap()
                .allocate_io_addresses(Some(GuestAddress(shutdown_pio_address.into())), 0x8, None)
                .ok_or(DeviceManagerError::AllocateIoPort)?;

            self.address_manager
                .io_bus
                .insert(shutdown_device, shutdown_pio_address.into(), 0x4)
                .map_err(DeviceManagerError::BusError)?;
        }

        let ged_irq = self
            .address_manager
            .allocator
            .lock()
            .unwrap()
            .allocate_irq()
            .unwrap();
        let interrupt_group = interrupt_manager
            .create_group(LegacyIrqGroupConfig {
                irq: ged_irq as InterruptIndex,
            })
            .map_err(DeviceManagerError::CreateInterruptGroup)?;
        let ged_address = self
            .address_manager
            .allocator
            .lock()
            .unwrap()
            .allocate_platform_mmio_addresses(
                None,
                devices::acpi::GED_DEVICE_ACPI_SIZE as u64,
                None,
            )
            .ok_or(DeviceManagerError::AllocateMmioAddress)?;
        let ged_device = Arc::new(Mutex::new(devices::AcpiGedDevice::new(
            interrupt_group,
            ged_irq,
            ged_address,
        )));
        self.address_manager
            .mmio_bus
            .insert(
                ged_device.clone(),
                ged_address.0,
                devices::acpi::GED_DEVICE_ACPI_SIZE as u64,
            )
            .map_err(DeviceManagerError::BusError)?;
        self.bus_devices
            .push(Arc::clone(&ged_device) as Arc<Mutex<dyn BusDevice>>);

        let pm_timer_device = Arc::new(Mutex::new(devices::AcpiPmTimerDevice::new()));

        self.bus_devices
            .push(Arc::clone(&pm_timer_device) as Arc<Mutex<dyn BusDevice>>);

        #[cfg(target_arch = "x86_64")]
        {
            let pm_timer_pio_address: u16 = 0x608;

            self.address_manager
                .allocator
                .lock()
                .unwrap()
                .allocate_io_addresses(Some(GuestAddress(pm_timer_pio_address.into())), 0x4, None)
                .ok_or(DeviceManagerError::AllocateIoPort)?;

            self.address_manager
                .io_bus
                .insert(pm_timer_device, pm_timer_pio_address.into(), 0x4)
                .map_err(DeviceManagerError::BusError)?;
        }

        Ok(Some(ged_device))
    }

    #[cfg(target_arch = "x86_64")]
    fn add_legacy_devices(&mut self, reset_evt: EventFd) -> DeviceManagerResult<()> {
        let vcpus_kill_signalled = self
            .cpu_manager
            .lock()
            .unwrap()
            .vcpus_kill_signalled()
            .clone();
        // Add a shutdown device (i8042)
        let i8042 = Arc::new(Mutex::new(devices::legacy::I8042Device::new(
            reset_evt.try_clone().unwrap(),
            vcpus_kill_signalled.clone(),
        )));

        self.bus_devices
            .push(Arc::clone(&i8042) as Arc<Mutex<dyn BusDevice>>);

        self.address_manager
            .io_bus
            .insert(i8042, 0x61, 0x4)
            .map_err(DeviceManagerError::BusError)?;
        {
            // Add a CMOS emulated device
            let mem_size = self
                .memory_manager
                .lock()
                .unwrap()
                .guest_memory()
                .memory()
                .last_addr()
                .0
                + 1;
            let mem_below_4g = std::cmp::min(arch::layout::MEM_32BIT_RESERVED_START.0, mem_size);
            let mem_above_4g = mem_size.saturating_sub(arch::layout::RAM_64BIT_START.0);

            let cmos = Arc::new(Mutex::new(devices::legacy::Cmos::new(
                mem_below_4g,
                mem_above_4g,
                reset_evt,
                Some(vcpus_kill_signalled),
            )));

            self.bus_devices
                .push(Arc::clone(&cmos) as Arc<Mutex<dyn BusDevice>>);

            self.address_manager
                .io_bus
                .insert(cmos, 0x70, 0x2)
                .map_err(DeviceManagerError::BusError)?;

            let fwdebug = Arc::new(Mutex::new(devices::legacy::FwDebugDevice::new()));

            self.bus_devices
                .push(Arc::clone(&fwdebug) as Arc<Mutex<dyn BusDevice>>);

            self.address_manager
                .io_bus
                .insert(fwdebug, 0x402, 0x1)
                .map_err(DeviceManagerError::BusError)?;
        }

        // 0x80 debug port
        let debug_port = Arc::new(Mutex::new(devices::legacy::DebugPort::new(self.timestamp)));
        self.bus_devices
            .push(Arc::clone(&debug_port) as Arc<Mutex<dyn BusDevice>>);
        self.address_manager
            .io_bus
            .insert(debug_port, 0x80, 0x1)
            .map_err(DeviceManagerError::BusError)?;

        Ok(())
    }

    #[cfg(target_arch = "aarch64")]
    fn add_legacy_devices(
        &mut self,
        interrupt_manager: &Arc<dyn InterruptManager<GroupConfig = LegacyIrqGroupConfig>>,
    ) -> DeviceManagerResult<()> {
        // Add a RTC device
        let rtc_irq = self
            .address_manager
            .allocator
            .lock()
            .unwrap()
            .allocate_irq()
            .unwrap();

        let interrupt_group = interrupt_manager
            .create_group(LegacyIrqGroupConfig {
                irq: rtc_irq as InterruptIndex,
            })
            .map_err(DeviceManagerError::CreateInterruptGroup)?;

        let rtc_device = Arc::new(Mutex::new(devices::legacy::Rtc::new(interrupt_group)));

        self.bus_devices
            .push(Arc::clone(&rtc_device) as Arc<Mutex<dyn BusDevice>>);

        let addr = arch::layout::LEGACY_RTC_MAPPED_IO_START;

        self.address_manager
            .mmio_bus
            .insert(rtc_device, addr.0, MMIO_LEN)
            .map_err(DeviceManagerError::BusError)?;

        self.id_to_dev_info.insert(
            (DeviceType::Rtc, "rtc".to_string()),
            MmioDeviceInfo {
                addr: addr.0,
                len: MMIO_LEN,
                irq: rtc_irq,
            },
        );

        Ok(())
    }

    #[cfg(target_arch = "x86_64")]
    fn add_debug_console_device(
        &mut self,
        debug_console_writer: Box<dyn io::Write + Send>,
    ) -> DeviceManagerResult<Arc<Mutex<DebugConsole>>> {
        let id = String::from(DEBUGCON_DEVICE_NAME);
        let debug_console = Arc::new(Mutex::new(DebugConsole::new(
            id.clone(),
            debug_console_writer,
        )));

        let port = self
            .config
            .lock()
            .unwrap()
            .debug_console
            .clone()
            .iobase
            .map(|port| port as u64)
            .unwrap_or(debug_console::DEFAULT_PORT);

        self.bus_devices
            .push(Arc::clone(&debug_console) as Arc<Mutex<dyn BusDevice>>);

        self.address_manager
            .allocator
            .lock()
            .unwrap()
            .allocate_io_addresses(Some(GuestAddress(port)), 0x1, None)
            .ok_or(DeviceManagerError::AllocateIoPort)?;

        self.address_manager
            .io_bus
            .insert(debug_console.clone(), port, 0x1)
            .map_err(DeviceManagerError::BusError)?;

        // Fill the device tree with a new node. In case of restore, we
        // know there is nothing to do, so we can simply override the
        // existing entry.
        self.device_tree
            .lock()
            .unwrap()
            .insert(id.clone(), device_node!(id, debug_console));

        Ok(debug_console)
    }

    #[cfg(target_arch = "x86_64")]
    fn add_serial_device(
        &mut self,
        interrupt_manager: &Arc<dyn InterruptManager<GroupConfig = LegacyIrqGroupConfig>>,
        serial_writer: Option<Box<dyn io::Write + Send>>,
    ) -> DeviceManagerResult<Arc<Mutex<Serial>>> {
        // Serial is tied to IRQ #4
        let serial_irq = 4;

        let id = String::from(SERIAL_DEVICE_NAME);

        let interrupt_group = interrupt_manager
            .create_group(LegacyIrqGroupConfig {
                irq: serial_irq as InterruptIndex,
            })
            .map_err(DeviceManagerError::CreateInterruptGroup)?;

        let serial = Arc::new(Mutex::new(Serial::new(
            id.clone(),
            interrupt_group,
            serial_writer,
            versioned_state_from_id(self.snapshot.as_ref(), id.as_str())
                .map_err(DeviceManagerError::RestoreGetState)?,
        )));

        self.bus_devices
            .push(Arc::clone(&serial) as Arc<Mutex<dyn BusDevice>>);

        self.address_manager
            .allocator
            .lock()
            .unwrap()
            .allocate_io_addresses(Some(GuestAddress(0x3f8)), 0x8, None)
            .ok_or(DeviceManagerError::AllocateIoPort)?;

        self.address_manager
            .io_bus
            .insert(serial.clone(), 0x3f8, 0x8)
            .map_err(DeviceManagerError::BusError)?;

        // Fill the device tree with a new node. In case of restore, we
        // know there is nothing to do, so we can simply override the
        // existing entry.
        self.device_tree
            .lock()
            .unwrap()
            .insert(id.clone(), device_node!(id, serial));

        Ok(serial)
    }

    #[cfg(target_arch = "aarch64")]
    fn add_serial_device(
        &mut self,
        interrupt_manager: &Arc<dyn InterruptManager<GroupConfig = LegacyIrqGroupConfig>>,
        serial_writer: Option<Box<dyn io::Write + Send>>,
    ) -> DeviceManagerResult<Arc<Mutex<Pl011>>> {
        let id = String::from(SERIAL_DEVICE_NAME);

        let serial_irq = self
            .address_manager
            .allocator
            .lock()
            .unwrap()
            .allocate_irq()
            .unwrap();

        let interrupt_group = interrupt_manager
            .create_group(LegacyIrqGroupConfig {
                irq: serial_irq as InterruptIndex,
            })
            .map_err(DeviceManagerError::CreateInterruptGroup)?;

        let serial = Arc::new(Mutex::new(devices::legacy::Pl011::new(
            id.clone(),
            interrupt_group,
            serial_writer,
            self.timestamp,
            versioned_state_from_id(self.snapshot.as_ref(), id.as_str())
                .map_err(DeviceManagerError::RestoreGetState)?,
        )));

        self.bus_devices
            .push(Arc::clone(&serial) as Arc<Mutex<dyn BusDevice>>);

        let addr = arch::layout::LEGACY_SERIAL_MAPPED_IO_START;

        self.address_manager
            .mmio_bus
            .insert(serial.clone(), addr.0, MMIO_LEN)
            .map_err(DeviceManagerError::BusError)?;

        self.id_to_dev_info.insert(
            (DeviceType::Serial, DeviceType::Serial.to_string()),
            MmioDeviceInfo {
                addr: addr.0,
                len: MMIO_LEN,
                irq: serial_irq,
            },
        );

        self.cmdline_additions
            .push(format!("earlycon=pl011,mmio,0x{:08x}", addr.0));

        // Fill the device tree with a new node. In case of restore, we
        // know there is nothing to do, so we can simply override the
        // existing entry.
        self.device_tree
            .lock()
            .unwrap()
            .insert(id.clone(), device_node!(id, serial));

        Ok(serial)
    }

    fn modify_mode<F: FnOnce(&mut termios)>(
        &mut self,
        fd: RawFd,
        f: F,
    ) -> vmm_sys_util::errno::Result<()> {
        // SAFETY: safe because we check the return value of isatty.
        if unsafe { isatty(fd) } != 1 {
            return Ok(());
        }

        // SAFETY: The following pair are safe because termios gets totally overwritten by tcgetattr
        // and we check the return result.
        let mut termios: termios = unsafe { zeroed() };
        // SAFETY: see above
        let ret = unsafe { tcgetattr(fd, &mut termios as *mut _) };
        if ret < 0 {
            return vmm_sys_util::errno::errno_result();
        }
        let mut original_termios_opt = self.original_termios_opt.lock().unwrap();
        if original_termios_opt.is_none() {
            *original_termios_opt = Some(termios);
        }
        f(&mut termios);
        // SAFETY: Safe because the syscall will only read the extent of termios and we check
        // the return result.
        let ret = unsafe { tcsetattr(fd, TCSANOW, &termios as *const _) };
        if ret < 0 {
            return vmm_sys_util::errno::errno_result();
        }

        Ok(())
    }

    fn set_raw_mode(&mut self, f: &dyn AsRawFd) -> vmm_sys_util::errno::Result<()> {
        // SAFETY: FFI call. Variable t is guaranteed to be a valid termios from modify_mode.
        self.modify_mode(f.as_raw_fd(), |t| unsafe { cfmakeraw(t) })
    }


    /// Adds all devices that behave like a console with respect to the VM
    /// configuration. This includes:
    /// - debug-console
    /// - serial-console
    /// - virtio-console
    fn add_console_devices(
        &mut self,
        interrupt_manager: &Arc<dyn InterruptManager<GroupConfig = LegacyIrqGroupConfig>>,
        serial_pty: Option<PtyPair>,
        #[cfg(target_arch = "x86_64")] debug_console_pty: Option<PtyPair>,
        #[cfg(not(target_arch = "x86_64"))] _: Option<PtyPair>,
    ) -> DeviceManagerResult<Arc<Console>> {
        let serial_config = self.config.lock().unwrap().serial.clone();
        let serial_writer: Option<Box<dyn io::Write + Send>> = match serial_config.mode {
            ConsoleOutputMode::File => Some(Box::new(
                File::create(serial_config.file.as_ref().unwrap())
                    .map_err(DeviceManagerError::SerialOutputFileOpen)?,
            )),
            ConsoleOutputMode::Pty => {
                if let Some(pty) = serial_pty.clone() {
                    self.config.lock().unwrap().serial.file = Some(pty.path.clone());
                    self.serial_pty = Some(Arc::new(Mutex::new(pty)));
                } else {
                    let (main, sub, path) =
                        create_pty().map_err(DeviceManagerError::SerialPtyOpen)?;
                    self.set_raw_mode(&sub)
                        .map_err(DeviceManagerError::SetPtyRaw)?;
                    self.config.lock().unwrap().serial.file = Some(path.clone());
                    self.serial_pty = Some(Arc::new(Mutex::new(PtyPair { main, path })));
                }
                None
            }
            ConsoleOutputMode::Tty => {
                let out = stdout();
                let _ = self.set_raw_mode(&out);
                Some(Box::new(out))
            }
            ConsoleOutputMode::Off | ConsoleOutputMode::Null | ConsoleOutputMode::Socket => None,
        };
        if serial_config.mode != ConsoleOutputMode::Off {
            let serial = self.add_serial_device(interrupt_manager, serial_writer)?;
            self.serial_manager = match serial_config.mode {
                ConsoleOutputMode::Pty | ConsoleOutputMode::Tty | ConsoleOutputMode::Socket => {
                    let serial_manager = SerialManager::new(
                        serial,
                        self.serial_pty.clone(),
                        serial_config.mode,
                        serial_config.socket,
                    )
                    .map_err(DeviceManagerError::CreateSerialManager)?;
                    if let Some(mut serial_manager) = serial_manager {
                        serial_manager
                            .start_thread(
                                self.exit_evt
                                    .try_clone()
                                    .map_err(DeviceManagerError::EventFd)?,
                            )
                            .map_err(DeviceManagerError::SpawnSerialManager)?;
                        Some(Arc::new(serial_manager))
                    } else {
                        None
                    }
                }
                _ => None,
            };
        }

        #[cfg(target_arch = "x86_64")]
        {
            let debug_console_config = self.config.lock().unwrap().debug_console.clone();
            let debug_console_writer: Option<Box<dyn io::Write + Send>> = match debug_console_config
                .mode
            {
                ConsoleOutputMode::File => Some(Box::new(
                    File::create(debug_console_config.file.as_ref().unwrap())
                        .map_err(DeviceManagerError::DebugconOutputFileOpen)?,
                )),
                ConsoleOutputMode::Pty => {
                    if let Some(pty) = debug_console_pty {
                        self.config.lock().unwrap().debug_console.file = Some(pty.path.clone());
                        self.debug_console_pty = Some(Arc::new(Mutex::new(pty)));
                    } else {
                        let (main, sub, path) =
                            create_pty().map_err(DeviceManagerError::DebugconPtyOpen)?;
                        self.set_raw_mode(&sub)
                            .map_err(DeviceManagerError::SetPtyRaw)?;
                        self.config.lock().unwrap().debug_console.file = Some(path.clone());
                        self.debug_console_pty = Some(Arc::new(Mutex::new(PtyPair { main, path })));
                    }
                    None
                }
                ConsoleOutputMode::Tty => {
                    let out = stdout();
                    let _ = self.set_raw_mode(&out);
                    Some(Box::new(out))
                }
                ConsoleOutputMode::Off | ConsoleOutputMode::Null | ConsoleOutputMode::Socket => {
                    None
                }
            };
            if let Some(writer) = debug_console_writer {
                let _ = self.add_debug_console_device(writer)?;
            }
        }

        Ok(Arc::new(Console {}))
    }

    #[cfg(target_arch = "x86_64")]
    pub fn io_bus(&self) -> &Arc<Bus> {
        &self.address_manager.io_bus
    }

    pub fn mmio_bus(&self) -> &Arc<Bus> {
        &self.address_manager.mmio_bus
    }

    pub fn allocator(&self) -> &Arc<Mutex<SystemAllocator>> {
        &self.address_manager.allocator
    }

    pub fn interrupt_controller(&self) -> Option<Arc<Mutex<dyn InterruptController>>> {
        self.interrupt_controller
            .as_ref()
            .map(|ic| ic.clone() as Arc<Mutex<dyn InterruptController>>)
    }

    pub fn console(&self) -> &Arc<Console> {
        &self.console
    }

    #[cfg(target_arch = "aarch64")]
    pub fn cmdline_additions(&self) -> &[String] {
        self.cmdline_additions.as_slice()
    }

    pub fn notify_hotplug(
        &self,
        _notification_type: AcpiNotificationFlags,
    ) -> DeviceManagerResult<()> {
        return self
            .ged_notification_device
            .as_ref()
            .unwrap()
            .lock()
            .unwrap()
            .notify(_notification_type)
            .map_err(DeviceManagerError::HotPlugNotification);
    }

    pub fn eject_device(&mut self, pci_segment_id: u16, device_id: u8) -> DeviceManagerResult<()> {
        info!(
            "Ejecting device_id = {} on segment_id={}",
            device_id, pci_segment_id
        );

        // Convert the device ID into the corresponding b/d/f.
        let pci_device_bdf = PciBdf::new(pci_segment_id, 0, device_id, 0);

        if let Some((_, iommu_attached_devices)) = &self.iommu_attached_devices {
            if iommu_attached_devices.contains(&pci_device_bdf) {
            }
        }

        // At this point, the device has been removed from all the list and
        // buses where it was stored. At the end of this function, after
        // any_device, bus_device and pci_device are released, the actual
        // device will be dropped.
        Ok(())
    }

    pub fn counters(&self) -> HashMap<String, HashMap<&'static str, Wrapping<u64>>> {
        let counters = HashMap::new();

        counters
    }

    pub fn device_tree(&self) -> Arc<Mutex<DeviceTree>> {
        self.device_tree.clone()
    }

    #[cfg(target_arch = "x86_64")]
    pub fn notify_power_button(&self) -> DeviceManagerResult<()> {
        self.ged_notification_device
            .as_ref()
            .unwrap()
            .lock()
            .unwrap()
            .notify(AcpiNotificationFlags::POWER_BUTTON_CHANGED)
            .map_err(DeviceManagerError::PowerButtonNotification)
    }

    #[cfg(target_arch = "aarch64")]
    pub fn notify_power_button(&self) -> DeviceManagerResult<()> {
        // There are two use cases:
        // 1. Users will use direct kernel boot with device tree.
        // 2. Users will use ACPI+UEFI boot.

        return self
            .ged_notification_device
            .as_ref()
            .unwrap()
            .lock()
            .unwrap()
            .notify(AcpiNotificationFlags::POWER_BUTTON_CHANGED)
            .map_err(DeviceManagerError::PowerButtonNotification);
    }

    pub fn iommu_attached_devices(&self) -> &Option<(PciBdf, Vec<PciBdf>)> {
        &self.iommu_attached_devices
    }
}

struct TpmDevice {}

impl Aml for TpmDevice {
    fn to_aml_bytes(&self, sink: &mut dyn acpi_tables::AmlSink) {
        aml::Device::new(
            "TPM2".into(),
            vec![
                &aml::Name::new("_HID".into(), &"MSFT0101"),
                &aml::Name::new("_STA".into(), &(0xF_usize)),
                &aml::Name::new(
                    "_CRS".into(),
                    &aml::ResourceTemplate::new(vec![&aml::Memory32Fixed::new(
                        true,
                        layout::TPM_START.0 as u32,
                        layout::TPM_SIZE as u32,
                    )]),
                ),
            ],
        )
        .to_aml_bytes(sink)
    }
}

impl Aml for DeviceManager {
    fn to_aml_bytes(&self, sink: &mut dyn acpi_tables::AmlSink) {
        #[cfg(target_arch = "aarch64")]
        use arch::aarch64::DeviceInfoForFdt;

        let pci_scan_inner: Vec<&dyn Aml> = Vec::new();

        // PCI hotplug controller
        aml::Device::new(
            "_SB_.PHPR".into(),
            vec![
                &aml::Name::new("_HID".into(), &aml::EISAName::new("PNP0A06")),
                &aml::Name::new("_STA".into(), &0x0bu8),
                &aml::Name::new("_UID".into(), &"PCI Hotplug Controller"),
                &aml::Mutex::new("BLCK".into(), 0),
                &aml::Name::new(
                    "_CRS".into(),
                    &aml::ResourceTemplate::new(vec![&aml::AddressSpace::new_memory(
                        aml::AddressSpaceCacheable::NotCacheable,
                        true,
                        self.acpi_address.0,
                        self.acpi_address.0 + DEVICE_MANAGER_ACPI_SIZE as u64 - 1,
                        None,
                    )]),
                ),
                // OpRegion and Fields map MMIO range into individual field values
                &aml::OpRegion::new(
                    "PCST".into(),
                    aml::OpRegionSpace::SystemMemory,
                    &(self.acpi_address.0 as usize),
                    &DEVICE_MANAGER_ACPI_SIZE,
                ),
                &aml::Field::new(
                    "PCST".into(),
                    aml::FieldAccessType::DWord,
                    aml::FieldLockRule::NoLock,
                    aml::FieldUpdateRule::WriteAsZeroes,
                    vec![
                        aml::FieldEntry::Named(*b"PCIU", 32),
                        aml::FieldEntry::Named(*b"PCID", 32),
                        aml::FieldEntry::Named(*b"B0EJ", 32),
                        aml::FieldEntry::Named(*b"PSEG", 32),
                    ],
                ),
                &aml::Method::new(
                    "PCEJ".into(),
                    2,
                    true,
                    vec![
                        // Take lock defined above
                        &aml::Acquire::new("BLCK".into(), 0xffff),
                        // Choose the current segment
                        &aml::Store::new(&aml::Path::new("PSEG"), &aml::Arg(1)),
                        // Write PCI bus number (in first argument) to I/O port via field
                        &aml::ShiftLeft::new(&aml::Path::new("B0EJ"), &aml::ONE, &aml::Arg(0)),
                        // Release lock
                        &aml::Release::new("BLCK".into()),
                        // Return 0
                        &aml::Return::new(&aml::ZERO),
                    ],
                ),
                &aml::Method::new("PSCN".into(), 0, true, pci_scan_inner),
            ],
        )
        .to_aml_bytes(sink);

        let mbrd_memory_refs = Vec::new();

        aml::Device::new(
            "_SB_.MBRD".into(),
            vec![
                &aml::Name::new("_HID".into(), &aml::EISAName::new("PNP0C02")),
                &aml::Name::new("_UID".into(), &aml::ZERO),
                &aml::Name::new("_CRS".into(), &aml::ResourceTemplate::new(mbrd_memory_refs)),
            ],
        )
        .to_aml_bytes(sink);

        // Serial device
        #[cfg(target_arch = "x86_64")]
        let serial_irq = 4;
        #[cfg(target_arch = "aarch64")]
        let serial_irq =
            if self.config.lock().unwrap().serial.clone().mode != ConsoleOutputMode::Off {
                self.get_device_info()
                    .clone()
                    .get(&(DeviceType::Serial, DeviceType::Serial.to_string()))
                    .unwrap()
                    .irq()
            } else {
                // If serial is turned off, add a fake device with invalid irq.
                31
            };
        if self.config.lock().unwrap().serial.mode != ConsoleOutputMode::Off {
            aml::Device::new(
                "_SB_.COM1".into(),
                vec![
                    &aml::Name::new(
                        "_HID".into(),
                        #[cfg(target_arch = "x86_64")]
                        &aml::EISAName::new("PNP0501"),
                        #[cfg(target_arch = "aarch64")]
                        &"ARMH0011",
                    ),
                    &aml::Name::new("_UID".into(), &aml::ZERO),
                    &aml::Name::new("_DDN".into(), &"COM1"),
                    &aml::Name::new(
                        "_CRS".into(),
                        &aml::ResourceTemplate::new(vec![
                            &aml::Interrupt::new(true, true, false, false, serial_irq),
                            #[cfg(target_arch = "x86_64")]
                            &aml::IO::new(0x3f8, 0x3f8, 0, 0x8),
                            #[cfg(target_arch = "aarch64")]
                            &aml::Memory32Fixed::new(
                                true,
                                arch::layout::LEGACY_SERIAL_MAPPED_IO_START.raw_value() as u32,
                                MMIO_LEN as u32,
                            ),
                        ]),
                    ),
                ],
            )
            .to_aml_bytes(sink);
        }

        aml::Name::new("_S5_".into(), &aml::Package::new(vec![&5u8])).to_aml_bytes(sink);

        aml::Device::new(
            "_SB_.PWRB".into(),
            vec![
                &aml::Name::new("_HID".into(), &aml::EISAName::new("PNP0C0C")),
                &aml::Name::new("_UID".into(), &aml::ZERO),
            ],
        )
        .to_aml_bytes(sink);

        if self.config.lock().unwrap().tpm.is_some() {
            // Add tpm device
            TpmDevice {}.to_aml_bytes(sink);
        }

        self.ged_notification_device
            .as_ref()
            .unwrap()
            .lock()
            .unwrap()
            .to_aml_bytes(sink)
    }
}

impl Pausable for DeviceManager {
    fn pause(&mut self) -> result::Result<(), MigratableError> {
        for (_, device_node) in self.device_tree.lock().unwrap().iter() {
            if let Some(migratable) = &device_node.migratable {
                migratable.lock().unwrap().pause()?;
            }
        }
        // On AArch64, the pause of device manager needs to trigger
        // a "pause" of GIC, which will flush the GIC pending tables
        // and ITS tables to guest RAM.
        #[cfg(target_arch = "aarch64")]
        {
            self.get_interrupt_controller()
                .unwrap()
                .lock()
                .unwrap()
                .pause()?;
        };

        Ok(())
    }

    fn resume(&mut self) -> result::Result<(), MigratableError> {
        for (_, device_node) in self.device_tree.lock().unwrap().iter() {
            if let Some(migratable) = &device_node.migratable {
                migratable.lock().unwrap().resume()?;
            }
        }

        Ok(())
    }
}

impl Snapshottable for DeviceManager {
    fn id(&self) -> String {
        DEVICE_MANAGER_SNAPSHOT_ID.to_string()
    }

    fn snapshot(&mut self) -> std::result::Result<Snapshot, MigratableError> {
        let mut snapshot = Snapshot::from_data(SnapshotData::new_from_state(&self.state())?);

        // We aggregate all devices snapshots.
        for (_, device_node) in self.device_tree.lock().unwrap().iter() {
            if let Some(migratable) = &device_node.migratable {
                let mut migratable = migratable.lock().unwrap();
                snapshot.add_snapshot(migratable.id(), migratable.snapshot()?);
            }
        }

        Ok(snapshot)
    }
}

impl Transportable for DeviceManager {}

impl Migratable for DeviceManager {
    fn start_dirty_log(&mut self) -> std::result::Result<(), MigratableError> {
        for (_, device_node) in self.device_tree.lock().unwrap().iter() {
            if let Some(migratable) = &device_node.migratable {
                migratable.lock().unwrap().start_dirty_log()?;
            }
        }
        Ok(())
    }

    fn stop_dirty_log(&mut self) -> std::result::Result<(), MigratableError> {
        for (_, device_node) in self.device_tree.lock().unwrap().iter() {
            if let Some(migratable) = &device_node.migratable {
                migratable.lock().unwrap().stop_dirty_log()?;
            }
        }
        Ok(())
    }

    fn dirty_log(&mut self) -> std::result::Result<MemoryRangeTable, MigratableError> {
        let mut tables = Vec::new();
        for (_, device_node) in self.device_tree.lock().unwrap().iter() {
            if let Some(migratable) = &device_node.migratable {
                tables.push(migratable.lock().unwrap().dirty_log()?);
            }
        }
        Ok(MemoryRangeTable::new_from_tables(tables))
    }

    fn start_migration(&mut self) -> std::result::Result<(), MigratableError> {
        for (_, device_node) in self.device_tree.lock().unwrap().iter() {
            if let Some(migratable) = &device_node.migratable {
                migratable.lock().unwrap().start_migration()?;
            }
        }
        Ok(())
    }

    fn complete_migration(&mut self) -> std::result::Result<(), MigratableError> {
        for (_, device_node) in self.device_tree.lock().unwrap().iter() {
            if let Some(migratable) = &device_node.migratable {
                migratable.lock().unwrap().complete_migration()?;
            }
        }
        Ok(())
    }
}

const B0EJ_FIELD_OFFSET: u64 = 8;
const PSEG_FIELD_OFFSET: u64 = 12;
const B0EJ_FIELD_SIZE: usize = 4;
const PSEG_FIELD_SIZE: usize = 4;

impl BusDevice for DeviceManager {
    fn read(&mut self, base: u64, offset: u64, data: &mut [u8]) {
        match offset {
            PSEG_FIELD_OFFSET => {
                assert_eq!(data.len(), PSEG_FIELD_SIZE);
                data.copy_from_slice(&(self.selected_segment as u32).to_le_bytes());
            }
            _ => error!(
                "Accessing unknown location at base 0x{:x}, offset 0x{:x}",
                base, offset
            ),
        }

        debug!(
            "PCI_HP_REG_R: base 0x{:x}, offset 0x{:x}, data {:?}",
            base, offset, data
        )
    }

    fn write(&mut self, base: u64, offset: u64, data: &[u8]) -> Option<Arc<std::sync::Barrier>> {
        match offset {
            B0EJ_FIELD_OFFSET => {
                assert!(data.len() == B0EJ_FIELD_SIZE);
                let mut data_array: [u8; 4] = [0, 0, 0, 0];
                data_array.copy_from_slice(data);
                let mut slot_bitmap = u32::from_le_bytes(data_array);

                while slot_bitmap > 0 {
                    let slot_id = slot_bitmap.trailing_zeros();
                    if let Err(e) = self.eject_device(self.selected_segment as u16, slot_id as u8) {
                        error!("Failed ejecting device {}: {:?}", slot_id, e);
                    }
                    slot_bitmap &= !(1 << slot_id);
                }
            }
            _ => error!(
                "Accessing unknown location at base 0x{:x}, offset 0x{:x}",
                base, offset
            ),
        }

        debug!(
            "PCI_HP_REG_W: base 0x{:x}, offset 0x{:x}, data {:?}",
            base, offset, data
        );

        None
    }
}
