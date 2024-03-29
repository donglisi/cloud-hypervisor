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
use crate::device_node;
#[cfg(target_arch = "x86_64")]
use arch::layout::{APIC_START, IOAPIC_SIZE, IOAPIC_START};
#[cfg(target_arch = "aarch64")]
use arch::{DeviceType, MmioDeviceInfo};
#[cfg(target_arch = "aarch64")]
use devices::gic;
#[cfg(target_arch = "x86_64")]
use devices::ioapic;
#[cfg(target_arch = "aarch64")]
use devices::legacy::Pl011;
use devices::{
    interrupt_controller, interrupt_controller::InterruptController,
};
use libc::{
    cfmakeraw, isatty, tcgetattr, tcsetattr, termios,
    TCSANOW,
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
use vm_allocator::{SystemAllocator};
use vm_device::interrupt::{
    InterruptIndex, InterruptManager, LegacyIrqGroupConfig, MsiIrqGroupConfig,
};
use vm_device::{Bus, BusDevice};
use vmm_sys_util::eventfd::EventFd;
#[cfg(target_arch = "x86_64")]
use {devices::legacy::Serial};

#[cfg(target_arch = "aarch64")]
const MMIO_LEN: u64 = 0x1000;

// Singleton devices / devices the user cannot name
#[cfg(target_arch = "x86_64")]
const IOAPIC_DEVICE_NAME: &str = "__ioapic";
const SERIAL_DEVICE_NAME: &str = "__serial";

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

    /// Cannot allocate IRQ.
    AllocateIrq,

    /// Cannot configure the IRQ.
    Irq(vmm_sys_util::errno::Error),

    /// Cannot register ioevent.
    RegisterIoevent(anyhow::Error),

    /// Cannot unregister ioevent.
    UnRegisterIoevent(anyhow::Error),

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

    /// Failed to remove a bus device from the IO bus.
    RemoveDeviceFromIoBus(vm_device::BusError),

    /// Failed to remove a bus device from the MMIO bus.
    RemoveDeviceFromMmioBus(vm_device::BusError),

    /// Failed to find the device corresponding to a specific PCI b/d/f.
    UnknownPciBdf(u32),

    /// Failed to find device corresponding to the given identifier.
    UnknownDeviceId(String),

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
    #[cfg(target_arch = "aarch64")]
    pub(crate) vm: Arc<dyn hypervisor::Vm>,
}

#[derive(Serialize, Deserialize)]
struct DeviceManagerState {
    device_tree: DeviceTree,
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

pub struct DeviceManager {
    // Manage address space related to devices
    address_manager: Arc<AddressManager>,

    // Console abstraction
    console: Arc<Console>,

    // serial PTY
    serial_pty: Option<Arc<Mutex<PtyPair>>>,

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

    #[cfg_attr(target_arch = "aarch64", allow(dead_code))]
    // MSI Interrupt Manager
    msi_interrupt_manager: Arc<dyn InterruptManager<GroupConfig = MsiIrqGroupConfig>>,

    #[cfg_attr(feature = "mshv", allow(dead_code))]
    // Legacy Interrupt Manager
    legacy_interrupt_manager: Option<Arc<dyn InterruptManager<GroupConfig = LegacyIrqGroupConfig>>>,

    // Tree of devices, representing the dependencies between devices.
    // Useful for introspection, snapshot and restore.
    device_tree: Arc<Mutex<DeviceTree>>,

    // Exit event
    exit_evt: EventFd,

    #[cfg(target_arch = "aarch64")]
    id_to_dev_info: HashMap<(DeviceType, String), MmioDeviceInfo>,

    selected_segment: usize,
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
        dynamic: bool,
    ) -> DeviceManagerResult<Arc<Mutex<Self>>> {
        let device_tree = Arc::new(Mutex::new(DeviceTree::new()));

        let address_manager = Arc::new(AddressManager {
            allocator: memory_manager.lock().unwrap().allocator(),
            #[cfg(target_arch = "x86_64")]
            io_bus,
            mmio_bus,
            #[cfg(target_arch = "aarch64")]
            vm: vm.clone(),
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
            config,
            memory_manager,
            cpu_manager,
            bus_devices: Vec::new(),
            msi_interrupt_manager,
            legacy_interrupt_manager: None,
            device_tree,
            exit_evt,
            #[cfg(target_arch = "aarch64")]
            id_to_dev_info: HashMap::new(),
            selected_segment: 0,
            serial_pty: None,
            serial_manager: None,
            original_termios_opt: Arc::new(Mutex::new(None)),
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

    pub fn create_devices(
        &mut self,
        serial_pty: Option<PtyPair>,
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

        self.original_termios_opt = original_termios_opt;

        self.console = self.add_console_devices(
            &legacy_interrupt_manager,
            serial_pty,
        )?;

        self.legacy_interrupt_manager = Some(legacy_interrupt_manager);

        Ok(())
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
                APIC_START,
                Arc::clone(&self.msi_interrupt_manager),
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
            interrupt_group,
            serial_writer,
        )));

        self.bus_devices
            .push(Arc::clone(&serial) as Arc<Mutex<dyn BusDevice>>);

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
            interrupt_group,
            serial_writer,
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

    pub fn counters(&self) -> HashMap<String, HashMap<&'static str, Wrapping<u64>>> {
        let counters = HashMap::new();

        counters
    }

    pub fn device_tree(&self) -> Arc<Mutex<DeviceTree>> {
        self.device_tree.clone()
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

impl Drop for DeviceManager {
    fn drop(&mut self) {
        if let Some(termios) = *self.original_termios_opt.lock().unwrap() {
            // SAFETY: FFI call
            let _ = unsafe { tcsetattr(stdout().lock().as_raw_fd(), TCSANOW, &termios) };
        }
    }
}
