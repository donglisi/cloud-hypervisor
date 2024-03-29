// Copyright © 2019 Intel Corporation
//
// SPDX-License-Identifier: Apache-2.0
//
#[cfg(target_arch = "x86_64")]
use crate::config::SgxEpcConfig;
use crate::config::{MemoryConfig, MemoryZoneConfig};
#[cfg(all(target_arch = "x86_64", feature = "guest_debug"))]
use crate::coredump::{
    CoredumpMemoryRegion, CoredumpMemoryRegions, DumpState, GuestDebuggableError,
};
use crate::{GuestMemoryMmap, GuestRegionMmap};
#[cfg(target_arch = "x86_64")]
use arch::x86_64::{SgxEpcRegion, SgxEpcSection};
use arch::RegionType;
#[cfg(target_arch = "x86_64")]
use devices::ioapic;
#[cfg(target_arch = "aarch64")]
use hypervisor::HypervisorVmError;
use libc::_SC_NPROCESSORS_ONLN;
#[cfg(target_arch = "x86_64")]
use libc::{MAP_NORESERVE, MAP_POPULATE, MAP_SHARED, PROT_READ, PROT_WRITE};
use serde::{Deserialize, Serialize};
#[cfg(all(target_arch = "x86_64", feature = "guest_debug"))]
use std::collections::BTreeMap;
use std::collections::HashMap;
use std::fs::{File, OpenOptions};
use std::io::{self};
use std::ops::{BitAnd, Deref, Not, Sub};
#[cfg(all(target_arch = "x86_64", feature = "guest_debug"))]
use std::os::fd::AsFd;
use std::os::unix::io::{AsRawFd, FromRawFd, RawFd};
use std::path::PathBuf;
use std::sync::{Arc, Barrier, Mutex};
use std::{ffi, thread};
use versionize::{VersionMap, Versionize, VersionizeResult};
use versionize_derive::Versionize;
#[cfg(target_arch = "x86_64")]
use vm_allocator::GsiApic;
use vm_allocator::{AddressAllocator, SystemAllocator};
use vm_device::BusDevice;
use vm_memory::bitmap::AtomicBitmap;
use vm_memory::guest_memory::FileOffset;
use vm_memory::{
    mmap::MmapRegionError, Address, Error as MmapError, GuestAddress, GuestAddressSpace,
    GuestMemory, GuestMemoryAtomic, GuestMemoryError, GuestMemoryRegion, GuestUsize, MmapRegion,
};

pub const MEMORY_MANAGER_ACPI_SIZE: usize = 0x18;

const DEFAULT_MEMORY_ZONE: &str = "mem0";

#[cfg(target_arch = "x86_64")]
const X86_64_IRQ_BASE: u32 = 5;

#[cfg(target_arch = "x86_64")]
const SGX_PAGE_SIZE: u64 = 1 << 12;

const HOTPLUG_COUNT: usize = 8;

// Memory policy constants
const MPOL_BIND: u32 = 2;
const MPOL_MF_STRICT: u32 = 1;
const MPOL_MF_MOVE: u32 = 1 << 1;

// Reserve 1 MiB for platform MMIO devices (e.g. ACPI control devices)
const PLATFORM_DEVICE_AREA_SIZE: u64 = 1 << 20;

const MAX_PREFAULT_THREAD_COUNT: usize = 16;

#[derive(Clone, Default, Serialize, Deserialize, Versionize)]
struct HotPlugState {
    base: u64,
    length: u64,
    active: bool,
    inserting: bool,
    removing: bool,
}

#[derive(Default)]
pub struct MemoryZone {
    regions: Vec<Arc<GuestRegionMmap>>,
}

impl MemoryZone {
    pub fn regions(&self) -> &Vec<Arc<GuestRegionMmap>> {
        &self.regions
    }
}

pub type MemoryZones = HashMap<String, MemoryZone>;

#[derive(Clone, Serialize, Deserialize, Versionize)]
struct GuestRamMapping {
    slot: u32,
    gpa: u64,
    size: u64,
    zone_id: String,
    virtio_mem: bool,
    file_offset: u64,
}

#[derive(Clone, Serialize, Deserialize, Versionize)]
struct ArchMemRegion {
    base: u64,
    size: usize,
    r_type: RegionType,
}

pub struct MemoryManager {
    boot_guest_memory: GuestMemoryMmap,
    guest_memory: GuestMemoryAtomic<GuestMemoryMmap>,
    next_memory_slot: u32,
    start_of_device_area: GuestAddress,
    end_of_device_area: GuestAddress,
    pub vm: Arc<dyn hypervisor::Vm>,
    hotplug_slots: Vec<HotPlugState>,
    selected_slot: usize,
    mergeable: bool,
    allocator: Arc<Mutex<SystemAllocator>>,
    shared: bool,
    hugepages: bool,
    hugepage_size: Option<u64>,
    prefault: bool,
    thp: bool,
    #[cfg(target_arch = "x86_64")]
    sgx_epc_region: Option<SgxEpcRegion>,
    memory_zones: MemoryZones,
    log_dirty: bool, // Enable dirty logging for created RAM regions
    arch_mem_regions: Vec<ArchMemRegion>,
    ram_allocator: AddressAllocator,

    // Keep track of calls to create_userspace_mapping() for guest RAM.
    // This is useful for getting the dirty pages as we need to know the
    // slots that the mapping is created in.
    guest_ram_mappings: Vec<GuestRamMapping>,

    pub acpi_address: Option<GuestAddress>,
    #[cfg(target_arch = "aarch64")]
    uefi_flash: Option<GuestMemoryAtomic<GuestMemoryMmap>>,
}

#[derive(Debug)]
pub enum Error {
    /// Failed to create shared file.
    SharedFileCreate(io::Error),

    /// Failed to set shared file length.
    SharedFileSetLen(io::Error),

    /// Mmap backed guest memory error
    GuestMemory(MmapError),

    /// Failed to allocate a memory range.
    MemoryRangeAllocation,

    /// Error from region creation
    GuestMemoryRegion(MmapRegionError),

    /// No ACPI slot available
    NoSlotAvailable,

    /// Not enough space in the hotplug RAM region
    InsufficientHotplugRam,

    /// The requested hotplug memory addition is not a valid size
    InvalidSize,

    /// Failed to create the user memory region.
    CreateUserMemoryRegion(hypervisor::HypervisorVmError),

    /// Failed to remove the user memory region.
    RemoveUserMemoryRegion(hypervisor::HypervisorVmError),

    /// Failed to EventFd.
    EventFdFail(io::Error),

    /// Eventfd write error
    EventfdError(io::Error),

    /// Cannot restore VM because source URL is missing
    RestoreMissingSourceUrl,

    /// Cannot create the system allocator
    CreateSystemAllocator,

    /// Invalid SGX EPC section size
    #[cfg(target_arch = "x86_64")]
    EpcSectionSizeInvalid,

    /// Failed allocating SGX EPC region
    #[cfg(target_arch = "x86_64")]
    SgxEpcRangeAllocation,

    /// Failed opening SGX virtual EPC device
    #[cfg(target_arch = "x86_64")]
    SgxVirtEpcOpen(io::Error),

    /// Failed setting the SGX virtual EPC section size
    #[cfg(target_arch = "x86_64")]
    SgxVirtEpcFileSetLen(io::Error),

    /// Failed opening SGX provisioning device
    #[cfg(target_arch = "x86_64")]
    SgxProvisionOpen(io::Error),

    /// Failed enabling SGX provisioning
    #[cfg(target_arch = "x86_64")]
    SgxEnableProvisioning(hypervisor::HypervisorVmError),

    /// Failed creating a new MmapRegion instance.
    #[cfg(target_arch = "x86_64")]
    NewMmapRegion(vm_memory::mmap::MmapRegionError),

    /// No memory zones found.
    MissingMemoryZones,

    /// Memory configuration is not valid.
    InvalidMemoryParameters,

    /// Forbidden operation. Impossible to resize guest memory if it is
    /// backed by user defined memory regions.
    InvalidResizeWithMemoryZones,

    /// It's invalid to try applying a NUMA policy to a memory zone that is
    /// memory mapped with MAP_SHARED.
    InvalidSharedMemoryZoneWithHostNuma,

    /// Failed applying NUMA memory policy.
    ApplyNumaPolicy(io::Error),

    /// Memory zone identifier is not unique.
    DuplicateZoneId,

    /// No virtio-mem resizing handler found.
    MissingVirtioMemHandler,

    /// Unknown memory zone.
    UnknownMemoryZone,

    /// Invalid size for resizing. Can be anything except 0.
    InvalidHotplugSize,

    /// Invalid hotplug method associated with memory zones resizing capability.
    InvalidHotplugMethodWithMemoryZones,

    /// Could not find specified memory zone identifier from hash map.
    MissingZoneIdentifier,

    /// Resizing the memory zone failed.
    ResizeZone,

    /// Guest address overflow
    GuestAddressOverFlow,

    /// Error opening snapshot file
    SnapshotOpen(io::Error),

    // Error copying snapshot into region
    SnapshotCopy(GuestMemoryError),

    /// Failed to allocate MMIO address
    AllocateMmioAddress,

    #[cfg(target_arch = "aarch64")]
    /// Failed to create UEFI flash
    CreateUefiFlash(HypervisorVmError),

    /// Using a directory as a backing file for memory is not supported
    DirectoryAsBackingFileForMemory,

    /// Failed to stat filesystem
    GetFileSystemBlockSize(io::Error),

    /// Memory size is misaligned with default page size or its hugepage size
    MisalignedMemorySize,
}

const ENABLE_FLAG: usize = 0;
const INSERTING_FLAG: usize = 1;
const REMOVING_FLAG: usize = 2;
const EJECT_FLAG: usize = 3;

const BASE_OFFSET_LOW: u64 = 0;
const BASE_OFFSET_HIGH: u64 = 0x4;
const LENGTH_OFFSET_LOW: u64 = 0x8;
const LENGTH_OFFSET_HIGH: u64 = 0xC;
const STATUS_OFFSET: u64 = 0x14;
const SELECTION_OFFSET: u64 = 0;

// The MMIO address space size is subtracted with 64k. This is done for the
// following reasons:
//  - Reduce the addressable space size by at least 4k to workaround a Linux
//    bug when the VMM allocates devices at the end of the addressable space
//  - Windows requires the addressable space size to be 64k aligned
fn mmio_address_space_size(phys_bits: u8) -> u64 {
    (1 << phys_bits) - (1 << 16)
}

// The `statfs` function can get information of hugetlbfs, and the hugepage size is in the
// `f_bsize` field.
//
// See: https://github.com/torvalds/linux/blob/v6.3/fs/hugetlbfs/inode.c#L1169
fn statfs_get_bsize(path: &str) -> Result<u64, Error> {
    let path = std::ffi::CString::new(path).map_err(|_| Error::InvalidMemoryParameters)?;
    let mut buf = std::mem::MaybeUninit::<libc::statfs>::uninit();

    // SAFETY: FFI call with a valid path and buffer
    let ret = unsafe { libc::statfs(path.as_ptr(), buf.as_mut_ptr()) };
    if ret != 0 {
        return Err(Error::GetFileSystemBlockSize(
            std::io::Error::last_os_error(),
        ));
    }

    // SAFETY: `buf` is valid at this point
    // Because this value is always positive, just convert it directly.
    // Note that the `f_bsize` is `i64` in glibc and `u64` in musl, using `as u64` will be warned
    // by `clippy` on musl target.  To avoid the warning, there should be `as _` instead of
    // `as u64`.
    let bsize = unsafe { (*buf.as_ptr()).f_bsize } as _;
    Ok(bsize)
}

fn memory_zone_get_align_size(zone: &MemoryZoneConfig) -> Result<u64, Error> {
    // SAFETY: FFI call. Trivially safe.
    let page_size = unsafe { libc::sysconf(libc::_SC_PAGESIZE) as u64 };

    // There is no backend file and the `hugepages` is disabled, just use system page size.
    if zone.file.is_none() && !zone.hugepages {
        return Ok(page_size);
    }

    // The `hugepages` is enabled and the `hugepage_size` is specified, just use it directly.
    if zone.hugepages && zone.hugepage_size.is_some() {
        return Ok(zone.hugepage_size.unwrap());
    }

    // There are two scenarios here:
    //  - `hugepages` is enabled but `hugepage_size` is not specified:
    //     Call `statfs` for `/dev/hugepages` for getting the default size of hugepage
    //  - The backing file is specified:
    //     Call `statfs` for the file and get its `f_bsize`.  If the value is larger than the page
    //     size of normal page, just use the `f_bsize` because the file is in a hugetlbfs.  If the
    //     value is less than or equal to the page size, just use the page size.
    let path = zone.file.as_ref().map_or(Ok("/dev/hugepages"), |pathbuf| {
        pathbuf.to_str().ok_or(Error::InvalidMemoryParameters)
    })?;

    let align_size = std::cmp::max(page_size, statfs_get_bsize(path)?);

    Ok(align_size)
}

#[inline]
fn align_down<T>(val: T, align: T) -> T
where
    T: BitAnd<Output = T> + Not<Output = T> + Sub<Output = T> + From<u8>,
{
    val & !(align - 1u8.into())
}

#[inline]
fn is_aligned<T>(val: T, align: T) -> bool
where
    T: BitAnd<Output = T> + Sub<Output = T> + From<u8> + PartialEq,
{
    (val & (align - 1u8.into())) == 0u8.into()
}

impl BusDevice for MemoryManager {
    fn read(&mut self, _base: u64, offset: u64, data: &mut [u8]) {
        if self.selected_slot < self.hotplug_slots.len() {
            let state = &self.hotplug_slots[self.selected_slot];
            match offset {
                BASE_OFFSET_LOW => {
                    data.copy_from_slice(&state.base.to_le_bytes()[..4]);
                }
                BASE_OFFSET_HIGH => {
                    data.copy_from_slice(&state.base.to_le_bytes()[4..]);
                }
                LENGTH_OFFSET_LOW => {
                    data.copy_from_slice(&state.length.to_le_bytes()[..4]);
                }
                LENGTH_OFFSET_HIGH => {
                    data.copy_from_slice(&state.length.to_le_bytes()[4..]);
                }
                STATUS_OFFSET => {
                    // The Linux kernel, quite reasonably, doesn't zero the memory it gives us.
                    data.fill(0);
                    if state.active {
                        data[0] |= 1 << ENABLE_FLAG;
                    }
                    if state.inserting {
                        data[0] |= 1 << INSERTING_FLAG;
                    }
                    if state.removing {
                        data[0] |= 1 << REMOVING_FLAG;
                    }
                }
                _ => {
                    warn!(
                        "Unexpected offset for accessing memory manager device: {:#}",
                        offset
                    );
                }
            }
        } else {
            warn!("Out of range memory slot: {}", self.selected_slot);
        }
    }

    fn write(&mut self, _base: u64, offset: u64, data: &[u8]) -> Option<Arc<Barrier>> {
        match offset {
            SELECTION_OFFSET => {
                self.selected_slot = usize::from(data[0]);
            }
            STATUS_OFFSET => {
                if self.selected_slot < self.hotplug_slots.len() {
                    let state = &mut self.hotplug_slots[self.selected_slot];
                    // The ACPI code writes back a 1 to acknowledge the insertion
                    if (data[0] & (1 << INSERTING_FLAG) == 1 << INSERTING_FLAG) && state.inserting {
                        state.inserting = false;
                    }
                    // Ditto for removal
                    if (data[0] & (1 << REMOVING_FLAG) == 1 << REMOVING_FLAG) && state.removing {
                        state.removing = false;
                    }
                    // Trigger removal of "DIMM"
                    if data[0] & (1 << EJECT_FLAG) == 1 << EJECT_FLAG {
                        warn!("Ejection of memory not currently supported");
                    }
                } else {
                    warn!("Out of range memory slot: {}", self.selected_slot);
                }
            }
            _ => {
                warn!(
                    "Unexpected offset for accessing memory manager device: {:#}",
                    offset
                );
            }
        };
        None
    }
}

impl MemoryManager {
    /// Creates all memory regions based on the available RAM ranges defined
    /// by `ram_regions`, and based on the description of the memory zones.
    /// In practice, this function can perform multiple memory mappings of the
    /// same backing file if there's a hole in the address space between two
    /// RAM ranges.
    /// One example might be ram_regions containing 2 regions (0-3G and 4G-6G)
    /// and zones containing two zones (size 1G and size 4G).
    /// This function will create 3 resulting memory regions:
    /// - First one mapping entirely the first memory zone on 0-1G range
    /// - Second one mapping partially the second memory zone on 1G-3G range
    /// - Third one mapping partially the second memory zone on 4G-6G range
    /// Also, all memory regions are page-size aligned (e.g. their sizes must
    /// be multiple of page-size), which may leave an additional hole in the
    /// address space when hugepage is used.
    fn create_memory_regions_from_zones(
        ram_regions: &[(GuestAddress, usize)],
        zones: &[MemoryZoneConfig],
        prefault: Option<bool>,
        thp: bool,
    ) -> Result<(Vec<Arc<GuestRegionMmap>>, MemoryZones), Error> {
        let mut zone_iter = zones.iter();
        let mut mem_regions = Vec::new();
        let mut zone = zone_iter.next().ok_or(Error::MissingMemoryZones)?;
        let mut zone_align_size = memory_zone_get_align_size(zone)?;
        let mut zone_offset = 0u64;
        let mut memory_zones = HashMap::new();

        if !is_aligned(zone.size, zone_align_size) {
            return Err(Error::MisalignedMemorySize);
        }

        // Add zone id to the list of memory zones.
        memory_zones.insert(zone.id.clone(), MemoryZone::default());

        for ram_region in ram_regions.iter() {
            let mut ram_region_offset = 0;
            let mut exit = false;

            loop {
                let mut ram_region_consumed = false;
                let mut pull_next_zone = false;

                let ram_region_available_size =
                    align_down(ram_region.1 as u64 - ram_region_offset, zone_align_size);
                if ram_region_available_size == 0 {
                    break;
                }
                let zone_sub_size = zone.size - zone_offset;

                let file_offset = zone_offset;
                let region_start = ram_region
                    .0
                    .checked_add(ram_region_offset)
                    .ok_or(Error::GuestAddressOverFlow)?;
                let region_size = if zone_sub_size <= ram_region_available_size {
                    if zone_sub_size == ram_region_available_size {
                        ram_region_consumed = true;
                    }

                    ram_region_offset += zone_sub_size;
                    pull_next_zone = true;

                    zone_sub_size
                } else {
                    zone_offset += ram_region_available_size;
                    ram_region_consumed = true;

                    ram_region_available_size
                };

                info!(
                    "create ram region for zone {}, region_start: {:#x}, region_size: {:#x}",
                    zone.id,
                    region_start.raw_value(),
                    region_size
                );
                let region = MemoryManager::create_ram_region(
                    &zone.file,
                    file_offset,
                    region_start,
                    region_size as usize,
                    prefault.unwrap_or(zone.prefault),
                    zone.shared,
                    zone.hugepages,
                    zone.hugepage_size,
                    zone.host_numa_node,
                    None,
                    thp,
                )?;

                // Add region to the list of regions associated with the
                // current memory zone.
                if let Some(memory_zone) = memory_zones.get_mut(&zone.id) {
                    memory_zone.regions.push(region.clone());
                }

                mem_regions.push(region);

                if pull_next_zone {
                    // Get the next zone and reset the offset.
                    zone_offset = 0;
                    if let Some(z) = zone_iter.next() {
                        zone = z;
                    } else {
                        exit = true;
                        break;
                    }
                    zone_align_size = memory_zone_get_align_size(zone)?;
                    if !is_aligned(zone.size, zone_align_size) {
                        return Err(Error::MisalignedMemorySize);
                    }

                    // Check if zone id already exist. In case it does, throw
                    // an error as we need unique identifiers. Otherwise, add
                    // the new zone id to the list of memory zones.
                    if memory_zones.contains_key(&zone.id) {
                        error!(
                            "Memory zone identifier '{}' found more than once. \
                            It must be unique",
                            zone.id,
                        );
                        return Err(Error::DuplicateZoneId);
                    }
                    memory_zones.insert(zone.id.clone(), MemoryZone::default());
                }

                if ram_region_consumed {
                    break;
                }
            }

            if exit {
                break;
            }
        }

        Ok((mem_regions, memory_zones))
    }

    fn validate_memory_config(
        config: &MemoryConfig,
        user_provided_zones: bool,
    ) -> Result<(u64, Vec<MemoryZoneConfig>, bool), Error> {
        let mut allow_mem_hotplug = false;

        if !user_provided_zones {
            if config.zones.is_some() {
                error!(
                    "User defined memory regions can't be provided if the \
                    memory size is not 0"
                );
                return Err(Error::InvalidMemoryParameters);
            }

            if config.hotplug_size.is_some() {
                allow_mem_hotplug = true;
            }

            // Create a single zone from the global memory config. This lets
            // us reuse the codepath for user defined memory zones.
            let zones = vec![MemoryZoneConfig {
                id: String::from(DEFAULT_MEMORY_ZONE),
                size: config.size,
                file: None,
                shared: config.shared,
                hugepages: config.hugepages,
                hugepage_size: config.hugepage_size,
                hotplugged_size: config.hotplugged_size,
                host_numa_node: None,
                hotplug_size: config.hotplug_size,
                prefault: config.prefault,
            }];

            Ok((config.size, zones, allow_mem_hotplug))
        } else {
            if config.zones.is_none() {
                error!(
                    "User defined memory regions must be provided if the \
                    memory size is 0"
                );
                return Err(Error::MissingMemoryZones);
            }

            // Safe to unwrap as we checked right above there were some
            // regions.
            let zones = config.zones.clone().unwrap();
            if zones.is_empty() {
                return Err(Error::MissingMemoryZones);
            }

            let mut total_ram_size: u64 = 0;
            for zone in zones.iter() {
                total_ram_size += zone.size;

                if zone.shared && zone.file.is_some() && zone.host_numa_node.is_some() {
                    error!(
                        "Invalid to set host NUMA policy for a memory zone \
                        backed by a regular file and mapped as 'shared'"
                    );
                    return Err(Error::InvalidSharedMemoryZoneWithHostNuma);
                }
            }

            Ok((total_ram_size, zones, allow_mem_hotplug))
        }
    }

    pub fn allocate_address_space(&mut self) -> Result<(), Error> {
        let mut list = Vec::new();

        for (zone_id, memory_zone) in self.memory_zones.iter() {
            let regions: Vec<(Arc<vm_memory::GuestRegionMmap<AtomicBitmap>>, bool)> =
                memory_zone
                    .regions()
                    .iter()
                    .map(|r| (r.clone(), false))
                    .collect();

            list.push((zone_id.clone(), regions));
        }

        for (zone_id, regions) in list {
            for (region, virtio_mem) in regions {
                let slot = self.create_userspace_mapping(
                    region.start_addr().raw_value(),
                    region.len(),
                    region.as_ptr() as u64,
                    self.mergeable,
                    false,
                    self.log_dirty,
                )?;

                let file_offset = if let Some(file_offset) = region.file_offset() {
                    file_offset.start()
                } else {
                    0
                };

                self.guest_ram_mappings.push(GuestRamMapping {
                    gpa: region.start_addr().raw_value(),
                    size: region.len(),
                    slot,
                    zone_id: zone_id.clone(),
                    virtio_mem,
                    file_offset,
                });
                self.ram_allocator
                    .allocate(Some(region.start_addr()), region.len(), None)
                    .ok_or(Error::MemoryRangeAllocation)?;
            }
        }

        // Allocate SubRegion and Reserved address ranges.
        for region in self.arch_mem_regions.iter() {
            if region.r_type == RegionType::Ram {
                // Ignore the RAM type since ranges have already been allocated
                // based on the GuestMemory regions.
                continue;
            }
            self.ram_allocator
                .allocate(
                    Some(GuestAddress(region.base)),
                    region.size as GuestUsize,
                    None,
                )
                .ok_or(Error::MemoryRangeAllocation)?;
        }

        Ok(())
    }

    #[cfg(target_arch = "aarch64")]
    fn add_uefi_flash(&mut self) -> Result<(), Error> {
        // On AArch64, the UEFI binary requires a flash device at address 0.
        // 4 MiB memory is mapped to simulate the flash.
        let uefi_mem_slot = self.allocate_memory_slot();
        let uefi_region = GuestRegionMmap::new(
            MmapRegion::new(arch::layout::UEFI_SIZE as usize).unwrap(),
            arch::layout::UEFI_START,
        )
        .unwrap();
        let uefi_mem_region = self.vm.make_user_memory_region(
            uefi_mem_slot,
            uefi_region.start_addr().raw_value(),
            uefi_region.len(),
            uefi_region.as_ptr() as u64,
            false,
            false,
        );
        self.vm
            .create_user_memory_region(uefi_mem_region)
            .map_err(Error::CreateUefiFlash)?;

        let uefi_flash =
            GuestMemoryAtomic::new(GuestMemoryMmap::from_regions(vec![uefi_region]).unwrap());

        self.uefi_flash = Some(uefi_flash);

        Ok(())
    }

    #[allow(clippy::too_many_arguments)]
    pub fn new(
        vm: Arc<dyn hypervisor::Vm>,
        config: &MemoryConfig,
        prefault: Option<bool>,
        phys_bits: u8,
        #[cfg(target_arch = "x86_64")] sgx_epc_config: Option<Vec<SgxEpcConfig>>,
    ) -> Result<Arc<Mutex<MemoryManager>>, Error> {
        let user_provided_zones = config.size == 0;

        let mmio_address_space_size = mmio_address_space_size(phys_bits);
        debug_assert_eq!(
            (((mmio_address_space_size) >> 16) << 16),
            mmio_address_space_size
        );
        let start_of_platform_device_area =
            GuestAddress(mmio_address_space_size - PLATFORM_DEVICE_AREA_SIZE);
        let end_of_device_area = start_of_platform_device_area.unchecked_sub(1);

        let (_ram_size, zones, allow_mem_hotplug) =
            Self::validate_memory_config(config, user_provided_zones)?;

        let (
            start_of_device_area,
            arch_mem_regions,
            memory_zones,
            guest_memory,
            boot_guest_memory,
            hotplug_slots,
            next_memory_slot,
            selected_slot,
        ) = {
            // Init guest memory
            let arch_mem_regions = arch::arch_memory_regions();

            let ram_regions: Vec<(GuestAddress, usize)> = arch_mem_regions
                .iter()
                .filter(|r| r.2 == RegionType::Ram)
                .map(|r| (r.0, r.1))
                .collect();

            let arch_mem_regions: Vec<ArchMemRegion> = arch_mem_regions
                .iter()
                .map(|(a, b, c)| ArchMemRegion {
                    base: a.0,
                    size: *b,
                    r_type: *c,
                })
                .collect();

            let (mem_regions, memory_zones) =
                Self::create_memory_regions_from_zones(&ram_regions, &zones, prefault, config.thp)?;

            let guest_memory =
                GuestMemoryMmap::from_arc_regions(mem_regions).map_err(Error::GuestMemory)?;

            let boot_guest_memory = guest_memory.clone();

            let start_of_device_area =
                MemoryManager::start_addr(guest_memory.last_addr(), allow_mem_hotplug)?;

            let mut hotplug_slots = Vec::with_capacity(HOTPLUG_COUNT);
            hotplug_slots.resize_with(HOTPLUG_COUNT, HotPlugState::default);

            (
                start_of_device_area,
                arch_mem_regions,
                memory_zones,
                guest_memory,
                boot_guest_memory,
                hotplug_slots,
                0,
                0,
            )
        };

        let guest_memory = GuestMemoryAtomic::new(guest_memory);

        // Both MMIO and PIO address spaces start at address 0.
        let allocator = Arc::new(Mutex::new(
            SystemAllocator::new(
                #[cfg(target_arch = "x86_64")]
                {
                    GuestAddress(0)
                },
                #[cfg(target_arch = "x86_64")]
                {
                    1 << 16
                },
                start_of_platform_device_area,
                PLATFORM_DEVICE_AREA_SIZE,
                #[cfg(target_arch = "x86_64")]
                vec![GsiApic::new(
                    X86_64_IRQ_BASE,
                    ioapic::NUM_IOAPIC_PINS as u32 - X86_64_IRQ_BASE,
                )],
            )
            .ok_or(Error::CreateSystemAllocator)?,
        ));

        #[cfg(not(feature = "tdx"))]
        let dynamic = true;
        #[cfg(feature = "tdx")]
        let dynamic = !tdx_enabled;

        let acpi_address = None;

        let ram_allocator = AddressAllocator::new(GuestAddress(0), start_of_device_area.0).unwrap();

        let mut memory_manager = MemoryManager {
            boot_guest_memory,
            guest_memory,
            next_memory_slot,
            start_of_device_area,
            end_of_device_area,
            vm,
            hotplug_slots,
            selected_slot,
            mergeable: config.mergeable,
            allocator,
            shared: config.shared,
            hugepages: config.hugepages,
            hugepage_size: config.hugepage_size,
            prefault: config.prefault,
            #[cfg(target_arch = "x86_64")]
            sgx_epc_region: None,
            memory_zones,
            guest_ram_mappings: Vec::new(),
            acpi_address,
            log_dirty: dynamic, // Cannot log dirty pages on a TD
            arch_mem_regions,
            ram_allocator,
            #[cfg(target_arch = "aarch64")]
            uefi_flash: None,
            thp: config.thp,
        };

        #[cfg(target_arch = "aarch64")]
        {
            // For Aarch64 we cannot lazily allocate the address space like we
            // do for x86, because while restoring a VM from snapshot we would
            // need the address space to be allocated to properly restore VGIC.
            // And the restore of VGIC happens before we attempt to run the vCPUs
            // for the first time, thus we need to allocate the address space
            // beforehand.
            memory_manager.allocate_address_space()?;
            memory_manager.add_uefi_flash()?;
        }

        #[cfg(target_arch = "x86_64")]
        if let Some(sgx_epc_config) = sgx_epc_config {
            memory_manager.setup_sgx(sgx_epc_config)?;
        }

        Ok(Arc::new(Mutex::new(memory_manager)))
    }

    fn memfd_create(name: &ffi::CStr, flags: u32) -> Result<RawFd, io::Error> {
        // SAFETY: FFI call with correct arguments
        let res = unsafe { libc::syscall(libc::SYS_memfd_create, name.as_ptr(), flags) };

        if res < 0 {
            Err(io::Error::last_os_error())
        } else {
            Ok(res as RawFd)
        }
    }

    fn mbind(
        addr: *mut u8,
        len: u64,
        mode: u32,
        nodemask: Vec<u64>,
        maxnode: u64,
        flags: u32,
    ) -> Result<(), io::Error> {
        // SAFETY: FFI call with correct arguments
        let res = unsafe {
            libc::syscall(
                libc::SYS_mbind,
                addr as *mut libc::c_void,
                len,
                mode,
                nodemask.as_ptr(),
                maxnode,
                flags,
            )
        };

        if res < 0 {
            Err(io::Error::last_os_error())
        } else {
            Ok(())
        }
    }

    fn create_anonymous_file(
        size: usize,
        hugepages: bool,
        hugepage_size: Option<u64>,
    ) -> Result<FileOffset, Error> {
        let fd = Self::memfd_create(
            &ffi::CString::new("ch_ram").unwrap(),
            libc::MFD_CLOEXEC
                | if hugepages {
                    libc::MFD_HUGETLB
                        | if let Some(hugepage_size) = hugepage_size {
                            /*
                             * From the Linux kernel:
                             * Several system calls take a flag to request "hugetlb" huge pages.
                             * Without further specification, these system calls will use the
                             * system's default huge page size.  If a system supports multiple
                             * huge page sizes, the desired huge page size can be specified in
                             * bits [26:31] of the flag arguments.  The value in these 6 bits
                             * will encode the log2 of the huge page size.
                             */

                            hugepage_size.trailing_zeros() << 26
                        } else {
                            // Use the system default huge page size
                            0
                        }
                } else {
                    0
                },
        )
        .map_err(Error::SharedFileCreate)?;

        // SAFETY: fd is valid
        let f = unsafe { File::from_raw_fd(fd) };
        f.set_len(size as u64).map_err(Error::SharedFileSetLen)?;

        Ok(FileOffset::new(f, 0))
    }

    fn open_backing_file(backing_file: &PathBuf, file_offset: u64) -> Result<FileOffset, Error> {
        if backing_file.is_dir() {
            Err(Error::DirectoryAsBackingFileForMemory)
        } else {
            let f = OpenOptions::new()
                .read(true)
                .write(true)
                .open(backing_file)
                .map_err(Error::SharedFileCreate)?;

            Ok(FileOffset::new(f, file_offset))
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub fn create_ram_region(
        backing_file: &Option<PathBuf>,
        file_offset: u64,
        start_addr: GuestAddress,
        size: usize,
        prefault: bool,
        shared: bool,
        hugepages: bool,
        hugepage_size: Option<u64>,
        host_numa_node: Option<u32>,
        existing_memory_file: Option<File>,
        thp: bool,
    ) -> Result<Arc<GuestRegionMmap>, Error> {
        let mut mmap_flags = libc::MAP_NORESERVE;

        // The duplication of mmap_flags ORing here is unfortunate but it also makes
        // the complexity of the handling clear.
        let fo = if let Some(f) = existing_memory_file {
            // It must be MAP_SHARED as we wouldn't already have an FD
            mmap_flags |= libc::MAP_SHARED;
            Some(FileOffset::new(f, file_offset))
        } else if let Some(backing_file) = backing_file {
            if shared {
                mmap_flags |= libc::MAP_SHARED;
            } else {
                mmap_flags |= libc::MAP_PRIVATE;
            }
            Some(Self::open_backing_file(backing_file, file_offset)?)
        } else if shared || hugepages {
            // For hugepages we must also MAP_SHARED otherwise we will trigger #4805
            // because the MAP_PRIVATE will trigger CoW against the backing file with
            // the VFIO pinning
            mmap_flags |= libc::MAP_SHARED;
            Some(Self::create_anonymous_file(size, hugepages, hugepage_size)?)
        } else {
            mmap_flags |= libc::MAP_PRIVATE | libc::MAP_ANONYMOUS;
            None
        };

        let region = GuestRegionMmap::new(
            MmapRegion::build(fo, size, libc::PROT_READ | libc::PROT_WRITE, mmap_flags)
                .map_err(Error::GuestMemoryRegion)?,
            start_addr,
        )
        .map_err(Error::GuestMemory)?;

        // Apply NUMA policy if needed.
        if let Some(node) = host_numa_node {
            let addr = region.deref().as_ptr();
            let len = region.deref().size() as u64;
            let mode = MPOL_BIND;
            let mut nodemask: Vec<u64> = Vec::new();
            let flags = MPOL_MF_STRICT | MPOL_MF_MOVE;

            // Linux is kind of buggy in the way it interprets maxnode as it
            // will cut off the last node. That's why we have to add 1 to what
            // we would consider as the proper maxnode value.
            let maxnode = node as u64 + 1 + 1;

            // Allocate the right size for the vector.
            nodemask.resize((node as usize / 64) + 1, 0);

            // Fill the global bitmask through the nodemask vector.
            let idx = (node / 64) as usize;
            let shift = node % 64;
            nodemask[idx] |= 1u64 << shift;

            // Policies are enforced by using MPOL_MF_MOVE flag as it will
            // force the kernel to move all pages that might have been already
            // allocated to the proper set of NUMA nodes. MPOL_MF_STRICT is
            // used to throw an error if MPOL_MF_MOVE didn't succeed.
            // MPOL_BIND is the selected mode as it specifies a strict policy
            // that restricts memory allocation to the nodes specified in the
            // nodemask.
            Self::mbind(addr, len, mode, nodemask, maxnode, flags)
                .map_err(Error::ApplyNumaPolicy)?;
        }

        // Prefault the region if needed, in parallel.
        if prefault {
            let page_size =
                Self::get_prefault_align_size(backing_file, hugepages, hugepage_size)? as usize;

            if !is_aligned(size, page_size) {
                warn!(
                    "Prefaulting memory size {} misaligned with page size {}",
                    size, page_size
                );
            }

            let num_pages = size / page_size;

            let num_threads = Self::get_prefault_num_threads(page_size, num_pages);

            let pages_per_thread = num_pages / num_threads;
            let remainder = num_pages % num_threads;

            let barrier = Arc::new(Barrier::new(num_threads));
            thread::scope(|s| {
                let r = &region;
                for i in 0..num_threads {
                    let barrier = Arc::clone(&barrier);
                    s.spawn(move || {
                        // Wait until all threads have been spawned to avoid contention
                        // over mmap_sem between thread stack allocation and page faulting.
                        barrier.wait();
                        let pages = pages_per_thread + if i < remainder { 1 } else { 0 };
                        let offset =
                            page_size * ((i * pages_per_thread) + std::cmp::min(i, remainder));
                        // SAFETY: FFI call with correct arguments
                        let ret = unsafe {
                            let addr = r.as_ptr().add(offset);
                            libc::madvise(addr as _, pages * page_size, libc::MADV_POPULATE_WRITE)
                        };
                        if ret != 0 {
                            let e = io::Error::last_os_error();
                            warn!("Failed to prefault pages: {}", e);
                        }
                    });
                }
            });
        }

        if region.file_offset().is_none() && thp {
            info!(
                "Anonymous mapping at 0x{:x} (size = 0x{:x})",
                region.as_ptr() as u64,
                size
            );
            // SAFETY: FFI call with correct arguments
            let ret = unsafe { libc::madvise(region.as_ptr() as _, size, libc::MADV_HUGEPAGE) };
            if ret != 0 {
                let e = io::Error::last_os_error();
                warn!("Failed to mark pages as THP eligible: {}", e);
            }
        }

        Ok(Arc::new(region))
    }

    // Duplicate of `memory_zone_get_align_size` that does not require a `zone`
    fn get_prefault_align_size(
        backing_file: &Option<PathBuf>,
        hugepages: bool,
        hugepage_size: Option<u64>,
    ) -> Result<u64, Error> {
        // SAFETY: FFI call. Trivially safe.
        let page_size = unsafe { libc::sysconf(libc::_SC_PAGESIZE) as u64 };
        match (hugepages, hugepage_size, backing_file) {
            (false, _, _) => Ok(page_size),
            (true, Some(hugepage_size), _) => Ok(hugepage_size),
            (true, None, _) => {
                // There are two scenarios here:
                //  - `hugepages` is enabled but `hugepage_size` is not specified:
                //     Call `statfs` for `/dev/hugepages` for getting the default size of hugepage
                //  - The backing file is specified:
                //     Call `statfs` for the file and get its `f_bsize`.  If the value is larger than the page
                //     size of normal page, just use the `f_bsize` because the file is in a hugetlbfs.  If the
                //     value is less than or equal to the page size, just use the page size.
                let path = backing_file
                    .as_ref()
                    .map_or(Ok("/dev/hugepages"), |pathbuf| {
                        pathbuf.to_str().ok_or(Error::InvalidMemoryParameters)
                    })?;
                let align_size = std::cmp::max(page_size, statfs_get_bsize(path)?);
                Ok(align_size)
            }
        }
    }

    fn get_prefault_num_threads(page_size: usize, num_pages: usize) -> usize {
        let mut n: usize = 1;

        // Do not create more threads than processors available.
        // SAFETY: FFI call. Trivially safe.
        let procs = unsafe { libc::sysconf(_SC_NPROCESSORS_ONLN) };
        if procs > 0 {
            n = std::cmp::min(procs as usize, MAX_PREFAULT_THREAD_COUNT);
        }

        // Do not create more threads than pages being allocated.
        n = std::cmp::min(n, num_pages);

        // Do not create threads to allocate less than 64 MiB of memory.
        n = std::cmp::min(
            n,
            std::cmp::max(1, page_size * num_pages / (64 * (1 << 26))),
        );

        n
    }

    // Update the GuestMemoryMmap with the new range
    fn add_region(&mut self, region: Arc<GuestRegionMmap>) -> Result<(), Error> {
        let guest_memory = self
            .guest_memory
            .memory()
            .insert_region(region)
            .map_err(Error::GuestMemory)?;
        self.guest_memory.lock().unwrap().replace(guest_memory);

        Ok(())
    }

    //
    // Calculate the start address of an area next to RAM.
    //
    // If memory hotplug is allowed, the start address needs to be aligned
    // (rounded-up) to 128MiB boundary.
    // If memory hotplug is not allowed, there is no alignment required.
    // And it must also start at the 64bit start.
    fn start_addr(mem_end: GuestAddress, allow_mem_hotplug: bool) -> Result<GuestAddress, Error> {
        let mut start_addr = if allow_mem_hotplug {
            GuestAddress(mem_end.0 | ((128 << 20) - 1))
        } else {
            mem_end
        };

        start_addr = start_addr
            .checked_add(1)
            .ok_or(Error::GuestAddressOverFlow)?;

        if mem_end < arch::layout::MEM_32BIT_RESERVED_START {
            return Ok(arch::layout::RAM_64BIT_START);
        }

        Ok(start_addr)
    }

    pub fn add_ram_region(
        &mut self,
        start_addr: GuestAddress,
        size: usize,
    ) -> Result<Arc<GuestRegionMmap>, Error> {
        // Allocate memory for the region
        let region = MemoryManager::create_ram_region(
            &None,
            0,
            start_addr,
            size,
            self.prefault,
            self.shared,
            self.hugepages,
            self.hugepage_size,
            None,
            None,
            self.thp,
        )?;

        // Map it into the guest
        let slot = self.create_userspace_mapping(
            region.start_addr().0,
            region.len(),
            region.as_ptr() as u64,
            self.mergeable,
            false,
            self.log_dirty,
        )?;
        self.guest_ram_mappings.push(GuestRamMapping {
            gpa: region.start_addr().raw_value(),
            size: region.len(),
            slot,
            zone_id: DEFAULT_MEMORY_ZONE.to_string(),
            virtio_mem: false,
            file_offset: 0,
        });

        self.add_region(Arc::clone(&region))?;

        Ok(region)
    }

    pub fn guest_memory(&self) -> GuestMemoryAtomic<GuestMemoryMmap> {
        self.guest_memory.clone()
    }

    pub fn boot_guest_memory(&self) -> GuestMemoryMmap {
        self.boot_guest_memory.clone()
    }

    pub fn allocator(&self) -> Arc<Mutex<SystemAllocator>> {
        self.allocator.clone()
    }

    pub fn start_of_device_area(&self) -> GuestAddress {
        self.start_of_device_area
    }

    pub fn end_of_device_area(&self) -> GuestAddress {
        self.end_of_device_area
    }

    pub fn allocate_memory_slot(&mut self) -> u32 {
        let slot_id = self.next_memory_slot;
        self.next_memory_slot += 1;
        slot_id
    }

    pub fn create_userspace_mapping(
        &mut self,
        guest_phys_addr: u64,
        memory_size: u64,
        userspace_addr: u64,
        mergeable: bool,
        readonly: bool,
        log_dirty: bool,
    ) -> Result<u32, Error> {
        let slot = self.allocate_memory_slot();
        let mem_region = self.vm.make_user_memory_region(
            slot,
            guest_phys_addr,
            memory_size,
            userspace_addr,
            readonly,
            log_dirty,
        );

        info!(
            "Creating userspace mapping: {:x} -> {:x} {:x}, slot {}",
            guest_phys_addr, userspace_addr, memory_size, slot
        );

        self.vm
            .create_user_memory_region(mem_region)
            .map_err(Error::CreateUserMemoryRegion)?;

        // SAFETY: the address and size are valid since the
        // mmap succeeded.
        let ret = unsafe {
            libc::madvise(
                userspace_addr as *mut libc::c_void,
                memory_size as libc::size_t,
                libc::MADV_DONTDUMP,
            )
        };
        if ret != 0 {
            let e = io::Error::last_os_error();
            warn!("Failed to mark mappin as MADV_DONTDUMP: {}", e);
        }

        // Mark the pages as mergeable if explicitly asked for.
        if mergeable {
            // SAFETY: the address and size are valid since the
            // mmap succeeded.
            let ret = unsafe {
                libc::madvise(
                    userspace_addr as *mut libc::c_void,
                    memory_size as libc::size_t,
                    libc::MADV_MERGEABLE,
                )
            };
            if ret != 0 {
                let err = io::Error::last_os_error();
                // Safe to unwrap because the error is constructed with
                // last_os_error(), which ensures the output will be Some().
                let errno = err.raw_os_error().unwrap();
                if errno == libc::EINVAL {
                    warn!("kernel not configured with CONFIG_KSM");
                } else {
                    warn!("madvise error: {}", err);
                }
                warn!("failed to mark pages as mergeable");
            }
        }

        info!(
            "Created userspace mapping: {:x} -> {:x} {:x}",
            guest_phys_addr, userspace_addr, memory_size
        );

        Ok(slot)
    }

    pub fn remove_userspace_mapping(
        &mut self,
        guest_phys_addr: u64,
        memory_size: u64,
        userspace_addr: u64,
        mergeable: bool,
        slot: u32,
    ) -> Result<(), Error> {
        let mem_region = self.vm.make_user_memory_region(
            slot,
            guest_phys_addr,
            memory_size,
            userspace_addr,
            false, /* readonly -- don't care */
            false, /* log dirty */
        );

        self.vm
            .remove_user_memory_region(mem_region)
            .map_err(Error::RemoveUserMemoryRegion)?;

        // Mark the pages as unmergeable if there were previously marked as
        // mergeable.
        if mergeable {
            // SAFETY: the address and size are valid as the region was
            // previously advised.
            let ret = unsafe {
                libc::madvise(
                    userspace_addr as *mut libc::c_void,
                    memory_size as libc::size_t,
                    libc::MADV_UNMERGEABLE,
                )
            };
            if ret != 0 {
                let err = io::Error::last_os_error();
                // Safe to unwrap because the error is constructed with
                // last_os_error(), which ensures the output will be Some().
                let errno = err.raw_os_error().unwrap();
                if errno == libc::EINVAL {
                    warn!("kernel not configured with CONFIG_KSM");
                } else {
                    warn!("madvise error: {}", err);
                }
                warn!("failed to mark pages as unmergeable");
            }
        }

        info!(
            "Removed userspace mapping: {:x} -> {:x} {:x}",
            guest_phys_addr, userspace_addr, memory_size
        );

        Ok(())
    }

    #[cfg(target_arch = "x86_64")]
    pub fn setup_sgx(&mut self, sgx_epc_config: Vec<SgxEpcConfig>) -> Result<(), Error> {
        let file = OpenOptions::new()
            .read(true)
            .open("/dev/sgx_provision")
            .map_err(Error::SgxProvisionOpen)?;
        self.vm
            .enable_sgx_attribute(file)
            .map_err(Error::SgxEnableProvisioning)?;

        // Go over each EPC section and verify its size is a 4k multiple. At
        // the same time, calculate the total size needed for the contiguous
        // EPC region.
        let mut epc_region_size = 0;
        for epc_section in sgx_epc_config.iter() {
            if epc_section.size == 0 {
                return Err(Error::EpcSectionSizeInvalid);
            }
            if epc_section.size & (SGX_PAGE_SIZE - 1) != 0 {
                return Err(Error::EpcSectionSizeInvalid);
            }

            epc_region_size += epc_section.size;
        }

        // Place the SGX EPC region on a 4k boundary between the RAM and the device area
        let epc_region_start = GuestAddress(
            ((self.start_of_device_area.0 + SGX_PAGE_SIZE - 1) / SGX_PAGE_SIZE) * SGX_PAGE_SIZE,
        );

        self.start_of_device_area = epc_region_start
            .checked_add(epc_region_size)
            .ok_or(Error::GuestAddressOverFlow)?;

        let mut sgx_epc_region = SgxEpcRegion::new(epc_region_start, epc_region_size as GuestUsize);
        info!(
            "SGX EPC region: 0x{:x} (0x{:x})",
            epc_region_start.0, epc_region_size
        );

        // Each section can be memory mapped into the allocated region.
        let mut epc_section_start = epc_region_start.raw_value();
        for epc_section in sgx_epc_config.iter() {
            let file = OpenOptions::new()
                .read(true)
                .write(true)
                .open("/dev/sgx_vepc")
                .map_err(Error::SgxVirtEpcOpen)?;

            let prot = PROT_READ | PROT_WRITE;
            let mut flags = MAP_NORESERVE | MAP_SHARED;
            if epc_section.prefault {
                flags |= MAP_POPULATE;
            }

            // We can't use the vm-memory crate to perform the memory mapping
            // here as it would try to ensure the size of the backing file is
            // matching the size of the expected mapping. The /dev/sgx_vepc
            // device does not work that way, it provides a file descriptor
            // which is not matching the mapping size, as it's a just a way to
            // let KVM know that an EPC section is being created for the guest.
            // SAFETY: FFI call with correct arguments
            let host_addr = unsafe {
                libc::mmap(
                    std::ptr::null_mut(),
                    epc_section.size as usize,
                    prot,
                    flags,
                    file.as_raw_fd(),
                    0,
                )
            } as u64;

            info!(
                "Adding SGX EPC section: 0x{:x} (0x{:x})",
                epc_section_start, epc_section.size
            );

            let _mem_slot = self.create_userspace_mapping(
                epc_section_start,
                epc_section.size,
                host_addr,
                false,
                false,
                false,
            )?;

            sgx_epc_region.insert(
                epc_section.id.clone(),
                SgxEpcSection::new(
                    GuestAddress(epc_section_start),
                    epc_section.size as GuestUsize,
                ),
            );

            epc_section_start += epc_section.size;
        }

        self.sgx_epc_region = Some(sgx_epc_region);

        Ok(())
    }

    #[cfg(target_arch = "x86_64")]
    pub fn sgx_epc_region(&self) -> &Option<SgxEpcRegion> {
        &self.sgx_epc_region
    }

    pub fn is_hardlink(f: &File) -> bool {
        let mut stat = std::mem::MaybeUninit::<libc::stat>::uninit();
        // SAFETY: FFI call with correct arguments
        let ret = unsafe { libc::fstat(f.as_raw_fd(), stat.as_mut_ptr()) };
        if ret != 0 {
            error!("Couldn't fstat the backing file");
            return false;
        }

        // SAFETY: stat is valid
        unsafe { (*stat.as_ptr()).st_nlink as usize > 0 }
    }

    pub fn memory_zones(&self) -> &MemoryZones {
        &self.memory_zones
    }

    pub fn memory_zones_mut(&mut self) -> &mut MemoryZones {
        &mut self.memory_zones
    }

    pub fn memory_slot_fds(&self) -> HashMap<u32, RawFd> {
        let mut memory_slot_fds = HashMap::new();
        for guest_ram_mapping in &self.guest_ram_mappings {
            let slot = guest_ram_mapping.slot;
            let guest_memory = self.guest_memory.memory();
            let file = guest_memory
                .find_region(GuestAddress(guest_ram_mapping.gpa))
                .unwrap()
                .file_offset()
                .unwrap()
                .file();
            memory_slot_fds.insert(slot, file.as_raw_fd());
        }
        memory_slot_fds
    }

    pub fn acpi_address(&self) -> Option<GuestAddress> {
        self.acpi_address
    }

    pub fn num_guest_ram_mappings(&self) -> u32 {
        self.guest_ram_mappings.len() as u32
    }

    #[cfg(target_arch = "aarch64")]
    pub fn uefi_flash(&self) -> GuestMemoryAtomic<GuestMemoryMmap> {
        self.uefi_flash.as_ref().unwrap().clone()
    }
}
