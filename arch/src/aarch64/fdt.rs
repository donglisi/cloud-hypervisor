// Copyright 2020 Arm Limited (or its affiliates). All rights reserved.
// Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
//
// Portions Copyright 2017 The Chromium OS Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the THIRD-PARTY file.

use crate::{NumaNodes};
use byteorder::{BigEndian, ByteOrder};
use hypervisor::arch::aarch64::gic::Vgic;
use std::cmp;
use std::collections::HashMap;
use std::ffi::CStr;
use std::fmt::Debug;
use std::result;
use std::str;
use std::sync::{Arc, Mutex};

use super::super::DeviceType;
use super::super::GuestMemoryMmap;
use super::super::InitramfsConfig;
use super::layout::{
    IRQ_BASE,
};
use std::fs;
use std::path::Path;
use vm_fdt::{FdtWriter, FdtWriterResult};
use vm_memory::{Address, Bytes, GuestMemory, GuestMemoryError, GuestMemoryRegion};

// This is a value for uniquely identifying the FDT node declaring the interrupt controller.
const GIC_PHANDLE: u32 = 1;
// This is a value for uniquely identifying the FDT node declaring the MSI controller.
const MSI_PHANDLE: u32 = 2;
// This is a value for uniquely identifying the FDT node containing the clock definition.
const CLOCK_PHANDLE: u32 = 3;
// This is a value for uniquely identifying the FDT node containing the gpio controller.
const GPIO_PHANDLE: u32 = 4;
// NOTE: Keep FIRST_VCPU_PHANDLE the last PHANDLE defined.
// This is a value for uniquely identifying the FDT node containing the first vCPU.
// The last number of vCPU phandle depends on the number of vCPUs.
const FIRST_VCPU_PHANDLE: u32 = 8;

// This is a value for uniquely identifying the FDT node containing the L2 cache info
const L2_CACHE_PHANDLE: u32 = 6;
// This is a value for uniquely identifying the FDT node containing the L3 cache info
const L3_CACHE_PHANDLE: u32 = 7;
// Read the documentation specified when appending the root node to the FDT.
const ADDRESS_CELLS: u32 = 0x2;
const SIZE_CELLS: u32 = 0x2;

// As per kvm tool and
// https://www.kernel.org/doc/Documentation/devicetree/bindings/interrupt-controller/arm%2Cgic.txt
// Look for "The 1st cell..."
const GIC_FDT_IRQ_TYPE_SPI: u32 = 0;
const GIC_FDT_IRQ_TYPE_PPI: u32 = 1;

// From https://elixir.bootlin.com/linux/v4.9.62/source/include/dt-bindings/interrupt-controller/irq.h#L17
const IRQ_TYPE_EDGE_RISING: u32 = 1;
const IRQ_TYPE_LEVEL_HI: u32 = 4;

// PMU PPI interrupt number
pub const AARCH64_PMU_IRQ: u32 = 7;

// Keys and Buttons
// System Power Down
const KEY_POWER: u32 = 116;

/// Trait for devices to be added to the Flattened Device Tree.
pub trait DeviceInfoForFdt {
    /// Returns the address where this device will be loaded.
    fn addr(&self) -> u64;
    /// Returns the associated interrupt for this device.
    fn irq(&self) -> u32;
    /// Returns the amount of memory that needs to be reserved for this device.
    fn length(&self) -> u64;
}

/// Errors thrown while configuring the Flattened Device Tree for aarch64.
#[derive(Debug)]
pub enum Error {
    /// Failure in writing FDT in memory.
    WriteFdtToMemory(GuestMemoryError),
}
type Result<T> = result::Result<T, Error>;

pub enum CacheLevel {
    /// L1 data cache
    L1D = 0,
    /// L1 instruction cache
    L1I = 1,
    /// L2 cache
    L2 = 2,
    /// L3 cache
    L3 = 3,
}

/// NOTE: cache size file directory example,
/// "/sys/devices/system/cpu/cpu0/cache/index0/size".
pub fn get_cache_size(cache_level: CacheLevel) -> u32 {
    let mut file_directory: String = "/sys/devices/system/cpu/cpu0/cache".to_string();
    match cache_level {
        CacheLevel::L1D => file_directory += "/index0/size",
        CacheLevel::L1I => file_directory += "/index1/size",
        CacheLevel::L2 => file_directory += "/index2/size",
        CacheLevel::L3 => file_directory += "/index3/size",
    }

    let file_path = Path::new(&file_directory);
    if !file_path.exists() {
        warn!("File: {} does not exist.", file_directory);
        0
    } else {
        info!("File: {} exist.", file_directory);

        let src = fs::read_to_string(file_directory).expect("File not exists or file corrupted.");
        // The content of the file is as simple as a size, like: "32K"
        let src = src.trim();
        let src_digits: u32 = src[0..src.len() - 1].parse().unwrap();
        let src_unit = &src[src.len() - 1..];

        src_digits
            * match src_unit {
                "K" => 1024,
                "M" => 1024u32.pow(2),
                "G" => 1024u32.pow(3),
                _ => 1,
            }
    }
}

/// NOTE: coherency_line_size file directory example,
/// "/sys/devices/system/cpu/cpu0/cache/index0/coherency_line_size".
pub fn get_cache_coherency_line_size(cache_level: CacheLevel) -> u32 {
    let mut file_directory: String = "/sys/devices/system/cpu/cpu0/cache".to_string();
    match cache_level {
        CacheLevel::L1D => file_directory += "/index0/coherency_line_size",
        CacheLevel::L1I => file_directory += "/index1/coherency_line_size",
        CacheLevel::L2 => file_directory += "/index2/coherency_line_size",
        CacheLevel::L3 => file_directory += "/index3/coherency_line_size",
    }

    let file_path = Path::new(&file_directory);
    if !file_path.exists() {
        warn!("File: {} does not exist.", file_directory);
        0
    } else {
        info!("File: {} exist.", file_directory);

        let src = fs::read_to_string(file_directory).expect("File not exists or file corrupted.");
        src.trim().parse::<u32>().unwrap()
    }
}

/// NOTE: number_of_sets file directory example,
/// "/sys/devices/system/cpu/cpu0/cache/index0/number_of_sets".
pub fn get_cache_number_of_sets(cache_level: CacheLevel) -> u32 {
    let mut file_directory: String = "/sys/devices/system/cpu/cpu0/cache".to_string();
    match cache_level {
        CacheLevel::L1D => file_directory += "/index0/number_of_sets",
        CacheLevel::L1I => file_directory += "/index1/number_of_sets",
        CacheLevel::L2 => file_directory += "/index2/number_of_sets",
        CacheLevel::L3 => file_directory += "/index3/number_of_sets",
    }

    let file_path = Path::new(&file_directory);
    if !file_path.exists() {
        warn!("File: {} does not exist.", file_directory);
        0
    } else {
        info!("File: {} exist.", file_directory);

        let src = fs::read_to_string(file_directory).expect("File not exists or file corrupted.");
        src.trim().parse::<u32>().unwrap()
    }
}

/// NOTE: shared_cpu_list file directory example,
/// "/sys/devices/system/cpu/cpu0/cache/index0/shared_cpu_list".
pub fn get_cache_shared(cache_level: CacheLevel) -> bool {
    let mut file_directory: String = "/sys/devices/system/cpu/cpu0/cache".to_string();
    let mut result = true;

    match cache_level {
        CacheLevel::L1D | CacheLevel::L1I => result = false,
        CacheLevel::L2 => file_directory += "/index2/shared_cpu_list",
        CacheLevel::L3 => file_directory += "/index3/shared_cpu_list",
    }

    if !result {
        return false;
    }

    let file_path = Path::new(&file_directory);
    if !file_path.exists() {
        warn!("File: {} does not exist.", file_directory);
        result = false;
    } else {
        info!("File: {} exist.", file_directory);

        let src = fs::read_to_string(file_directory).expect("File not exists or file corrupted.");
        let src = src.trim();
        if src.is_empty() {
            result = false;
        } else {
            result = src.contains('-') || src.contains(',');
        }
    }

    result
}

/// Creates the flattened device tree for this aarch64 VM.
#[allow(clippy::too_many_arguments)]
pub fn create_fdt<T: DeviceInfoForFdt + Clone + Debug, S: ::std::hash::BuildHasher>(
    guest_mem: &GuestMemoryMmap,
    cmdline: &str,
    vcpu_mpidr: Vec<u64>,
    vcpu_topology: Option<(u8, u8, u8)>,
    device_info: &HashMap<(DeviceType, String), T, S>,
    gic_device: &Arc<Mutex<dyn Vgic>>,
    initrd: &Option<InitramfsConfig>,
    pmu_supported: bool,
) -> FdtWriterResult<Vec<u8>> {
    // Allocate stuff necessary for the holding the blob.
    let mut fdt = FdtWriter::new().unwrap();

    // For an explanation why these nodes were introduced in the blob take a look at
    // https://github.com/torvalds/linux/blob/master/Documentation/devicetree/booting-without-of.txt#L845
    // Look for "Required nodes and properties".

    // Header or the root node as per above mentioned documentation.
    let root_node = fdt.begin_node("")?;
    fdt.property_string("compatible", "linux,dummy-virt")?;
    // For info on #address-cells and size-cells read "Note about cells and address representation"
    // from the above mentioned txt file.
    fdt.property_u32("#address-cells", ADDRESS_CELLS)?;
    fdt.property_u32("#size-cells", SIZE_CELLS)?;
    // This is not mandatory but we use it to point the root node to the node
    // containing description of the interrupt controller for this VM.
    fdt.property_u32("interrupt-parent", GIC_PHANDLE)?;
    create_cpu_nodes(&mut fdt, &vcpu_mpidr, vcpu_topology)?;
    create_memory_node(&mut fdt, guest_mem)?;
    create_chosen_node(&mut fdt, cmdline, initrd)?;
    create_gic_node(&mut fdt, gic_device)?;
    create_timer_node(&mut fdt)?;
    if pmu_supported {
        create_pmu_node(&mut fdt)?;
    }
    create_clock_node(&mut fdt)?;
    create_psci_node(&mut fdt)?;
    create_devices_node(&mut fdt, device_info)?;

    // End Header node.
    fdt.end_node(root_node)?;

    let fdt_final = fdt.finish()?;

    Ok(fdt_final)
}

pub fn write_fdt_to_memory(fdt_final: Vec<u8>, guest_mem: &GuestMemoryMmap) -> Result<()> {
    // Write FDT to memory.
    guest_mem
        .write_slice(fdt_final.as_slice(), super::layout::FDT_START)
        .map_err(Error::WriteFdtToMemory)?;
    Ok(())
}

// Following are the auxiliary function for creating the different nodes that we append to our FDT.
fn create_cpu_nodes(
    fdt: &mut FdtWriter,
    vcpu_mpidr: &[u64],
    vcpu_topology: Option<(u8, u8, u8)>,
) -> FdtWriterResult<()> {
    // See https://github.com/torvalds/linux/blob/master/Documentation/devicetree/bindings/arm/cpus.yaml.
    let cpus_node = fdt.begin_node("cpus")?;
    fdt.property_u32("#address-cells", 0x1)?;
    fdt.property_u32("#size-cells", 0x0)?;

    let num_cpus = vcpu_mpidr.len();
    let (threads_per_core, cores_per_package, packages) = vcpu_topology.unwrap_or((1, 1, 1));
    let max_cpus: u32 = (threads_per_core * cores_per_package * packages).into();

    // Add cache info.
    // L1 Data Cache Info.
    let mut l1_d_cache_size: u32 = 0;
    let mut l1_d_cache_line_size: u32 = 0;
    let mut l1_d_cache_sets: u32 = 0;

    // L1 Instruction Cache Info.
    let mut l1_i_cache_size: u32 = 0;
    let mut l1_i_cache_line_size: u32 = 0;
    let mut l1_i_cache_sets: u32 = 0;

    // L2 Cache Info.
    let mut l2_cache_size: u32 = 0;
    let mut l2_cache_line_size: u32 = 0;
    let mut l2_cache_sets: u32 = 0;

    // L3 Cache Info.
    let mut l3_cache_size: u32 = 0;
    let mut l3_cache_line_size: u32 = 0;
    let mut l3_cache_sets: u32 = 0;

    // Cache Shared Info.
    let mut l2_cache_shared: bool = false;
    let mut l3_cache_shared: bool = false;

    let cache_path = Path::new("/sys/devices/system/cpu/cpu0/cache");
    let cache_exist: bool = cache_path.exists();
    if !cache_exist {
        warn!("cache sysfs system does not exist.");
    } else {
        info!("cache sysfs system exists.");
        // L1 Data Cache Info.
        l1_d_cache_size = get_cache_size(CacheLevel::L1D);
        l1_d_cache_line_size = get_cache_coherency_line_size(CacheLevel::L1D);
        l1_d_cache_sets = get_cache_number_of_sets(CacheLevel::L1D);

        // L1 Instruction Cache Info.
        l1_i_cache_size = get_cache_size(CacheLevel::L1I);
        l1_i_cache_line_size = get_cache_coherency_line_size(CacheLevel::L1I);
        l1_i_cache_sets = get_cache_number_of_sets(CacheLevel::L1I);

        // L2 Cache Info.
        l2_cache_size = get_cache_size(CacheLevel::L2);
        l2_cache_line_size = get_cache_coherency_line_size(CacheLevel::L2);
        l2_cache_sets = get_cache_number_of_sets(CacheLevel::L2);

        // L3 Cache Info.
        l3_cache_size = get_cache_size(CacheLevel::L3);
        l3_cache_line_size = get_cache_coherency_line_size(CacheLevel::L3);
        l3_cache_sets = get_cache_number_of_sets(CacheLevel::L3);

        // Cache Shared Info.
        if l2_cache_size != 0 {
            l2_cache_shared = get_cache_shared(CacheLevel::L2);
        }
        if l3_cache_size != 0 {
            l3_cache_shared = get_cache_shared(CacheLevel::L3);
        }
    }

    for (cpu_id, mpidr) in vcpu_mpidr.iter().enumerate().take(num_cpus) {
        let cpu_name = format!("cpu@{cpu_id:x}");
        let cpu_node = fdt.begin_node(&cpu_name)?;
        fdt.property_string("device_type", "cpu")?;
        fdt.property_string("compatible", "arm,arm-v8")?;
        if num_cpus > 1 {
            // This is required on armv8 64-bit. See aforementioned documentation.
            fdt.property_string("enable-method", "psci")?;
        }
        // Set the field to first 24 bits of the MPIDR - Multiprocessor Affinity Register.
        // See http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.ddi0488c/BABHBJCI.html.
        fdt.property_u32("reg", (mpidr & 0x7FFFFF) as u32)?;
        fdt.property_u32("phandle", cpu_id as u32 + FIRST_VCPU_PHANDLE)?;

        if cache_exist && l1_d_cache_size != 0 && l1_i_cache_size != 0 {
            // Add cache info.
            fdt.property_u32("d-cache-size", l1_d_cache_size)?;
            fdt.property_u32("d-cache-line-size", l1_d_cache_line_size)?;
            fdt.property_u32("d-cache-sets", l1_d_cache_sets)?;

            fdt.property_u32("i-cache-size", l1_i_cache_size)?;
            fdt.property_u32("i-cache-line-size", l1_i_cache_line_size)?;
            fdt.property_u32("i-cache-sets", l1_i_cache_sets)?;

            if l2_cache_size != 0 && !l2_cache_shared {
                fdt.property_u32(
                    "next-level-cache",
                    cpu_id as u32 + max_cpus + FIRST_VCPU_PHANDLE + L2_CACHE_PHANDLE,
                )?;

                let l2_cache_name = "l2-cache0";
                let l2_cache_node = fdt.begin_node(l2_cache_name)?;
                // PHANDLE is used to mark device node, and PHANDLE is unique. To avoid phandle
                // conflicts with other device nodes, consider the previous CPU PHANDLE, so the
                // CPU L2 cache PHANDLE must start from the largest CPU PHANDLE plus 1.
                fdt.property_u32(
                    "phandle",
                    cpu_id as u32 + max_cpus + FIRST_VCPU_PHANDLE + L2_CACHE_PHANDLE,
                )?;

                fdt.property_string("compatible", "cache")?;
                fdt.property_u32("cache-size", l2_cache_size)?;
                fdt.property_u32("cache-line-size", l2_cache_line_size)?;
                fdt.property_u32("cache-sets", l2_cache_sets)?;
                fdt.property_u32("cache-level", 2)?;

                if l3_cache_size != 0 && l3_cache_shared {
                    let package_id: u32 = cpu_id as u32 / cores_per_package as u32;
                    fdt.property_u32(
                        "next-level-cache",
                        package_id
                            + num_cpus as u32
                            + max_cpus
                            + FIRST_VCPU_PHANDLE
                            + L2_CACHE_PHANDLE
                            + L3_CACHE_PHANDLE,
                    )?;
                }

                fdt.end_node(l2_cache_node)?;
            }
            if l2_cache_size != 0 && l2_cache_shared {
                warn!("L2 cache shared with other cpus");
            }
        }

        fdt.end_node(cpu_node)?;
    }

    if cache_exist && l3_cache_size != 0 && !l2_cache_shared && l3_cache_shared {
        let mut i: u32 = 0;
        while i < packages.into() {
            let l3_cache_name = "l3-cache0";
            let l3_cache_node = fdt.begin_node(l3_cache_name)?;
            // ARM L3 cache is generally shared within the package (socket), so the
            // L3 cache node pointed to by the CPU in the package has the same L3
            // cache PHANDLE. The L3 cache phandle must start from the largest L2
            // cache PHANDLE plus 1 to avoid duplication.
            fdt.property_u32(
                "phandle",
                i + num_cpus as u32
                    + max_cpus
                    + FIRST_VCPU_PHANDLE
                    + L2_CACHE_PHANDLE
                    + L3_CACHE_PHANDLE,
            )?;

            fdt.property_string("compatible", "cache")?;
            fdt.property_null("cache-unified")?;
            fdt.property_u32("cache-size", l3_cache_size)?;
            fdt.property_u32("cache-line-size", l3_cache_line_size)?;
            fdt.property_u32("cache-sets", l3_cache_sets)?;
            fdt.property_u32("cache-level", 3)?;
            fdt.end_node(l3_cache_node)?;

            i += 1;
        }
    }

    if let Some(topology) = vcpu_topology {
        let (threads_per_core, cores_per_package, packages) = topology;
        let cpu_map_node = fdt.begin_node("cpu-map")?;

        // Create device tree nodes with regard of above mapping.
        for package_idx in 0..packages {
            let package_name = format!("socket{package_idx:x}");
            let package_node = fdt.begin_node(&package_name)?;

            // Cluster is the container of cores, and it is mandatory in the CPU topology.
            // Add a default "cluster0" in each socket/package.
            let cluster_node = fdt.begin_node("cluster0")?;

            for core_idx in 0..cores_per_package {
                let core_name = format!("core{core_idx:x}");
                let core_node = fdt.begin_node(&core_name)?;

                for thread_idx in 0..threads_per_core {
                    let thread_name = format!("thread{thread_idx:x}");
                    let thread_node = fdt.begin_node(&thread_name)?;
                    let cpu_idx = threads_per_core * cores_per_package * package_idx
                        + threads_per_core * core_idx
                        + thread_idx;
                    fdt.property_u32("cpu", cpu_idx as u32 + FIRST_VCPU_PHANDLE)?;
                    fdt.end_node(thread_node)?;
                }

                fdt.end_node(core_node)?;
            }
            fdt.end_node(cluster_node)?;
            fdt.end_node(package_node)?;
        }
        fdt.end_node(cpu_map_node)?;
    } else {
        debug!("Boot using device tree, CPU topology is not (correctly) specified");
    }

    fdt.end_node(cpus_node)?;

    Ok(())
}

fn create_memory_node(
    fdt: &mut FdtWriter,
    guest_mem: &GuestMemoryMmap,
) -> FdtWriterResult<()> {
    {
        // Note: memory regions from "GuestMemory" are sorted and non-zero sized.
        let ram_regions = {
            let mut ram_regions = Vec::new();
            let mut current_start = guest_mem
                .iter()
                .next()
                .map(GuestMemoryRegion::start_addr)
                .expect("GuestMemory must have one memory region at least")
                .raw_value();
            let mut current_end = current_start;

            for (start, size) in guest_mem
                .iter()
                .map(|m| (m.start_addr().raw_value(), m.len()))
            {
                if current_end == start {
                    // This zone is continuous with the previous one.
                    current_end += size;
                } else {
                    ram_regions.push((current_start, current_end));

                    current_start = start;
                    current_end = start + size;
                }
            }

            ram_regions.push((current_start, current_end));

            ram_regions
        };

        if ram_regions.len() > 2 {
            panic!(
                "There should be up to two non-continuous regions, devidided by the
                    gap at the end of 32bit address space."
            );
        }

        // Create the memory node for memory region before the gap
        {
            let (first_region_start, first_region_end) = ram_regions
                .first()
                .expect("There should be at last one memory region");
            let ram_start = super::layout::RAM_START.raw_value();
            let mem_32bit_reserved_start = super::layout::MEM_32BIT_RESERVED_START.raw_value();

            if !((first_region_start <= &ram_start)
                && (first_region_end > &ram_start)
                && (first_region_end <= &mem_32bit_reserved_start))
            {
                panic!(
                    "Unexpected first memory region layout: (start: 0x{:08x}, end: 0x{:08x}).
                    ram_start: 0x{:08x}, mem_32bit_reserved_start: 0x{:08x}",
                    first_region_start, first_region_end, ram_start, mem_32bit_reserved_start
                );
            }

            let mem_size = first_region_end - ram_start;
            let mem_reg_prop = [ram_start, mem_size];
            let memory_node_name = format!("memory@{:x}", ram_start);
            let memory_node = fdt.begin_node(&memory_node_name)?;
            fdt.property_string("device_type", "memory")?;
            fdt.property_array_u64("reg", &mem_reg_prop)?;
            fdt.end_node(memory_node)?;
        }

        // Create the memory map entry for memory region after the gap if any
        if let Some((second_region_start, second_region_end)) = ram_regions.get(1) {
            let ram_64bit_start = super::layout::RAM_64BIT_START.raw_value();

            if second_region_start != &ram_64bit_start {
                panic!(
                    "Unexpected second memory region layout: start: 0x{:08x}, ram_64bit_start: 0x{:08x}",
                    second_region_start, ram_64bit_start
                );
            }

            let mem_size = second_region_end - ram_64bit_start;
            let mem_reg_prop = [ram_64bit_start, mem_size];
            let memory_node_name = format!("memory@{:x}", ram_64bit_start);
            let memory_node = fdt.begin_node(&memory_node_name)?;
            fdt.property_string("device_type", "memory")?;
            fdt.property_array_u64("reg", &mem_reg_prop)?;
            fdt.end_node(memory_node)?;
        }
    }

    Ok(())
}

fn create_chosen_node(
    fdt: &mut FdtWriter,
    cmdline: &str,
    initrd: &Option<InitramfsConfig>,
) -> FdtWriterResult<()> {
    let chosen_node = fdt.begin_node("chosen")?;
    fdt.property_string("bootargs", cmdline)?;

    if let Some(initrd_config) = initrd {
        let initrd_start = initrd_config.address.raw_value();
        let initrd_end = initrd_config.address.raw_value() + initrd_config.size as u64;
        fdt.property_u64("linux,initrd-start", initrd_start)?;
        fdt.property_u64("linux,initrd-end", initrd_end)?;
    }

    fdt.end_node(chosen_node)?;

    Ok(())
}

fn create_gic_node(fdt: &mut FdtWriter, gic_device: &Arc<Mutex<dyn Vgic>>) -> FdtWriterResult<()> {
    let gic_reg_prop = gic_device.lock().unwrap().device_properties();

    let intc_node = fdt.begin_node("intc")?;

    fdt.property_string("compatible", gic_device.lock().unwrap().fdt_compatibility())?;
    fdt.property_null("interrupt-controller")?;
    // "interrupt-cells" field specifies the number of cells needed to encode an
    // interrupt source. The type shall be a <u32> and the value shall be 3 if no PPI affinity description
    // is required.
    fdt.property_u32("#interrupt-cells", 3)?;
    fdt.property_array_u64("reg", &gic_reg_prop)?;
    fdt.property_u32("phandle", GIC_PHANDLE)?;
    fdt.property_u32("#address-cells", 2)?;
    fdt.property_u32("#size-cells", 2)?;
    fdt.property_null("ranges")?;

    let gic_intr_prop = [
        GIC_FDT_IRQ_TYPE_PPI,
        gic_device.lock().unwrap().fdt_maint_irq(),
        IRQ_TYPE_LEVEL_HI,
    ];
    fdt.property_array_u32("interrupts", &gic_intr_prop)?;

    if gic_device.lock().unwrap().msi_compatible() {
        let msic_node = fdt.begin_node("msic")?;
        fdt.property_string("compatible", gic_device.lock().unwrap().msi_compatibility())?;
        fdt.property_null("msi-controller")?;
        fdt.property_u32("phandle", MSI_PHANDLE)?;
        let msi_reg_prop = gic_device.lock().unwrap().msi_properties();
        fdt.property_array_u64("reg", &msi_reg_prop)?;
        fdt.end_node(msic_node)?;
    }

    fdt.end_node(intc_node)?;

    Ok(())
}

fn create_clock_node(fdt: &mut FdtWriter) -> FdtWriterResult<()> {
    // The Advanced Peripheral Bus (APB) is part of the Advanced Microcontroller Bus Architecture
    // (AMBA) protocol family. It defines a low-cost interface that is optimized for minimal power
    // consumption and reduced interface complexity.
    // PCLK is the clock source and this node defines exactly the clock for the APB.
    let clock_node = fdt.begin_node("apb-pclk")?;
    fdt.property_string("compatible", "fixed-clock")?;
    fdt.property_u32("#clock-cells", 0x0)?;
    fdt.property_u32("clock-frequency", 24000000)?;
    fdt.property_string("clock-output-names", "clk24mhz")?;
    fdt.property_u32("phandle", CLOCK_PHANDLE)?;
    fdt.end_node(clock_node)?;

    Ok(())
}

fn create_timer_node(fdt: &mut FdtWriter) -> FdtWriterResult<()> {
    // See
    // https://github.com/torvalds/linux/blob/master/Documentation/devicetree/bindings/interrupt-controller/arch_timer.txt
    // These are fixed interrupt numbers for the timer device.
    let irqs = [13, 14, 11, 10];
    let compatible = "arm,armv8-timer";

    let mut timer_reg_cells: Vec<u32> = Vec::new();
    for &irq in irqs.iter() {
        timer_reg_cells.push(GIC_FDT_IRQ_TYPE_PPI);
        timer_reg_cells.push(irq);
        timer_reg_cells.push(IRQ_TYPE_LEVEL_HI);
    }

    let timer_node = fdt.begin_node("timer")?;
    fdt.property_string("compatible", compatible)?;
    fdt.property_null("always-on")?;
    fdt.property_array_u32("interrupts", &timer_reg_cells)?;
    fdt.end_node(timer_node)?;

    Ok(())
}

fn create_psci_node(fdt: &mut FdtWriter) -> FdtWriterResult<()> {
    let compatible = "arm,psci-0.2";
    let psci_node = fdt.begin_node("psci")?;
    fdt.property_string("compatible", compatible)?;
    // Two methods available: hvc and smc.
    // As per documentation, PSCI calls between a guest and hypervisor may use the HVC conduit instead of SMC.
    // So, since we are using kvm, we need to use hvc.
    fdt.property_string("method", "hvc")?;
    fdt.end_node(psci_node)?;

    Ok(())
}

fn create_virtio_node<T: DeviceInfoForFdt + Clone + Debug>(
    fdt: &mut FdtWriter,
    dev_info: &T,
) -> FdtWriterResult<()> {
    let device_reg_prop = [dev_info.addr(), dev_info.length()];
    let irq = [GIC_FDT_IRQ_TYPE_SPI, dev_info.irq(), IRQ_TYPE_EDGE_RISING];

    let virtio_node = fdt.begin_node(&format!("virtio_mmio@{:x}", dev_info.addr()))?;
    fdt.property_string("compatible", "virtio,mmio")?;
    fdt.property_array_u64("reg", &device_reg_prop)?;
    fdt.property_array_u32("interrupts", &irq)?;
    fdt.property_u32("interrupt-parent", GIC_PHANDLE)?;
    fdt.end_node(virtio_node)?;

    Ok(())
}

fn create_serial_node<T: DeviceInfoForFdt + Clone + Debug>(
    fdt: &mut FdtWriter,
    dev_info: &T,
) -> FdtWriterResult<()> {
    let compatible = b"arm,pl011\0arm,primecell\0";
    let serial_reg_prop = [dev_info.addr(), dev_info.length()];
    let irq = [
        GIC_FDT_IRQ_TYPE_SPI,
        dev_info.irq() - IRQ_BASE,
        IRQ_TYPE_EDGE_RISING,
    ];

    let serial_node = fdt.begin_node(&format!("pl011@{:x}", dev_info.addr()))?;
    fdt.property("compatible", compatible)?;
    fdt.property_array_u64("reg", &serial_reg_prop)?;
    fdt.property_u32("clocks", CLOCK_PHANDLE)?;
    fdt.property_string("clock-names", "apb_pclk")?;
    fdt.property_array_u32("interrupts", &irq)?;
    fdt.end_node(serial_node)?;

    Ok(())
}

fn create_rtc_node<T: DeviceInfoForFdt + Clone + Debug>(
    fdt: &mut FdtWriter,
    dev_info: &T,
) -> FdtWriterResult<()> {
    let compatible = b"arm,pl031\0arm,primecell\0";
    let rtc_reg_prop = [dev_info.addr(), dev_info.length()];
    let irq = [
        GIC_FDT_IRQ_TYPE_SPI,
        dev_info.irq() - IRQ_BASE,
        IRQ_TYPE_LEVEL_HI,
    ];

    let rtc_node = fdt.begin_node(&format!("rtc@{:x}", dev_info.addr()))?;
    fdt.property("compatible", compatible)?;
    fdt.property_array_u64("reg", &rtc_reg_prop)?;
    fdt.property_array_u32("interrupts", &irq)?;
    fdt.property_u32("clocks", CLOCK_PHANDLE)?;
    fdt.property_string("clock-names", "apb_pclk")?;
    fdt.end_node(rtc_node)?;

    Ok(())
}

fn create_gpio_node<T: DeviceInfoForFdt + Clone + Debug>(
    fdt: &mut FdtWriter,
    dev_info: &T,
) -> FdtWriterResult<()> {
    // PL061 GPIO controller node
    let compatible = b"arm,pl061\0arm,primecell\0";
    let gpio_reg_prop = [dev_info.addr(), dev_info.length()];
    let irq = [
        GIC_FDT_IRQ_TYPE_SPI,
        dev_info.irq() - IRQ_BASE,
        IRQ_TYPE_EDGE_RISING,
    ];

    let gpio_node = fdt.begin_node(&format!("pl061@{:x}", dev_info.addr()))?;
    fdt.property("compatible", compatible)?;
    fdt.property_array_u64("reg", &gpio_reg_prop)?;
    fdt.property_array_u32("interrupts", &irq)?;
    fdt.property_null("gpio-controller")?;
    fdt.property_u32("#gpio-cells", 2)?;
    fdt.property_u32("clocks", CLOCK_PHANDLE)?;
    fdt.property_string("clock-names", "apb_pclk")?;
    fdt.property_u32("phandle", GPIO_PHANDLE)?;
    fdt.end_node(gpio_node)?;

    // gpio-keys node
    let gpio_keys_node = fdt.begin_node("gpio-keys")?;
    fdt.property_string("compatible", "gpio-keys")?;
    fdt.property_u32("#size-cells", 0)?;
    fdt.property_u32("#address-cells", 1)?;
    let gpio_keys_poweroff_node = fdt.begin_node("button@1")?;
    fdt.property_string("label", "GPIO Key Poweroff")?;
    fdt.property_u32("linux,code", KEY_POWER)?;
    let gpios = [GPIO_PHANDLE, 3, 0];
    fdt.property_array_u32("gpios", &gpios)?;
    fdt.end_node(gpio_keys_poweroff_node)?;
    fdt.end_node(gpio_keys_node)?;

    Ok(())
}

fn create_devices_node<T: DeviceInfoForFdt + Clone + Debug, S: ::std::hash::BuildHasher>(
    fdt: &mut FdtWriter,
    dev_info: &HashMap<(DeviceType, String), T, S>,
) -> FdtWriterResult<()> {
    // Create one temp Vec to store all virtio devices
    let mut ordered_virtio_device: Vec<&T> = Vec::new();

    for ((device_type, _device_id), info) in dev_info {
        match device_type {
            DeviceType::Gpio => create_gpio_node(fdt, info)?,
            DeviceType::Rtc => create_rtc_node(fdt, info)?,
            DeviceType::Serial => create_serial_node(fdt, info)?,
            DeviceType::Virtio(_) => {
                ordered_virtio_device.push(info);
            }
        }
    }

    // Sort out virtio devices by address from low to high and insert them into fdt table.
    ordered_virtio_device.sort_by_key(|&a| a.addr());
    // Current address allocation strategy in cloud-hypervisor is: the first created device
    // will be allocated to higher address. Here we reverse the vector to make sure that
    // the older created device will appear in front of the newer created device in FDT.
    ordered_virtio_device.reverse();
    for ordered_device_info in ordered_virtio_device.drain(..) {
        create_virtio_node(fdt, ordered_device_info)?;
    }

    Ok(())
}

fn create_pmu_node(fdt: &mut FdtWriter) -> FdtWriterResult<()> {
    let compatible = "arm,armv8-pmuv3";
    let irq = [GIC_FDT_IRQ_TYPE_PPI, AARCH64_PMU_IRQ, IRQ_TYPE_LEVEL_HI];

    let pmu_node = fdt.begin_node("pmu")?;
    fdt.property_string("compatible", compatible)?;
    fdt.property_array_u32("interrupts", &irq)?;
    fdt.end_node(pmu_node)?;
    Ok(())
}

// Parse the DTB binary and print for debugging
pub fn print_fdt(dtb: &[u8]) {
    match fdt_parser::Fdt::new(dtb) {
        Ok(fdt) => {
            if let Some(root) = fdt.find_node("/") {
                debug!("Printing the FDT:");
                print_node(root, 0);
            } else {
                debug!("Failed to find root node in FDT for debugging.");
            }
        }
        Err(_) => debug!("Failed to parse FDT for debugging."),
    }
}

fn print_node(node: fdt_parser::node::FdtNode<'_, '_>, n_spaces: usize) {
    debug!("{:indent$}{}/", "", node.name, indent = n_spaces);
    for property in node.properties() {
        let name = property.name;

        // If the property is 'compatible', its value requires special handling.
        // The u8 array could contain multiple null-terminated strings.
        // We copy the original array and simply replace all 'null' characters with spaces.
        let value = if name == "compatible" {
            let mut compatible = vec![0u8; 256];
            let handled_value = property
                .value
                .iter()
                .map(|&c| if c == 0 { b' ' } else { c })
                .collect::<Vec<_>>();
            let len = cmp::min(255, handled_value.len());
            compatible[..len].copy_from_slice(&handled_value[..len]);
            compatible[..(len + 1)].to_vec()
        } else {
            property.value.to_vec()
        };
        let value = &value;

        // Now the value can be either:
        //   - A null-terminated C string, or
        //   - Binary data
        // We follow a very simple logic to present the value:
        //   - At first, try to convert it to CStr and print,
        //   - If failed, print it as u32 array.
        let value_result = match CStr::from_bytes_with_nul(value) {
            Ok(value_cstr) => match value_cstr.to_str() {
                Ok(value_str) => Some(value_str),
                Err(_e) => None,
            },
            Err(_e) => None,
        };

        if let Some(value_str) = value_result {
            debug!(
                "{:indent$}{} : {:#?}",
                "",
                name,
                value_str,
                indent = (n_spaces + 2)
            );
        } else {
            let mut array = Vec::with_capacity(256);
            array.resize(value.len() / 4, 0u32);
            BigEndian::read_u32_into(value, &mut array);
            debug!(
                "{:indent$}{} : {:X?}",
                "",
                name,
                array,
                indent = (n_spaces + 2)
            );
        };
    }

    // Print children nodes if there is any
    for child in node.children() {
        print_node(child, n_spaces + 2);
    }
}
