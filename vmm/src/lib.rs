// Copyright © 2019 Intel Corporation
//
// SPDX-License-Identifier: Apache-2.0
//

#[macro_use]
extern crate log;

use crate::api::{
    ApiRequest, ApiResponse, RequestHandler, VmInfoResponse, VmmPingResponse,
};
use crate::config::{
    VmConfig,
};
#[cfg(all(target_arch = "x86_64", feature = "guest_debug"))]
use crate::coredump::GuestDebuggable;
use crate::vm::{Error as VmError, Vm, VmState};
#[cfg(feature = "dbus_api")]
use api::dbus::{DBusApiOptions, DBusApiShutdownChannels};
use libc::{tcsetattr, termios, EFD_NONBLOCK, SIGINT, SIGTERM, TCSANOW};
use serde::{Deserialize, Serialize};
use signal_hook::iterator::{Handle, Signals};
use std::fs::File;
use std::io;
use std::io::{stdout};
use std::os::unix::io::{AsRawFd, FromRawFd, RawFd};
use std::panic::AssertUnwindSafe;
use std::rc::Rc;
use std::sync::mpsc::{Receiver, RecvError, SendError, Sender};
use std::sync::{Arc, Mutex};
use std::{result, thread};
use thiserror::Error;
use vm_memory::bitmap::AtomicBitmap;
use vmm_sys_util::eventfd::EventFd;
use vmm_sys_util::signal::unblock_signal;

pub mod api;
pub mod config;
#[cfg(all(target_arch = "x86_64", feature = "guest_debug"))]
mod coredump;
pub mod cpu;
pub mod device_manager;
pub mod device_tree;
#[cfg(feature = "guest_debug")]
mod gdb;
#[cfg(feature = "igvm")]
mod igvm;
pub mod interrupt;
pub mod memory_manager;
pub mod migration;
mod serial_manager;
pub mod vm;
pub mod vm_config;

type GuestMemoryMmap = vm_memory::GuestMemoryMmap<AtomicBitmap>;
type GuestRegionMmap = vm_memory::GuestRegionMmap<AtomicBitmap>;

/// Errors associated with VMM management
#[derive(Debug, Error)]
pub enum Error {
    /// API request receive error
    #[error("Error receiving API request: {0}")]
    ApiRequestRecv(#[source] RecvError),

    /// API response send error
    #[error("Error sending API request: {0}")]
    ApiResponseSend(#[source] SendError<ApiResponse>),

    /// Cannot bind to the UNIX domain socket path
    #[error("Error binding to UNIX domain socket: {0}")]
    Bind(#[source] io::Error),

    /// Cannot clone EventFd.
    #[error("Error cloning EventFd: {0}")]
    EventFdClone(#[source] io::Error),

    /// Cannot create EventFd.
    #[error("Error creating EventFd: {0}")]
    EventFdCreate(#[source] io::Error),

    /// Cannot read from EventFd.
    #[error("Error reading from EventFd: {0}")]
    EventFdRead(#[source] io::Error),

    /// Cannot create epoll context.
    #[error("Error creating epoll context: {0}")]
    Epoll(#[source] io::Error),

    /// Cannot create HTTP thread
    #[error("Error spawning HTTP thread: {0}")]
    HttpThreadSpawn(#[source] io::Error),

    /// Cannot create D-Bus thread
    #[cfg(feature = "dbus_api")]
    #[error("Error spawning D-Bus thread: {0}")]
    DBusThreadSpawn(#[source] io::Error),

    /// Cannot start D-Bus session
    #[cfg(feature = "dbus_api")]
    #[error("Error starting D-Bus session: {0}")]
    CreateDBusSession(#[source] zbus::Error),

    /// Cannot create `event-monitor` thread
    #[error("Error spawning `event-monitor` thread: {0}")]
    EventMonitorThreadSpawn(#[source] io::Error),

    /// Cannot handle the VM STDIN stream
    #[error("Error handling VM stdin: {0:?}")]
    Stdin(VmError),

    /// Cannot handle the VM pty stream
    #[error("Error handling VM pty: {0:?}")]
    Pty(VmError),

    /// Cannot reboot the VM
    #[error("Error rebooting VM: {0:?}")]
    VmReboot(VmError),

    /// Cannot create VMM thread
    #[error("Error spawning VMM thread {0:?}")]
    VmmThreadSpawn(#[source] io::Error),

    /// Cannot shut the VMM down
    #[error("Error shutting down VMM: {0:?}")]
    VmmShutdown(VmError),

    /// Error creating API server
    #[error("Error creating API server {0:?}")]
    CreateApiServer(micro_http::ServerError),

    /// Error binding API server socket
    #[error("Error creation API server's socket {0:?}")]
    CreateApiServerSocket(#[source] io::Error),

    #[cfg(feature = "guest_debug")]
    #[error("Failed to start the GDB thread: {0}")]
    GdbThreadSpawn(io::Error),

    /// GDB request receive error
    #[cfg(feature = "guest_debug")]
    #[error("Error receiving GDB request: {0}")]
    GdbRequestRecv(#[source] RecvError),

    /// GDB response send error
    #[cfg(feature = "guest_debug")]
    #[error("Error sending GDB request: {0}")]
    GdbResponseSend(#[source] SendError<gdb::GdbResponse>),

    #[error("Cannot spawn a signal handler thread: {0}")]
    SignalHandlerSpawn(#[source] io::Error),

    #[error("Failed to join on threads: {0:?}")]
    ThreadCleanup(std::boxed::Box<dyn std::any::Any + std::marker::Send>),
}
pub type Result<T> = result::Result<T, Error>;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u64)]
pub enum EpollDispatch {
    Exit = 0,
    Reset = 1,
    Api = 2,
    Debug = 4,
    Unknown,
}

impl From<u64> for EpollDispatch {
    fn from(v: u64) -> Self {
        use EpollDispatch::*;
        match v {
            0 => Exit,
            1 => Reset,
            2 => Api,
            4 => Debug,
            _ => Unknown,
        }
    }
}

pub struct EpollContext {
    epoll_file: File,
}

impl EpollContext {
    pub fn new() -> result::Result<EpollContext, io::Error> {
        let epoll_fd = epoll::create(true)?;
        // Use 'File' to enforce closing on 'epoll_fd'
        // SAFETY: the epoll_fd returned by epoll::create is valid and owned by us.
        let epoll_file = unsafe { File::from_raw_fd(epoll_fd) };

        Ok(EpollContext { epoll_file })
    }

    pub fn add_event<T>(&mut self, fd: &T, token: EpollDispatch) -> result::Result<(), io::Error>
    where
        T: AsRawFd,
    {
        let dispatch_index = token as u64;
        epoll::ctl(
            self.epoll_file.as_raw_fd(),
            epoll::ControlOptions::EPOLL_CTL_ADD,
            fd.as_raw_fd(),
            epoll::Event::new(epoll::Events::EPOLLIN, dispatch_index),
        )?;

        Ok(())
    }

    #[cfg(fuzzing)]
    pub fn add_event_custom<T>(
        &mut self,
        fd: &T,
        id: u64,
        evts: epoll::Events,
    ) -> result::Result<(), io::Error>
    where
        T: AsRawFd,
    {
        epoll::ctl(
            self.epoll_file.as_raw_fd(),
            epoll::ControlOptions::EPOLL_CTL_ADD,
            fd.as_raw_fd(),
            epoll::Event::new(evts, id),
        )?;

        Ok(())
    }
}

impl AsRawFd for EpollContext {
    fn as_raw_fd(&self) -> RawFd {
        self.epoll_file.as_raw_fd()
    }
}

pub struct PciDeviceInfo {
    pub id: String,
}

pub fn feature_list() -> Vec<String> {
    vec![
        #[cfg(feature = "dbus_api")]
        "dbus_api".to_string(),
        #[cfg(feature = "dhat-heap")]
        "dhat-heap".to_string(),
        #[cfg(feature = "guest_debug")]
        "guest_debug".to_string(),
        #[cfg(feature = "igvm")]
        "igvm".to_string(),
        #[cfg(feature = "io_uring")]
        "io_uring".to_string(),
        #[cfg(feature = "kvm")]
        "kvm".to_string(),
        #[cfg(feature = "mshv")]
        "mshv".to_string(),
        #[cfg(feature = "sev_snp")]
        "sev_snp".to_string(),
        #[cfg(feature = "tdx")]
        "tdx".to_string(),
        #[cfg(feature = "tracing")]
        "tracing".to_string(),
    ]
}

#[allow(unused_variables)]
#[allow(clippy::too_many_arguments)]
pub fn start_vmm_thread(
    vmm_version: VmmVersionInfo,
    http_path: &Option<String>,
    http_fd: Option<RawFd>,
    #[cfg(feature = "dbus_api")] dbus_options: Option<DBusApiOptions>,
    api_event: EventFd,
    api_sender: Sender<ApiRequest>,
    api_receiver: Receiver<ApiRequest>,
    #[cfg(feature = "guest_debug")] debug_path: Option<PathBuf>,
    #[cfg(feature = "guest_debug")] debug_event: EventFd,
    #[cfg(feature = "guest_debug")] vm_debug_event: EventFd,
    exit_event: EventFd,
    hypervisor: Arc<dyn hypervisor::Hypervisor>,
) -> Result<VmmThreadHandle> {
    #[cfg(feature = "guest_debug")]
    let gdb_hw_breakpoints = hypervisor.get_guest_debug_hw_bps();
    #[cfg(feature = "guest_debug")]
    let (gdb_sender, gdb_receiver) = std::sync::mpsc::channel();
    #[cfg(feature = "guest_debug")]
    let gdb_debug_event = debug_event.try_clone().map_err(Error::EventFdClone)?;
    #[cfg(feature = "guest_debug")]
    let gdb_vm_debug_event = vm_debug_event.try_clone().map_err(Error::EventFdClone)?;

    let api_event_clone = api_event.try_clone().map_err(Error::EventFdClone)?;
    let hypervisor_type = hypervisor.hypervisor_type();

    let thread = {
        let exit_event = exit_event.try_clone().map_err(Error::EventFdClone)?;
        thread::Builder::new()
            .name("vmm".to_string())
            .spawn(move || {
                let mut vmm = Vmm::new(
                    vmm_version,
                    api_event,
                    #[cfg(feature = "guest_debug")]
                    debug_event,
                    #[cfg(feature = "guest_debug")]
                    vm_debug_event,
                    hypervisor,
                    exit_event,
                )?;

                vmm.setup_signal_handler()?;

                vmm.control_loop(
                    Rc::new(api_receiver),
                    #[cfg(feature = "guest_debug")]
                    Rc::new(gdb_receiver),
                )
            })
            .map_err(Error::VmmThreadSpawn)?
    };

    // The VMM thread is started, we can start the dbus thread
    // and start serving HTTP requests
    #[cfg(feature = "dbus_api")]
    let dbus_shutdown_chs = match dbus_options {
        Some(opts) => {
            let (_, chs) = api::start_dbus_thread(
                opts,
                api_event_clone.try_clone().map_err(Error::EventFdClone)?,
                api_sender.clone(),
                exit_event.try_clone().map_err(Error::EventFdClone)?,
                hypervisor_type,
            )?;
            Some(chs)
        }
        None => None,
    };

    #[cfg(feature = "guest_debug")]
    if let Some(debug_path) = debug_path {
        let target = gdb::GdbStub::new(
            gdb_sender,
            gdb_debug_event,
            gdb_vm_debug_event,
            gdb_hw_breakpoints,
        );
        thread::Builder::new()
            .name("gdb".to_owned())
            .spawn(move || gdb::gdb_thread(target, &debug_path))
            .map_err(Error::GdbThreadSpawn)?;
    }

    Ok(VmmThreadHandle {
        thread_handle: thread,
        #[cfg(feature = "dbus_api")]
        dbus_shutdown_chs,
    })
}

#[derive(Clone, Deserialize, Serialize)]
struct VmMigrationConfig {
    vm_config: Arc<Mutex<VmConfig>>,
    #[cfg(all(feature = "kvm", target_arch = "x86_64"))]
    common_cpuid: Vec<hypervisor::arch::x86::CpuIdEntry>,
}

#[derive(Debug, Clone)]
pub struct VmmVersionInfo {
    pub build_version: String,
    pub version: String,
}

impl VmmVersionInfo {
    pub fn new(build_version: &str, version: &str) -> Self {
        Self {
            build_version: build_version.to_owned(),
            version: version.to_owned(),
        }
    }
}

pub struct VmmThreadHandle {
    pub thread_handle: thread::JoinHandle<Result<()>>,
    #[cfg(feature = "dbus_api")]
    pub dbus_shutdown_chs: Option<DBusApiShutdownChannels>,
}

pub struct Vmm {
    epoll: EpollContext,
    exit_evt: EventFd,
    reset_evt: EventFd,
    api_evt: EventFd,
    #[cfg(feature = "guest_debug")]
    debug_evt: EventFd,
    #[cfg(feature = "guest_debug")]
    vm_debug_evt: EventFd,
    version: VmmVersionInfo,
    vm: Option<Vm>,
    vm_config: Option<Arc<Mutex<VmConfig>>>,
    hypervisor: Arc<dyn hypervisor::Hypervisor>,
    signals: Option<Handle>,
    threads: Vec<thread::JoinHandle<()>>,
    original_termios_opt: Arc<Mutex<Option<termios>>>,
}

impl Vmm {
    pub const HANDLED_SIGNALS: [i32; 2] = [SIGTERM, SIGINT];

    fn signal_handler(
        mut signals: Signals,
        original_termios_opt: Arc<Mutex<Option<termios>>>,
        exit_evt: &EventFd,
    ) {
        for sig in &Self::HANDLED_SIGNALS {
            unblock_signal(*sig).unwrap();
        }

        for signal in signals.forever() {
            match signal {
                SIGTERM | SIGINT => {
                    if exit_evt.write(1).is_err() {
                        // Resetting the terminal is usually done as the VMM exits
                        if let Ok(lock) = original_termios_opt.lock() {
                            if let Some(termios) = *lock {
                                // SAFETY: FFI call
                                let _ = unsafe {
                                    tcsetattr(stdout().lock().as_raw_fd(), TCSANOW, &termios)
                                };
                            }
                        } else {
                            warn!("Failed to lock original termios");
                        }

                        std::process::exit(1);
                    }
                }
                _ => (),
            }
        }
    }

    fn setup_signal_handler(&mut self) -> Result<()> {
        let signals = Signals::new(Self::HANDLED_SIGNALS);
        match signals {
            Ok(signals) => {
                self.signals = Some(signals.handle());
                let exit_evt = self.exit_evt.try_clone().map_err(Error::EventFdClone)?;
                let original_termios_opt = Arc::clone(&self.original_termios_opt);

                self.threads.push(
                    thread::Builder::new()
                        .name("vmm_signal_handler".to_string())
                        .spawn(move || {
                            std::panic::catch_unwind(AssertUnwindSafe(|| {
                                Vmm::signal_handler(signals, original_termios_opt, &exit_evt);
                            }))
                            .map_err(|_| {
                                error!("vmm signal_handler thread panicked");
                                exit_evt.write(1).ok()
                            })
                            .ok();
                        })
                        .map_err(Error::SignalHandlerSpawn)?,
                );
            }
            Err(e) => error!("Signal not found {}", e),
        }
        Ok(())
    }

    fn new(
        vmm_version: VmmVersionInfo,
        api_evt: EventFd,
        #[cfg(feature = "guest_debug")] debug_evt: EventFd,
        #[cfg(feature = "guest_debug")] vm_debug_evt: EventFd,
        hypervisor: Arc<dyn hypervisor::Hypervisor>,
        exit_evt: EventFd,
    ) -> Result<Self> {
        let mut epoll = EpollContext::new().map_err(Error::Epoll)?;
        let reset_evt = EventFd::new(EFD_NONBLOCK).map_err(Error::EventFdCreate)?;

        epoll
            .add_event(&exit_evt, EpollDispatch::Exit)
            .map_err(Error::Epoll)?;

        epoll
            .add_event(&reset_evt, EpollDispatch::Reset)
            .map_err(Error::Epoll)?;

        epoll
            .add_event(&api_evt, EpollDispatch::Api)
            .map_err(Error::Epoll)?;

        #[cfg(feature = "guest_debug")]
        epoll
            .add_event(&debug_evt, EpollDispatch::Debug)
            .map_err(Error::Epoll)?;

        Ok(Vmm {
            epoll,
            exit_evt,
            reset_evt,
            api_evt,
            #[cfg(feature = "guest_debug")]
            debug_evt,
            #[cfg(feature = "guest_debug")]
            vm_debug_evt,
            version: vmm_version,
            vm: None,
            vm_config: None,
            hypervisor,
            signals: None,
            threads: vec![],
            original_termios_opt: Arc::new(Mutex::new(None)),
        })
    }

    fn control_loop(
        &mut self,
        api_receiver: Rc<Receiver<ApiRequest>>,
        #[cfg(feature = "guest_debug")] gdb_receiver: Rc<Receiver<gdb::GdbRequest>>,
    ) -> Result<()> {
        const EPOLL_EVENTS_LEN: usize = 100;

        let mut events = vec![epoll::Event::new(epoll::Events::empty(), 0); EPOLL_EVENTS_LEN];
        let epoll_fd = self.epoll.as_raw_fd();

        'outer: loop {
            let num_events = match epoll::wait(epoll_fd, -1, &mut events[..]) {
                Ok(res) => res,
                Err(e) => {
                    if e.kind() == io::ErrorKind::Interrupted {
                        // It's well defined from the epoll_wait() syscall
                        // documentation that the epoll loop can be interrupted
                        // before any of the requested events occurred or the
                        // timeout expired. In both those cases, epoll_wait()
                        // returns an error of type EINTR, but this should not
                        // be considered as a regular error. Instead it is more
                        // appropriate to retry, by calling into epoll_wait().
                        continue;
                    }
                    return Err(Error::Epoll(e));
                }
            };

            for event in events.iter().take(num_events) {
                let dispatch_event: EpollDispatch = event.data.into();
                match dispatch_event {
                    EpollDispatch::Unknown => {
                        let event = event.data;
                        warn!("Unknown VMM loop event: {}", event);
                    }
                    EpollDispatch::Exit => {
                        info!("VM exit event");
                        // Consume the event.
                        self.exit_evt.read().map_err(Error::EventFdRead)?;
                        self.vmm_shutdown().map_err(Error::VmmShutdown)?;

                        break 'outer;
                    }
                    EpollDispatch::Reset => {
                        info!("VM reset event");
                        // Consume the event.
                        self.reset_evt.read().map_err(Error::EventFdRead)?;
                        self.vm_reboot().map_err(Error::VmReboot)?;
                    }
                    EpollDispatch::Api => {
                        // Consume the events.
                        for _ in 0..self.api_evt.read().map_err(Error::EventFdRead)? {
                            // Read from the API receiver channel
                            let api_request = api_receiver.recv().map_err(Error::ApiRequestRecv)?;

                            if api_request(self)? {
                                break 'outer;
                            }
                        }
                    }
                    #[cfg(feature = "guest_debug")]
                    EpollDispatch::Debug => {
                        // Consume the events.
                        for _ in 0..self.debug_evt.read().map_err(Error::EventFdRead)? {
                            // Read from the API receiver channel
                            let gdb_request = gdb_receiver.recv().map_err(Error::GdbRequestRecv)?;

                            let response = if let Some(ref mut vm) = self.vm {
                                vm.debug_request(&gdb_request.payload, gdb_request.cpu_id)
                            } else {
                                Err(VmError::VmNotRunning)
                            }
                            .map_err(gdb::Error::Vm);

                            gdb_request
                                .sender
                                .send(response)
                                .map_err(Error::GdbResponseSend)?;
                        }
                    }
                    #[cfg(not(feature = "guest_debug"))]
                    EpollDispatch::Debug => {}
                }
            }
        }

        // Trigger the termination of the signal_handler thread
        if let Some(signals) = self.signals.take() {
            signals.close();
        }

        // Wait for all the threads to finish
        for thread in self.threads.drain(..) {
            thread.join().map_err(Error::ThreadCleanup)?
        }

        Ok(())
    }
}

impl RequestHandler for Vmm {
    fn vm_create(&mut self, config: Arc<Mutex<VmConfig>>) -> result::Result<(), VmError> {
        // We only store the passed VM config.
        // The VM will be created when being asked to boot it.
        if self.vm_config.is_none() {
            self.vm_config = Some(config);
            Ok(())
        } else {
            Err(VmError::VmAlreadyCreated)
        }
    }

    fn vm_boot(&mut self) -> result::Result<(), VmError> {
        let r = {
            // If we don't have a config, we can not boot a VM.
            if self.vm_config.is_none() {
                return Err(VmError::VmMissingConfig);
            };

            // Create a new VM if we don't have one yet.
            if self.vm.is_none() {
                let exit_evt = self.exit_evt.try_clone().map_err(VmError::EventFdClone)?;
                let reset_evt = self.reset_evt.try_clone().map_err(VmError::EventFdClone)?;
                #[cfg(feature = "guest_debug")]
                let vm_debug_evt = self
                    .vm_debug_evt
                    .try_clone()
                    .map_err(VmError::EventFdClone)?;

                if let Some(ref vm_config) = self.vm_config {
                    let vm = Vm::new(
                        Arc::clone(vm_config),
                        exit_evt,
                        reset_evt,
                        #[cfg(feature = "guest_debug")]
                        vm_debug_evt,
                        self.hypervisor.clone(),
                        None,
                        Arc::clone(&self.original_termios_opt),
                        None,
                    )?;

                    self.vm = Some(vm);
                }
            }

            // Now we can boot the VM.
            if let Some(ref mut vm) = self.vm {
                vm.boot()
            } else {
                Err(VmError::VmNotCreated)
            }
        };
        r
    }

    #[cfg(all(target_arch = "x86_64", feature = "guest_debug"))]
    fn vm_coredump(&mut self, destination_url: &str) -> result::Result<(), VmError> {
        if let Some(ref mut vm) = self.vm {
            vm.coredump(destination_url).map_err(VmError::Coredump)
        } else {
            Err(VmError::VmNotRunning)
        }
    }

    fn vm_shutdown(&mut self) -> result::Result<(), VmError> {
        if let Some(ref mut vm) = self.vm.take() {
            vm.shutdown()
        } else {
            Err(VmError::VmNotRunning)
        }
    }

    fn vm_reboot(&mut self) -> result::Result<(), VmError> {
        // First we stop the current VM
        let (config, serial_pty) =
            if let Some(mut vm) = self.vm.take() {
                let config = vm.get_config();
                let serial_pty = vm.serial_pty();
                vm.shutdown()?;
                (
                    config,
                    serial_pty,
                )
            } else {
                return Err(VmError::VmNotCreated);
            };

        let exit_evt = self.exit_evt.try_clone().map_err(VmError::EventFdClone)?;
        let reset_evt = self.reset_evt.try_clone().map_err(VmError::EventFdClone)?;
        #[cfg(feature = "guest_debug")]
        let debug_evt = self
            .vm_debug_evt
            .try_clone()
            .map_err(VmError::EventFdClone)?;

        // The Linux kernel fires off an i8042 reset after doing the ACPI reset so there may be
        // an event sitting in the shared reset_evt. Without doing this we get very early reboots
        // during the boot process.
        if self.reset_evt.read().is_ok() {
            warn!("Spurious second reset event received. Ignoring.");
        }

        // Then we create the new VM
        let mut vm = Vm::new(
            config,
            exit_evt,
            reset_evt,
            #[cfg(feature = "guest_debug")]
            debug_evt,
            self.hypervisor.clone(),
            serial_pty,
            Arc::clone(&self.original_termios_opt),
            None,
        )?;

        // And we boot it
        vm.boot()?;

        self.vm = Some(vm);

        Ok(())
    }

    fn vm_info(&self) -> result::Result<VmInfoResponse, VmError> {
        match &self.vm_config {
            Some(config) => {
                let state = match &self.vm {
                    Some(vm) => vm.get_state()?,
                    None => VmState::Created,
                };

                let config = Arc::clone(config);

                let memory_actual_size = config.lock().unwrap().memory.total_size();

                let device_tree = self.vm.as_ref().map(|vm| vm.device_tree());

                Ok(VmInfoResponse {
                    config,
                    state,
                    memory_actual_size,
                    device_tree,
                })
            }
            None => Err(VmError::VmNotCreated),
        }
    }

    fn vmm_ping(&self) -> VmmPingResponse {
        let VmmVersionInfo {
            build_version,
            version,
        } = self.version.clone();

        VmmPingResponse {
            build_version,
            version,
            pid: std::process::id() as i64,
            features: feature_list(),
        }
    }

    fn vm_delete(&mut self) -> result::Result<(), VmError> {
        if self.vm_config.is_none() {
            return Ok(());
        }

        // If a VM is booted, we first try to shut it down.
        if self.vm.is_some() {
            self.vm_shutdown()?;
        }

        self.vm_config = None;

        Ok(())
    }

    fn vmm_shutdown(&mut self) -> result::Result<(), VmError> {
        self.vm_delete()?;
        Ok(())
    }

    fn vm_counters(&mut self) -> result::Result<Option<Vec<u8>>, VmError> {
        if let Some(ref mut vm) = self.vm {
            let info = vm.counters().map_err(|e| {
                error!("Error when getting counters from the VM: {:?}", e);
                e
            })?;
            serde_json::to_vec(&info)
                .map(Some)
                .map_err(VmError::SerializeJson)
        } else {
            Err(VmError::VmNotRunning)
        }
    }

    fn vm_power_button(&mut self) -> result::Result<(), VmError> {
        if let Some(ref mut vm) = self.vm {
            vm.power_button()
        } else {
            Err(VmError::VmNotRunning)
        }
    }
}

const CPU_MANAGER_SNAPSHOT_ID: &str = "cpu-manager";

#[cfg(test)]
mod unit_tests {
    use super::*;
    #[cfg(target_arch = "x86_64")]
    use crate::config::DebugConsoleConfig;
    use config::{
        ConsoleConfig, ConsoleOutputMode, CpusConfig, HotplugMethod, MemoryConfig, PayloadConfig,
        RngConfig, VmConfig,
    };

    fn create_dummy_vmm() -> Vmm {
        Vmm::new(
            VmmVersionInfo::new("dummy", "dummy"),
            EventFd::new(EFD_NONBLOCK).unwrap(),
            #[cfg(feature = "guest_debug")]
            EventFd::new(EFD_NONBLOCK).unwrap(),
            #[cfg(feature = "guest_debug")]
            EventFd::new(EFD_NONBLOCK).unwrap(),
            SeccompAction::Allow,
            hypervisor::new().unwrap(),
            EventFd::new(EFD_NONBLOCK).unwrap(),
        )
        .unwrap()
    }

    fn create_dummy_vm_config() -> Arc<Mutex<VmConfig>> {
        Arc::new(Mutex::new(VmConfig {
            cpus: CpusConfig {
                boot_vcpus: 1,
                max_vcpus: 1,
                topology: None,
                kvm_hyperv: false,
                max_phys_bits: 46,
                affinity: None,
                features: config::CpuFeatures::default(),
            },
            memory: MemoryConfig {
                size: 536_870_912,
                mergeable: false,
                hotplug_method: HotplugMethod::Acpi,
                hotplug_size: None,
                hotplugged_size: None,
                shared: true,
                hugepages: false,
                hugepage_size: None,
                prefault: false,
                zones: None,
                thp: true,
            },
            payload: Some(PayloadConfig {
                kernel: Some(PathBuf::from("/path/to/kernel")),
                firmware: None,
                cmdline: None,
                initramfs: None,
                #[cfg(feature = "igvm")]
                igvm: None,
            }),
            rate_limit_groups: None,
            disks: None,
            net: None,
            rng: RngConfig {
                src: PathBuf::from("/dev/urandom"),
                iommu: false,
            },
            balloon: None,
            fs: None,
            pmem: None,
            serial: ConsoleConfig {
                file: None,
                mode: ConsoleOutputMode::Null,
                iommu: false,
                socket: None,
            },
            console: ConsoleConfig {
                file: None,
                mode: ConsoleOutputMode::Tty,
                iommu: false,
                socket: None,
            },
            #[cfg(target_arch = "x86_64")]
            debug_console: DebugConsoleConfig::default(),
            devices: None,
            user_devices: None,
            vdpa: None,
            vsock: None,
            pvpanic: false,
            iommu: false,
            #[cfg(target_arch = "x86_64")]
            sgx_epc: None,
            numa: None,
            watchdog: false,
            #[cfg(feature = "guest_debug")]
            gdb: false,
            platform: None,
            tpm: None,
            preserved_fds: None,
        }))
    }

    #[test]
    fn test_vmm_vm_create() {
        let mut vmm = create_dummy_vmm();
        let config = create_dummy_vm_config();

        assert!(matches!(vmm.vm_create(config.clone()), Ok(())));
        assert!(matches!(
            vmm.vm_create(config),
            Err(VmError::VmAlreadyCreated)
        ));
    }

    #[test]
    fn test_vmm_vm_cold_add_disk() {
        let mut vmm = create_dummy_vmm();
        let disk_config = DiskConfig::parse("path=/path/to_file").unwrap();

        assert!(matches!(
            vmm.vm_add_disk(disk_config.clone()),
            Err(VmError::VmNotCreated)
        ));

        let _ = vmm.vm_create(create_dummy_vm_config());
        assert!(vmm
            .vm_config
            .as_ref()
            .unwrap()
            .lock()
            .unwrap()
            .disks
            .is_none());

        let result = vmm.vm_add_disk(disk_config.clone());
        assert!(result.is_ok());
        assert!(result.unwrap().is_none());
        assert_eq!(
            vmm.vm_config
                .as_ref()
                .unwrap()
                .lock()
                .unwrap()
                .disks
                .clone()
                .unwrap()
                .len(),
            1
        );
        assert_eq!(
            vmm.vm_config
                .as_ref()
                .unwrap()
                .lock()
                .unwrap()
                .disks
                .clone()
                .unwrap()[0],
            disk_config
        );
    }

    #[test]
    fn test_vmm_vm_cold_add_pmem() {
        let mut vmm = create_dummy_vmm();
        let pmem_config = PmemConfig::parse("file=/tmp/pmem,size=128M").unwrap();

        assert!(matches!(
            vmm.vm_add_pmem(pmem_config.clone()),
            Err(VmError::VmNotCreated)
        ));

        let _ = vmm.vm_create(create_dummy_vm_config());
        assert!(vmm
            .vm_config
            .as_ref()
            .unwrap()
            .lock()
            .unwrap()
            .pmem
            .is_none());

        let result = vmm.vm_add_pmem(pmem_config.clone());
        assert!(result.is_ok());
        assert!(result.unwrap().is_none());
        assert_eq!(
            vmm.vm_config
                .as_ref()
                .unwrap()
                .lock()
                .unwrap()
                .pmem
                .clone()
                .unwrap()
                .len(),
            1
        );
        assert_eq!(
            vmm.vm_config
                .as_ref()
                .unwrap()
                .lock()
                .unwrap()
                .pmem
                .clone()
                .unwrap()[0],
            pmem_config
        );
    }

    #[test]
    fn test_vmm_vm_cold_add_net() {
        let mut vmm = create_dummy_vmm();
        let net_config = NetConfig::parse(
            "mac=de:ad:be:ef:12:34,host_mac=12:34:de:ad:be:ef,vhost_user=true,socket=/tmp/sock",
        )
        .unwrap();

        assert!(matches!(
            vmm.vm_add_net(net_config.clone()),
            Err(VmError::VmNotCreated)
        ));

        let _ = vmm.vm_create(create_dummy_vm_config());
        assert!(vmm
            .vm_config
            .as_ref()
            .unwrap()
            .lock()
            .unwrap()
            .net
            .is_none());

        let result = vmm.vm_add_net(net_config.clone());
        assert!(result.is_ok());
        assert!(result.unwrap().is_none());
        assert_eq!(
            vmm.vm_config
                .as_ref()
                .unwrap()
                .lock()
                .unwrap()
                .net
                .clone()
                .unwrap()
                .len(),
            1
        );
        assert_eq!(
            vmm.vm_config
                .as_ref()
                .unwrap()
                .lock()
                .unwrap()
                .net
                .clone()
                .unwrap()[0],
            net_config
        );
    }
}
