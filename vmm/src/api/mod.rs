// Copyright © 2019 Intel Corporation
// Copyright 2024 Alyssa Ross <hi@alyssa.is>
//
// SPDX-License-Identifier: Apache-2.0
//

//! The internal VMM API for Cloud Hypervisor.
//!
//! This API is a synchronous, [mpsc](https://doc.rust-lang.org/std/sync/mpsc/)
//! based IPC for sending commands to the VMM thread, from other
//! Cloud Hypervisor threads. The IPC follows a command-response protocol, i.e.
//! each command will receive a response back.
//!
//! The main Cloud Hypervisor thread creates an API event file descriptor
//! to notify the VMM thread about pending API commands, together with an
//! API mpsc channel. The former is the IPC control plane, the latter is the
//! IPC data plane.
//! In order to use the IPC, a Cloud Hypervisor thread needs to have a clone
//! of both the API event file descriptor and the channel Sender. Then it must
//! go through the following steps:
//!
//! 1. The thread creates an mpsc channel for receiving the command response.
//! 2. The thread sends an ApiRequest to the Sender endpoint. The ApiRequest
//!    encapsulates the response channel Sender, for the VMM API server to be
//!    able to send the response back.
//! 3. The thread writes to the API event file descriptor to notify the VMM
//!    API server about a pending command.
//! 4. The thread reads the response back from the VMM API server, from the
//!    response channel Receiver.
//! 5. The thread handles the response and forwards potential errors.

#[cfg(feature = "dbus_api")]
pub mod dbus;

#[cfg(feature = "dbus_api")]
pub use self::dbus::start_dbus_thread;

use crate::config::{
    VmConfig,
};
use crate::device_tree::DeviceTree;
use crate::vm::{Error as VmError, VmState};
use crate::Error as VmmError;
use core::fmt;
use micro_http::Body;
use serde::{Deserialize, Serialize};
use std::fmt::Display;
use std::io;
use std::sync::mpsc::{channel, RecvError, SendError, Sender};
use std::sync::{Arc, Mutex};
use vmm_sys_util::eventfd::EventFd;

/// API errors are sent back from the VMM API server through the ApiResponse.
#[derive(Debug)]
pub enum ApiError {
    /// Cannot write to EventFd.
    EventFdWrite(io::Error),

    /// API request send error
    RequestSend(SendError<ApiRequest>),

    /// Wrong response payload type
    ResponsePayloadType,

    /// API response receive error
    ResponseRecv(RecvError),

    /// The VM could not boot.
    VmBoot(VmError),

    /// The VM could not be created.
    VmCreate(VmError),

    /// The VM could not be deleted.
    VmDelete(VmError),

    /// The VM info is not available.
    VmInfo(VmError),

    /// The VM could not be paused.
    VmPause(VmError),

    /// The VM could not resume.
    VmResume(VmError),

    /// The VM is not booted.
    VmNotBooted,

    /// The VM is not created.
    VmNotCreated,

    /// The VM could not shutdown.
    VmShutdown(VmError),

    /// The VM could not reboot.
    VmReboot(VmError),

    /// The VM could not be snapshotted.
    VmSnapshot(VmError),

    /// The VM could not restored.
    VmRestore(VmError),

    /// The VM could not be coredumped.
    VmCoredump(VmError),

    /// The VMM could not shutdown.
    VmmShutdown(VmError),

    /// The device could not be added to the VM.
    VmAddDevice(VmError),

    /// The user device could not be added to the VM.
    VmAddUserDevice(VmError),

    /// The device could not be removed from the VM.
    VmRemoveDevice(VmError),

    /// The disk could not be added to the VM.
    VmAddDisk(VmError),

    /// The fs could not be added to the VM.
    VmAddFs(VmError),

    /// The pmem device could not be added to the VM.
    VmAddPmem(VmError),

    /// The network device could not be added to the VM.
    VmAddNet(VmError),

    /// The vDPA device could not be added to the VM.
    VmAddVdpa(VmError),

    /// The vsock device could not be added to the VM.
    VmAddVsock(VmError),

    /// Error triggering power button
    VmPowerButton(VmError),
}
pub type ApiResult<T> = Result<T, ApiError>;

impl Display for ApiError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        use self::ApiError::*;
        match self {
            EventFdWrite(serde_error) => write!(f, "{}", serde_error),
            RequestSend(send_error) => write!(f, "{}", send_error),
            ResponsePayloadType => write!(f, "Wrong response payload type"),
            ResponseRecv(recv_error) => write!(f, "{}", recv_error),
            VmBoot(vm_error) => write!(f, "{}", vm_error),
            VmCreate(vm_error) => write!(f, "{}", vm_error),
            VmDelete(vm_error) => write!(f, "{}", vm_error),
            VmInfo(vm_error) => write!(f, "{}", vm_error),
            VmPause(vm_error) => write!(f, "{}", vm_error),
            VmResume(vm_error) => write!(f, "{}", vm_error),
            VmNotBooted => write!(f, "VM is not booted"),
            VmNotCreated => write!(f, "VM is not created"),
            VmShutdown(vm_error) => write!(f, "{}", vm_error),
            VmReboot(vm_error) => write!(f, "{}", vm_error),
            VmSnapshot(vm_error) => write!(f, "{}", vm_error),
            VmRestore(vm_error) => write!(f, "{}", vm_error),
            VmCoredump(vm_error) => write!(f, "{}", vm_error),
            VmmShutdown(vm_error) => write!(f, "{}", vm_error),
            VmAddDevice(vm_error) => write!(f, "{}", vm_error),
            VmAddUserDevice(vm_error) => write!(f, "{}", vm_error),
            VmRemoveDevice(vm_error) => write!(f, "{}", vm_error),
            VmAddDisk(vm_error) => write!(f, "{}", vm_error),
            VmAddFs(vm_error) => write!(f, "{}", vm_error),
            VmAddPmem(vm_error) => write!(f, "{}", vm_error),
            VmAddNet(vm_error) => write!(f, "{}", vm_error),
            VmAddVdpa(vm_error) => write!(f, "{}", vm_error),
            VmAddVsock(vm_error) => write!(f, "{}", vm_error),
            VmPowerButton(vm_error) => write!(f, "{}", vm_error),
        }
    }
}

#[derive(Clone, Deserialize, Serialize)]
pub struct VmInfoResponse {
    pub config: Arc<Mutex<VmConfig>>,
    pub state: VmState,
    pub memory_actual_size: u64,
    pub device_tree: Option<Arc<Mutex<DeviceTree>>>,
}

#[derive(Clone, Deserialize, Serialize)]
pub struct VmmPingResponse {
    pub build_version: String,
    pub version: String,
    pub pid: i64,
    pub features: Vec<String>,
}

#[derive(Clone, Deserialize, Serialize, Default, Debug)]
pub struct VmResizeData {
    pub desired_vcpus: Option<u8>,
    pub desired_ram: Option<u64>,
    pub desired_balloon: Option<u64>,
}

#[derive(Clone, Deserialize, Serialize, Default, Debug)]
pub struct VmResizeZoneData {
    pub id: String,
    pub desired_ram: u64,
}

#[derive(Clone, Deserialize, Serialize, Default, Debug)]
pub struct VmRemoveDeviceData {
    pub id: String,
}

#[derive(Clone, Deserialize, Serialize, Default, Debug)]
pub struct VmSnapshotConfig {
    /// The snapshot destination URL
    pub destination_url: String,
}

#[derive(Clone, Deserialize, Serialize, Default, Debug)]
pub struct VmCoredumpData {
    /// The coredump destination file
    pub destination_url: String,
}

#[derive(Clone, Deserialize, Serialize, Default, Debug)]
pub struct VmReceiveMigrationData {
    /// URL for the reception of migration state
    pub receiver_url: String,
}

#[derive(Clone, Deserialize, Serialize, Default, Debug)]
pub struct VmSendMigrationData {
    /// URL to migrate the VM to
    pub destination_url: String,
    /// Send memory across socket without copying
    #[serde(default)]
    pub local: bool,
}

pub enum ApiResponsePayload {
    /// No data is sent on the channel.
    Empty,

    /// Virtual machine information
    VmInfo(VmInfoResponse),

    /// Vmm ping response
    VmmPing(VmmPingResponse),

    /// Vm action response
    VmAction(Option<Vec<u8>>),
}

/// This is the response sent by the VMM API server through the mpsc channel.
pub type ApiResponse = Result<ApiResponsePayload, ApiError>;

pub trait RequestHandler {
    fn vm_create(&mut self, config: Arc<Mutex<VmConfig>>) -> Result<(), VmError>;

    fn vm_boot(&mut self) -> Result<(), VmError>;

    #[cfg(all(target_arch = "x86_64", feature = "guest_debug"))]
    fn vm_coredump(&mut self, destination_url: &str) -> Result<(), VmError>;

    fn vm_shutdown(&mut self) -> Result<(), VmError>;

    fn vm_reboot(&mut self) -> Result<(), VmError>;

    fn vm_info(&self) -> Result<VmInfoResponse, VmError>;

    fn vmm_ping(&self) -> VmmPingResponse;

    fn vm_delete(&mut self) -> Result<(), VmError>;

    fn vmm_shutdown(&mut self) -> Result<(), VmError>;
}

/// It would be nice if we could pass around an object like this:
///
/// ```
/// # use vmm::api::ApiAction;
/// struct ApiRequest<Action: ApiAction + 'static> {
///     action: &'static Action,
///     body: Action::RequestBody,
/// }
/// ```
///
/// Unfortunately, it's not possible to use such a type in a trait object,
/// so as a workaround, we instead encapsulate that data in a closure, and have
/// the event loop call that closure to process a request.
pub type ApiRequest =
    Box<dyn FnOnce(&mut dyn RequestHandler) -> Result<bool, VmmError> + Send + 'static>;

fn get_response<Action: ApiAction>(
    action: &Action,
    api_evt: EventFd,
    api_sender: Sender<ApiRequest>,
    data: Action::RequestBody,
) -> ApiResult<ApiResponsePayload> {
    let (response_sender, response_receiver) = channel();

    let request = action.request(data, response_sender);
    // Send the VM request.
    api_sender.send(request).map_err(ApiError::RequestSend)?;
    api_evt.write(1).map_err(ApiError::EventFdWrite)?;

    response_receiver.recv().map_err(ApiError::ResponseRecv)?
}

fn get_response_body<Action: ApiAction<ResponseBody = Option<Body>>>(
    action: &Action,
    api_evt: EventFd,
    api_sender: Sender<ApiRequest>,
    data: Action::RequestBody,
) -> ApiResult<Option<Body>> {
    let body = match get_response(action, api_evt, api_sender, data)? {
        ApiResponsePayload::VmAction(response) => response.map(Body::new),
        ApiResponsePayload::Empty => None,
        _ => return Err(ApiError::ResponsePayloadType),
    };

    Ok(body)
}

pub trait ApiAction: Send + Sync {
    type RequestBody: Send + Sync + Sized;
    type ResponseBody: Send + Sized;

    fn request(&self, body: Self::RequestBody, response_sender: Sender<ApiResponse>) -> ApiRequest;

    fn send(
        &self,
        api_evt: EventFd,
        api_sender: Sender<ApiRequest>,
        data: Self::RequestBody,
    ) -> ApiResult<Self::ResponseBody>;
}

pub struct VmBoot;

impl ApiAction for VmBoot {
    type RequestBody = ();
    type ResponseBody = Option<Body>;

    fn request(&self, _: Self::RequestBody, response_sender: Sender<ApiResponse>) -> ApiRequest {
        Box::new(move |vmm| {
            info!("API request event: VmBoot");

            let response = vmm
                .vm_boot()
                .map_err(ApiError::VmBoot)
                .map(|_| ApiResponsePayload::Empty);

            response_sender
                .send(response)
                .map_err(VmmError::ApiResponseSend)?;

            Ok(false)
        })
    }

    fn send(
        &self,
        api_evt: EventFd,
        api_sender: Sender<ApiRequest>,
        data: Self::RequestBody,
    ) -> ApiResult<Self::ResponseBody> {
        get_response_body(self, api_evt, api_sender, data)
    }
}

#[cfg(all(target_arch = "x86_64", feature = "guest_debug"))]
pub struct VmCoredump;

#[cfg(all(target_arch = "x86_64", feature = "guest_debug"))]
impl ApiAction for VmCoredump {
    type RequestBody = VmCoredumpData;
    type ResponseBody = Option<Body>;

    fn request(
        &self,
        coredump_data: Self::RequestBody,
        response_sender: Sender<ApiResponse>,
    ) -> ApiRequest {
        Box::new(move |vmm| {
            info!("API request event: VmCoredump {:?}", coredump_data);

            let response = vmm
                .vm_coredump(&coredump_data.destination_url)
                .map_err(ApiError::VmCoredump)
                .map(|_| ApiResponsePayload::Empty);

            response_sender
                .send(response)
                .map_err(VmmError::ApiResponseSend)?;

            Ok(false)
        })
    }

    fn send(
        &self,
        api_evt: EventFd,
        api_sender: Sender<ApiRequest>,
        data: Self::RequestBody,
    ) -> ApiResult<Self::ResponseBody> {
        get_response_body(self, api_evt, api_sender, data)
    }
}

pub struct VmCreate;

impl ApiAction for VmCreate {
    type RequestBody = Arc<Mutex<VmConfig>>;
    type ResponseBody = ();

    fn request(
        &self,
        config: Self::RequestBody,
        response_sender: Sender<ApiResponse>,
    ) -> ApiRequest {
        Box::new(move |vmm| {
            info!("API request event: VmCreate {:?}", config);

            let response = vmm
                .vm_create(config)
                .map_err(ApiError::VmCreate)
                .map(|_| ApiResponsePayload::Empty);

            response_sender
                .send(response)
                .map_err(VmmError::ApiResponseSend)?;

            Ok(false)
        })
    }

    fn send(
        &self,
        api_evt: EventFd,
        api_sender: Sender<ApiRequest>,
        data: Self::RequestBody,
    ) -> ApiResult<()> {
        get_response(self, api_evt, api_sender, data)?;

        Ok(())
    }
}

pub struct VmDelete;

impl ApiAction for VmDelete {
    type RequestBody = ();
    type ResponseBody = Option<Body>;

    fn request(&self, _: Self::RequestBody, response_sender: Sender<ApiResponse>) -> ApiRequest {
        Box::new(move |vmm| {
            info!("API request event: VmDelete");

            let response = vmm
                .vm_delete()
                .map_err(ApiError::VmDelete)
                .map(|_| ApiResponsePayload::Empty);

            response_sender
                .send(response)
                .map_err(VmmError::ApiResponseSend)?;

            Ok(false)
        })
    }

    fn send(
        &self,
        api_evt: EventFd,
        api_sender: Sender<ApiRequest>,
        data: Self::RequestBody,
    ) -> ApiResult<Self::ResponseBody> {
        get_response_body(self, api_evt, api_sender, data)
    }
}

pub struct VmInfo;

impl ApiAction for VmInfo {
    type RequestBody = ();
    type ResponseBody = VmInfoResponse;

    fn request(&self, _: Self::RequestBody, response_sender: Sender<ApiResponse>) -> ApiRequest {
        Box::new(move |vmm| {
            info!("API request event: VmInfo");

            let response = vmm
                .vm_info()
                .map_err(ApiError::VmInfo)
                .map(ApiResponsePayload::VmInfo);

            response_sender
                .send(response)
                .map_err(VmmError::ApiResponseSend)?;

            Ok(false)
        })
    }

    fn send(
        &self,
        api_evt: EventFd,
        api_sender: Sender<ApiRequest>,
        data: (),
    ) -> ApiResult<VmInfoResponse> {
        let vm_info = get_response(self, api_evt, api_sender, data)?;

        match vm_info {
            ApiResponsePayload::VmInfo(info) => Ok(info),
            _ => Err(ApiError::ResponsePayloadType),
        }
    }
}

pub struct VmReboot;

impl ApiAction for VmReboot {
    type RequestBody = ();
    type ResponseBody = Option<Body>;

    fn request(&self, _: Self::RequestBody, response_sender: Sender<ApiResponse>) -> ApiRequest {
        Box::new(move |vmm| {
            info!("API request event: VmReboot");

            let response = vmm
                .vm_reboot()
                .map_err(ApiError::VmReboot)
                .map(|_| ApiResponsePayload::Empty);

            response_sender
                .send(response)
                .map_err(VmmError::ApiResponseSend)?;

            Ok(false)
        })
    }

    fn send(
        &self,
        api_evt: EventFd,
        api_sender: Sender<ApiRequest>,
        data: Self::RequestBody,
    ) -> ApiResult<Self::ResponseBody> {
        get_response_body(self, api_evt, api_sender, data)
    }
}

pub struct VmShutdown;

impl ApiAction for VmShutdown {
    type RequestBody = ();
    type ResponseBody = Option<Body>;

    fn request(
        &self,
        config: Self::RequestBody,
        response_sender: Sender<ApiResponse>,
    ) -> ApiRequest {
        Box::new(move |vmm| {
            info!("API request event: VmShutdown {:?}", config);

            let response = vmm
                .vm_shutdown()
                .map_err(ApiError::VmShutdown)
                .map(|_| ApiResponsePayload::Empty);

            response_sender
                .send(response)
                .map_err(VmmError::ApiResponseSend)?;

            Ok(false)
        })
    }

    fn send(
        &self,
        api_evt: EventFd,
        api_sender: Sender<ApiRequest>,
        data: Self::RequestBody,
    ) -> ApiResult<Self::ResponseBody> {
        get_response_body(self, api_evt, api_sender, data)
    }
}

pub struct VmmPing;

impl ApiAction for VmmPing {
    type RequestBody = ();
    type ResponseBody = VmmPingResponse;

    fn request(&self, _: Self::RequestBody, response_sender: Sender<ApiResponse>) -> ApiRequest {
        Box::new(move |vmm| {
            info!("API request event: VmmPing");

            let response = ApiResponsePayload::VmmPing(vmm.vmm_ping());

            response_sender
                .send(Ok(response))
                .map_err(VmmError::ApiResponseSend)?;

            Ok(false)
        })
    }

    fn send(
        &self,
        api_evt: EventFd,
        api_sender: Sender<ApiRequest>,
        data: (),
    ) -> ApiResult<VmmPingResponse> {
        let vmm_pong = get_response(self, api_evt, api_sender, data)?;

        match vmm_pong {
            ApiResponsePayload::VmmPing(pong) => Ok(pong),
            _ => Err(ApiError::ResponsePayloadType),
        }
    }
}

pub struct VmmShutdown;

impl ApiAction for VmmShutdown {
    type RequestBody = ();
    type ResponseBody = ();

    fn request(&self, _: Self::RequestBody, response_sender: Sender<ApiResponse>) -> ApiRequest {
        Box::new(move |vmm| {
            info!("API request event: VmmShutdown");

            let response = vmm
                .vmm_shutdown()
                .map_err(ApiError::VmmShutdown)
                .map(|_| ApiResponsePayload::Empty);

            response_sender
                .send(response)
                .map_err(VmmError::ApiResponseSend)?;

            Ok(true)
        })
    }

    fn send(&self, api_evt: EventFd, api_sender: Sender<ApiRequest>, data: ()) -> ApiResult<()> {
        get_response(self, api_evt, api_sender, data)?;

        Ok(())
    }
}
