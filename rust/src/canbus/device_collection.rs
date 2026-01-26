//! CAN device collection for managing multiple devices.

use pyo3::prelude::*;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use super::device::{CANDeviceTrait, MotorDeviceCan};
use super::socket::CANSocket;
use crate::damiao_motor::CallbackMode;

/// Collection of CAN devices with frame dispatch.
#[pyclass]
pub struct CANDeviceCollection {
    devices: Arc<Mutex<HashMap<u32, Arc<Mutex<MotorDeviceCan>>>>>,
    socket: Arc<Mutex<CANSocket>>,
}

#[pymethods]
impl CANDeviceCollection {
    /// Register a device with the collection.
    pub fn register_device(&self, device: &MotorDeviceCan) {
        let recv_id = device.recv_can_id();
        let device_clone = device.clone_inner();
        self.devices
            .lock()
            .unwrap()
            .insert(recv_id, Arc::new(Mutex::new(device_clone)));
    }

    /// Unregister a device from the collection.
    pub fn unregister_device(&self, recv_can_id: u32) {
        self.devices.lock().unwrap().remove(&recv_can_id);
    }

    /// Get the number of registered devices.
    pub fn device_count(&self) -> usize {
        self.devices.lock().unwrap().len()
    }

    /// Get the socket interface name.
    #[getter]
    pub fn get_interface(&self) -> String {
        self.socket.lock().unwrap().get_interface().to_string()
    }

    /// Set callback mode for all devices.
    pub fn set_callback_mode_all(&self, mode: CallbackMode) {
        let devices = self.devices.lock().unwrap();
        for device in devices.values() {
            device.lock().unwrap().set_callback_mode(mode);
        }
    }

    /// Dispatch a received frame to the appropriate device.
    pub fn dispatch_frame(&self, can_id: u32, data: Vec<u8>) -> bool {
        let devices = self.devices.lock().unwrap();
        if let Some(device) = devices.get(&can_id) {
            device.lock().unwrap().process_callback(&data);
            true
        } else {
            false
        }
    }

    fn __repr__(&self) -> String {
        format!(
            "CANDeviceCollection(devices={})",
            self.device_count()
        )
    }
}

impl CANDeviceCollection {
    /// Create a new device collection (internal).
    pub(crate) fn new(socket: CANSocket) -> Self {
        Self {
            devices: Arc::new(Mutex::new(HashMap::new())),
            socket: Arc::new(Mutex::new(socket)),
        }
    }

    /// Get socket reference (internal).
    pub(crate) fn socket(&self) -> Arc<Mutex<CANSocket>> {
        Arc::clone(&self.socket)
    }

    /// Get devices map (internal).
    pub(crate) fn devices(&self) -> Arc<Mutex<HashMap<u32, Arc<Mutex<MotorDeviceCan>>>>> {
        Arc::clone(&self.devices)
    }

    /// Create from existing socket Arc (internal).
    pub(crate) fn from_socket_arc(socket: Arc<Mutex<CANSocket>>) -> Self {
        Self {
            devices: Arc::new(Mutex::new(HashMap::new())),
            socket,
        }
    }

    /// Register device (internal).
    pub(crate) fn register_device_internal(&self, device: Arc<Mutex<MotorDeviceCan>>) {
        let recv_id = device.lock().unwrap().recv_can_id();
        self.devices.lock().unwrap().insert(recv_id, device);
    }

    /// Receive all available frames with timeout for first frame.
    pub(crate) fn recv_all(&self, first_timeout_us: u64) -> PyResult<usize> {
        let socket = self.socket.lock().unwrap();
        let mut count = 0;

        // Wait for first frame with specified timeout
        if socket.is_data_available(first_timeout_us)? {
            if let Some((can_id, data)) = socket.read_raw().map_err(|e| {
                pyo3::exceptions::PyIOError::new_err(format!("Read error: {}", e))
            })? {
                drop(socket); // Release lock before dispatch
                self.dispatch_frame(can_id, data);
                count += 1;

                // Read remaining frames with zero timeout (non-blocking)
                loop {
                    let socket = self.socket.lock().unwrap();
                    if !socket.is_data_available(0)? {
                        break;
                    }
                    if let Some((can_id, data)) = socket.read_raw().map_err(|e| {
                        pyo3::exceptions::PyIOError::new_err(format!("Read error: {}", e))
                    })? {
                        drop(socket);
                        self.dispatch_frame(can_id, data);
                        count += 1;
                    } else {
                        break;
                    }
                }
            }
        }

        Ok(count)
    }

    /// Send a CAN packet through the socket.
    pub(crate) fn send_packet(&self, can_id: u32, data: &[u8]) -> std::io::Result<()> {
        let socket = self.socket.lock().unwrap();
        socket.write_raw(can_id, data)
    }
}
