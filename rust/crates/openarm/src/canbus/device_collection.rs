//! CAN device collection for managing multiple devices.

use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use super::device::{CANDeviceTrait, MotorDeviceCan};
use super::socket::CANSocket;
use crate::damiao_motor::CallbackMode;
use crate::error::Result;

/// Collection of CAN devices with frame dispatch.
pub struct CANDeviceCollection {
    devices: Arc<Mutex<HashMap<u32, Arc<Mutex<MotorDeviceCan>>>>>,
    socket: Arc<Mutex<CANSocket>>,
}

impl CANDeviceCollection {
    /// Create a new device collection.
    pub fn new(socket: CANSocket) -> Self {
        Self {
            devices: Arc::new(Mutex::new(HashMap::new())),
            socket: Arc::new(Mutex::new(socket)),
        }
    }

    /// Create from existing socket Arc.
    pub fn from_socket_arc(socket: Arc<Mutex<CANSocket>>) -> Self {
        Self {
            devices: Arc::new(Mutex::new(HashMap::new())),
            socket,
        }
    }

    /// Register a device with the collection.
    pub fn register_device(&self, device: &MotorDeviceCan) {
        let recv_id = device.recv_can_id();
        let device_clone = device.clone_inner();
        self.devices
            .lock()
            .unwrap()
            .insert(recv_id, Arc::new(Mutex::new(device_clone)));
    }

    /// Register device from Arc.
    pub fn register_device_internal(&self, device: Arc<Mutex<MotorDeviceCan>>) {
        let recv_id = device.lock().unwrap().recv_can_id();
        self.devices.lock().unwrap().insert(recv_id, device);
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
    pub fn interface(&self) -> String {
        self.socket.lock().unwrap().interface().to_string()
    }

    /// Get socket reference.
    pub fn socket(&self) -> Arc<Mutex<CANSocket>> {
        Arc::clone(&self.socket)
    }

    /// Get devices map.
    pub fn devices(&self) -> Arc<Mutex<HashMap<u32, Arc<Mutex<MotorDeviceCan>>>>> {
        Arc::clone(&self.devices)
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

    /// Receive all available frames with timeout for first frame.
    pub fn recv_all(&self, first_timeout_us: u64) -> Result<usize> {
        let socket = self.socket.lock().unwrap();
        let mut count = 0;

        // Wait for first frame with specified timeout
        if socket.is_data_available(first_timeout_us)? {
            if let Some((can_id, data)) = socket.read_raw()? {
                drop(socket); // Release lock before dispatch
                self.dispatch_frame(can_id, data);
                count += 1;

                // Read remaining frames with zero timeout (non-blocking)
                loop {
                    let socket = self.socket.lock().unwrap();
                    if !socket.is_data_available(0)? {
                        break;
                    }
                    if let Some((can_id, data)) = socket.read_raw()? {
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
    pub fn send_packet(&self, can_id: u32, data: &[u8]) -> Result<()> {
        let socket = self.socket.lock().unwrap();
        socket.write_raw(can_id, data)
    }
}

/// Collection of CAN devices with frame dispatch, supporting both local and remote sockets.
#[cfg(feature = "remote")]
pub struct AnyCANDeviceCollection {
    devices: Arc<Mutex<HashMap<u32, Arc<Mutex<MotorDeviceCan>>>>>,
    socket: Arc<Mutex<super::socket::AnyCANSocket>>,
}

#[cfg(feature = "remote")]
impl AnyCANDeviceCollection {
    /// Create a new device collection from an AnyCANSocket.
    pub fn new(socket: super::socket::AnyCANSocket) -> Self {
        Self {
            devices: Arc::new(Mutex::new(HashMap::new())),
            socket: Arc::new(Mutex::new(socket)),
        }
    }

    /// Create from existing socket Arc.
    pub fn from_socket_arc(socket: Arc<Mutex<super::socket::AnyCANSocket>>) -> Self {
        Self {
            devices: Arc::new(Mutex::new(HashMap::new())),
            socket,
        }
    }

    /// Register a device with the collection.
    pub fn register_device(&self, device: &MotorDeviceCan) {
        let recv_id = device.recv_can_id();
        let device_clone = device.clone_inner();
        self.devices
            .lock()
            .unwrap()
            .insert(recv_id, Arc::new(Mutex::new(device_clone)));
    }

    /// Register device from Arc.
    pub fn register_device_internal(&self, device: Arc<Mutex<MotorDeviceCan>>) {
        let recv_id = device.lock().unwrap().recv_can_id();
        self.devices.lock().unwrap().insert(recv_id, device);
    }

    /// Unregister a device from the collection.
    pub fn unregister_device(&self, recv_can_id: u32) {
        self.devices.lock().unwrap().remove(&recv_can_id);
    }

    /// Get the number of registered devices.
    pub fn device_count(&self) -> usize {
        self.devices.lock().unwrap().len()
    }

    /// Get socket reference.
    pub fn socket(&self) -> Arc<Mutex<super::socket::AnyCANSocket>> {
        Arc::clone(&self.socket)
    }

    /// Get devices map.
    pub fn devices(&self) -> Arc<Mutex<HashMap<u32, Arc<Mutex<MotorDeviceCan>>>>> {
        Arc::clone(&self.devices)
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

    /// Receive all available frames with timeout for first frame.
    pub fn recv_all(&self, first_timeout_us: u64) -> Result<usize> {
        let socket = self.socket.lock().unwrap();
        let mut count = 0;

        // Wait for first frame with specified timeout
        if socket.is_data_available(first_timeout_us)? {
            if let Some((can_id, data)) = socket.read_raw()? {
                drop(socket); // Release lock before dispatch
                self.dispatch_frame(can_id, data);
                count += 1;

                // Read remaining frames with zero timeout (non-blocking)
                loop {
                    let socket = self.socket.lock().unwrap();
                    if !socket.is_data_available(0)? {
                        break;
                    }
                    if let Some((can_id, data)) = socket.read_raw()? {
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
    pub fn send_packet(&self, can_id: u32, data: &[u8]) -> Result<()> {
        let socket = self.socket.lock().unwrap();
        socket.write_raw(can_id, data)
    }
}
