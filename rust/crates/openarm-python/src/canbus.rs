//! Python wrappers for canbus types.

use pyo3::prelude::*;

use crate::error::IntoPyResult;

/// Standard CAN frame wrapper for Python.
#[pyclass(name = "CanFrame", get_all)]
#[derive(Debug, Clone)]
pub struct PyCanFrame {
    pub can_id: u32,
    pub data: Vec<u8>,
}

#[pymethods]
impl PyCanFrame {
    #[new]
    #[pyo3(signature = (can_id, data))]
    pub fn new(can_id: u32, data: Vec<u8>) -> Self {
        Self { can_id, data }
    }

    fn __repr__(&self) -> String {
        format!("CanFrame(can_id=0x{:X}, data={:?})", self.can_id, self.data)
    }
}

impl From<openarm::CanFrame> for PyCanFrame {
    fn from(f: openarm::CanFrame) -> Self {
        Self {
            can_id: f.can_id,
            data: f.data,
        }
    }
}

impl From<&PyCanFrame> for openarm::CanFrame {
    fn from(f: &PyCanFrame) -> Self {
        Self {
            can_id: f.can_id,
            data: f.data.clone(),
        }
    }
}

/// CAN-FD frame wrapper for Python.
#[pyclass(name = "CanFdFrame", get_all)]
#[derive(Debug, Clone)]
pub struct PyCanFdFrame {
    pub can_id: u32,
    pub data: Vec<u8>,
    pub flags: u8,
}

#[pymethods]
impl PyCanFdFrame {
    #[new]
    #[pyo3(signature = (can_id, data, flags=0))]
    pub fn new(can_id: u32, data: Vec<u8>, flags: u8) -> Self {
        Self { can_id, data, flags }
    }

    fn __repr__(&self) -> String {
        format!(
            "CanFdFrame(can_id=0x{:X}, data={:?}, flags={})",
            self.can_id, self.data, self.flags
        )
    }
}

impl From<openarm::CanFdFrame> for PyCanFdFrame {
    fn from(f: openarm::CanFdFrame) -> Self {
        Self {
            can_id: f.can_id,
            data: f.data,
            flags: f.flags,
        }
    }
}

impl From<&PyCanFdFrame> for openarm::CanFdFrame {
    fn from(f: &PyCanFdFrame) -> Self {
        Self {
            can_id: f.can_id,
            data: f.data.clone(),
            flags: f.flags,
        }
    }
}

/// Low-level CAN socket interface wrapper.
#[pyclass(name = "CANSocket")]
pub struct PyCANSocket {
    inner: openarm::CANSocket,
}

#[pymethods]
impl PyCANSocket {
    #[new]
    #[pyo3(signature = (interface, enable_fd=false, recv_timeout_us=100))]
    pub fn new(interface: String, enable_fd: bool, recv_timeout_us: u64) -> PyResult<Self> {
        let inner = openarm::CANSocket::new(interface, enable_fd, recv_timeout_us).into_py_result()?;
        Ok(Self { inner })
    }

    /// Initialize or reinitialize the CAN socket.
    pub fn initialize_socket(&mut self) -> PyResult<()> {
        self.inner.initialize_socket().into_py_result()
    }

    /// Close the CAN socket.
    pub fn close(&mut self) {
        self.inner.close();
    }

    /// Check if socket is open.
    pub fn is_open(&self) -> bool {
        self.inner.is_open()
    }

    /// Get the interface name.
    #[getter]
    pub fn get_interface(&self) -> &str {
        self.inner.interface()
    }

    /// Check if CAN-FD is enabled.
    #[getter]
    pub fn get_enable_fd(&self) -> bool {
        self.inner.enable_fd()
    }

    /// Write a standard CAN frame.
    pub fn write_can_frame(&self, frame: &PyCanFrame) -> PyResult<()> {
        self.inner.write_can_frame(&frame.into()).into_py_result()
    }

    /// Write a CAN-FD frame.
    pub fn write_canfd_frame(&self, frame: &PyCanFdFrame) -> PyResult<()> {
        self.inner.write_canfd_frame(&frame.into()).into_py_result()
    }

    /// Read a standard CAN frame.
    pub fn read_can_frame(&self) -> PyResult<Option<PyCanFrame>> {
        self.inner
            .read_can_frame()
            .into_py_result()
            .map(|opt| opt.map(PyCanFrame::from))
    }

    /// Read a CAN-FD frame.
    pub fn read_canfd_frame(&self) -> PyResult<Option<PyCanFdFrame>> {
        self.inner
            .read_canfd_frame()
            .into_py_result()
            .map(|opt| opt.map(PyCanFdFrame::from))
    }

    /// Check if data is available on the socket with timeout.
    pub fn is_data_available(&self, timeout_us: u64) -> PyResult<bool> {
        self.inner.is_data_available(timeout_us).into_py_result()
    }

    /// Set receive timeout.
    pub fn set_recv_timeout(&mut self, timeout_us: u64) -> PyResult<()> {
        self.inner.set_recv_timeout(timeout_us).into_py_result()
    }

    fn __repr__(&self) -> String {
        format!(
            "CANSocket(interface='{}', enable_fd={}, open={})",
            self.inner.interface(),
            self.inner.enable_fd(),
            self.inner.is_open()
        )
    }
}

/// Abstract base CAN device wrapper.
#[pyclass(name = "CANDevice", subclass)]
#[derive(Clone)]
pub struct PyCANDevice {
    send_can_id: u32,
    recv_can_id: u32,
}

#[pymethods]
impl PyCANDevice {
    #[new]
    #[pyo3(signature = (send_can_id, recv_can_id))]
    pub fn new(send_can_id: u32, recv_can_id: u32) -> Self {
        Self {
            send_can_id,
            recv_can_id,
        }
    }

    /// Get the send CAN ID.
    #[getter]
    pub fn get_send_can_id(&self) -> u32 {
        self.send_can_id
    }

    /// Get the receive CAN ID.
    #[getter]
    pub fn get_recv_can_id(&self) -> u32 {
        self.recv_can_id
    }

    fn __repr__(&self) -> String {
        format!(
            "CANDevice(send_id=0x{:X}, recv_id=0x{:X})",
            self.send_can_id, self.recv_can_id
        )
    }
}
