//! CAN socket implementation using Linux SocketCAN.

use pyo3::exceptions::PyException;
use pyo3::prelude::*;
use socketcan::{CanFdSocket, CanSocket, EmbeddedFrame, Frame, Socket};
use std::os::unix::io::AsRawFd;
use std::time::Duration;

// Python exception for CAN socket errors.
pyo3::create_exception!(openarm_can, CANSocketException, PyException);

/// Standard CAN frame wrapper for Python.
#[pyclass(get_all)]
#[derive(Debug, Clone)]
pub struct CanFrame {
    pub can_id: u32,
    pub data: Vec<u8>,
}

#[pymethods]
impl CanFrame {
    #[new]
    #[pyo3(signature = (can_id, data))]
    pub fn new(can_id: u32, data: Vec<u8>) -> Self {
        Self { can_id, data }
    }

    fn __repr__(&self) -> String {
        format!("CanFrame(can_id=0x{:X}, data={:?})", self.can_id, self.data)
    }
}

/// CAN-FD frame wrapper for Python.
#[pyclass(get_all)]
#[derive(Debug, Clone)]
pub struct CanFdFrame {
    pub can_id: u32,
    pub data: Vec<u8>,
    pub flags: u8,
}

#[pymethods]
impl CanFdFrame {
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

/// Internal socket wrapper to handle both CAN and CAN-FD.
enum SocketInner {
    Can(CanSocket),
    CanFd(CanFdSocket),
}

/// Low-level CAN socket interface.
#[pyclass]
pub struct CANSocket {
    inner: Option<SocketInner>,
    interface: String,
    enable_fd: bool,
    recv_timeout_us: u64,
}

#[pymethods]
impl CANSocket {
    #[new]
    #[pyo3(signature = (interface, enable_fd=false, recv_timeout_us=100))]
    pub fn new(interface: String, enable_fd: bool, recv_timeout_us: u64) -> PyResult<Self> {
        let mut socket = Self {
            inner: None,
            interface,
            enable_fd,
            recv_timeout_us,
        };
        socket.initialize_socket()?;
        Ok(socket)
    }

    /// Initialize or reinitialize the CAN socket.
    pub fn initialize_socket(&mut self) -> PyResult<()> {
        if self.enable_fd {
            let sock = CanFdSocket::open(&self.interface).map_err(|e| {
                CANSocketException::new_err(format!(
                    "Failed to open CAN-FD socket on {}: {}",
                    self.interface, e
                ))
            })?;

            sock.set_read_timeout(Duration::from_micros(self.recv_timeout_us))
                .map_err(|e| {
                    CANSocketException::new_err(format!("Failed to set read timeout: {}", e))
                })?;

            self.inner = Some(SocketInner::CanFd(sock));
        } else {
            let sock = CanSocket::open(&self.interface).map_err(|e| {
                CANSocketException::new_err(format!(
                    "Failed to open CAN socket on {}: {}",
                    self.interface, e
                ))
            })?;

            sock.set_read_timeout(Duration::from_micros(self.recv_timeout_us))
                .map_err(|e| {
                    CANSocketException::new_err(format!("Failed to set read timeout: {}", e))
                })?;

            self.inner = Some(SocketInner::Can(sock));
        }
        Ok(())
    }

    /// Close the CAN socket.
    pub fn close(&mut self) {
        self.inner = None;
    }

    /// Check if socket is open.
    pub fn is_open(&self) -> bool {
        self.inner.is_some()
    }

    /// Get the interface name.
    #[getter]
    pub fn get_interface(&self) -> &str {
        &self.interface
    }

    /// Check if CAN-FD is enabled.
    #[getter]
    pub fn get_enable_fd(&self) -> bool {
        self.enable_fd
    }

    /// Write a standard CAN frame.
    pub fn write_can_frame(&self, frame: &CanFrame) -> PyResult<()> {
        let inner = self.inner.as_ref().ok_or_else(|| {
            CANSocketException::new_err("Socket not open")
        })?;

        let can_frame = socketcan::CanFrame::new(
            socketcan::StandardId::new(frame.can_id as u16).ok_or_else(|| {
                CANSocketException::new_err(format!("Invalid CAN ID: 0x{:X}", frame.can_id))
            })?,
            &frame.data,
        )
        .ok_or_else(|| CANSocketException::new_err("Failed to create CAN frame"))?;

        match inner {
            SocketInner::Can(sock) => {
                sock.write_frame(&can_frame).map_err(|e| {
                    CANSocketException::new_err(format!("Failed to write CAN frame: {}", e))
                })?;
            }
            SocketInner::CanFd(sock) => {
                sock.write_frame(&can_frame).map_err(|e| {
                    CANSocketException::new_err(format!("Failed to write CAN frame: {}", e))
                })?;
            }
        }
        Ok(())
    }

    /// Write a CAN-FD frame.
    pub fn write_canfd_frame(&self, frame: &CanFdFrame) -> PyResult<()> {
        let inner = self.inner.as_ref().ok_or_else(|| {
            CANSocketException::new_err("Socket not open")
        })?;

        match inner {
            SocketInner::Can(_) => {
                Err(CANSocketException::new_err(
                    "CAN-FD frames not supported on standard CAN socket",
                ))
            }
            SocketInner::CanFd(sock) => {
                let fd_frame = socketcan::CanFdFrame::new(
                    socketcan::StandardId::new(frame.can_id as u16).ok_or_else(|| {
                        CANSocketException::new_err(format!("Invalid CAN ID: 0x{:X}", frame.can_id))
                    })?,
                    &frame.data,
                )
                .ok_or_else(|| CANSocketException::new_err("Failed to create CAN-FD frame"))?;

                sock.write_frame(&fd_frame).map_err(|e| {
                    CANSocketException::new_err(format!("Failed to write CAN-FD frame: {}", e))
                })?;
                Ok(())
            }
        }
    }

    /// Read a standard CAN frame.
    pub fn read_can_frame(&self) -> PyResult<Option<CanFrame>> {
        let inner = self.inner.as_ref().ok_or_else(|| {
            CANSocketException::new_err("Socket not open")
        })?;

        match inner {
            SocketInner::Can(sock) => match sock.read_frame() {
                Ok(frame) => Ok(Some(CanFrame {
                    can_id: frame.raw_id(),
                    data: frame.data().to_vec(),
                })),
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => Ok(None),
                Err(e) if e.kind() == std::io::ErrorKind::TimedOut => Ok(None),
                Err(e) => Err(CANSocketException::new_err(format!(
                    "Failed to read CAN frame: {}",
                    e
                ))),
            },
            SocketInner::CanFd(sock) => match sock.read_frame() {
                Ok(frame) => Ok(Some(CanFrame {
                    can_id: frame.raw_id(),
                    data: frame.data().to_vec(),
                })),
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => Ok(None),
                Err(e) if e.kind() == std::io::ErrorKind::TimedOut => Ok(None),
                Err(e) => Err(CANSocketException::new_err(format!(
                    "Failed to read CAN frame: {}",
                    e
                ))),
            },
        }
    }

    /// Read a CAN-FD frame.
    pub fn read_canfd_frame(&self) -> PyResult<Option<CanFdFrame>> {
        let inner = self.inner.as_ref().ok_or_else(|| {
            CANSocketException::new_err("Socket not open")
        })?;

        match inner {
            SocketInner::Can(_) => Err(CANSocketException::new_err(
                "CAN-FD frames not supported on standard CAN socket",
            )),
            SocketInner::CanFd(sock) => match sock.read_frame() {
                Ok(frame) => Ok(Some(CanFdFrame {
                    can_id: frame.raw_id(),
                    data: frame.data().to_vec(),
                    flags: 0, // socketcan crate doesn't expose flags directly
                })),
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => Ok(None),
                Err(e) if e.kind() == std::io::ErrorKind::TimedOut => Ok(None),
                Err(e) => Err(CANSocketException::new_err(format!(
                    "Failed to read CAN-FD frame: {}",
                    e
                ))),
            },
        }
    }

    /// Check if data is available on the socket with timeout.
    pub fn is_data_available(&self, timeout_us: u64) -> PyResult<bool> {
        let inner = self.inner.as_ref().ok_or_else(|| {
            CANSocketException::new_err("Socket not open")
        })?;

        let fd = match inner {
            SocketInner::Can(sock) => sock.as_raw_fd(),
            SocketInner::CanFd(sock) => sock.as_raw_fd(),
        };

        // Use select() for timeout-based polling
        let mut read_fds = unsafe { std::mem::zeroed::<libc::fd_set>() };
        unsafe {
            libc::FD_ZERO(&mut read_fds);
            libc::FD_SET(fd, &mut read_fds);
        }

        let mut timeout = libc::timeval {
            tv_sec: (timeout_us / 1_000_000) as libc::time_t,
            tv_usec: (timeout_us % 1_000_000) as libc::suseconds_t,
        };

        let result = unsafe {
            libc::select(
                fd + 1,
                &mut read_fds,
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                &mut timeout,
            )
        };

        if result < 0 {
            let errno = std::io::Error::last_os_error();
            // EINTR means interrupted by signal (e.g., Ctrl+C) - just return false
            if errno.raw_os_error() == Some(libc::EINTR) {
                Ok(false)
            } else {
                Err(CANSocketException::new_err(format!("select() failed: {}", errno)))
            }
        } else {
            Ok(result > 0)
        }
    }

    /// Set receive timeout.
    pub fn set_recv_timeout(&mut self, timeout_us: u64) -> PyResult<()> {
        self.recv_timeout_us = timeout_us;

        if let Some(inner) = &self.inner {
            let duration = Duration::from_micros(timeout_us);
            match inner {
                SocketInner::Can(sock) => {
                    sock.set_read_timeout(duration).map_err(|e| {
                        CANSocketException::new_err(format!("Failed to set timeout: {}", e))
                    })?;
                }
                SocketInner::CanFd(sock) => {
                    sock.set_read_timeout(duration).map_err(|e| {
                        CANSocketException::new_err(format!("Failed to set timeout: {}", e))
                    })?;
                }
            }
        }
        Ok(())
    }

    fn __repr__(&self) -> String {
        format!(
            "CANSocket(interface='{}', enable_fd={}, open={})",
            self.interface, self.enable_fd, self.is_open()
        )
    }
}

impl CANSocket {
    /// Get raw file descriptor (internal use).
    pub(crate) fn raw_fd(&self) -> Option<i32> {
        self.inner.as_ref().map(|inner| match inner {
            SocketInner::Can(sock) => sock.as_raw_fd(),
            SocketInner::CanFd(sock) => sock.as_raw_fd(),
        })
    }

    /// Write raw bytes as CAN frame (internal use).
    pub(crate) fn write_raw(&self, can_id: u32, data: &[u8]) -> std::io::Result<()> {
        let inner = self.inner.as_ref().ok_or_else(|| {
            std::io::Error::new(std::io::ErrorKind::NotConnected, "Socket not open")
        })?;

        let frame = socketcan::CanFrame::new(
            socketcan::StandardId::new(can_id as u16).ok_or_else(|| {
                std::io::Error::new(std::io::ErrorKind::InvalidInput, "Invalid CAN ID")
            })?,
            data,
        )
        .ok_or_else(|| {
            std::io::Error::new(std::io::ErrorKind::InvalidInput, "Failed to create frame")
        })?;

        match inner {
            SocketInner::Can(sock) => sock.write_frame(&frame),
            SocketInner::CanFd(sock) => sock.write_frame(&frame),
        }
    }

    /// Write raw bytes as CAN-FD frame (internal use).
    pub(crate) fn write_raw_fd(&self, can_id: u32, data: &[u8]) -> std::io::Result<()> {
        let inner = self.inner.as_ref().ok_or_else(|| {
            std::io::Error::new(std::io::ErrorKind::NotConnected, "Socket not open")
        })?;

        match inner {
            SocketInner::Can(_) => Err(std::io::Error::new(
                std::io::ErrorKind::Unsupported,
                "CAN-FD not supported",
            )),
            SocketInner::CanFd(sock) => {
                let frame = socketcan::CanFdFrame::new(
                    socketcan::StandardId::new(can_id as u16).ok_or_else(|| {
                        std::io::Error::new(std::io::ErrorKind::InvalidInput, "Invalid CAN ID")
                    })?,
                    data,
                )
                .ok_or_else(|| {
                    std::io::Error::new(std::io::ErrorKind::InvalidInput, "Failed to create frame")
                })?;
                sock.write_frame(&frame)
            }
        }
    }

    /// Read raw CAN frame (internal use).
    pub(crate) fn read_raw(&self) -> std::io::Result<Option<(u32, Vec<u8>)>> {
        let inner = self.inner.as_ref().ok_or_else(|| {
            std::io::Error::new(std::io::ErrorKind::NotConnected, "Socket not open")
        })?;

        match inner {
            SocketInner::Can(sock) => match sock.read_frame() {
                Ok(frame) => Ok(Some((frame.raw_id(), frame.data().to_vec()))),
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => Ok(None),
                Err(e) if e.kind() == std::io::ErrorKind::TimedOut => Ok(None),
                Err(e) => Err(e),
            },
            SocketInner::CanFd(sock) => match sock.read_frame() {
                Ok(frame) => Ok(Some((frame.raw_id(), frame.data().to_vec()))),
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => Ok(None),
                Err(e) if e.kind() == std::io::ErrorKind::TimedOut => Ok(None),
                Err(e) => Err(e),
            },
        }
    }
}
