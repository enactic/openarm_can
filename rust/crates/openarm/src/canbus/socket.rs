//! CAN socket implementation using Linux SocketCAN.

use crate::error::{OpenArmError, Result};
use socketcan::{CanFdSocket, CanSocket, EmbeddedFrame, Frame, Socket};
use std::os::unix::io::AsRawFd;
use std::time::Duration;

/// Standard CAN frame.
#[derive(Debug, Clone)]
pub struct CanFrame {
    pub can_id: u32,
    pub data: Vec<u8>,
}

impl CanFrame {
    /// Create a new CAN frame.
    pub fn new(can_id: u32, data: Vec<u8>) -> Self {
        Self { can_id, data }
    }
}

/// CAN-FD frame.
#[derive(Debug, Clone)]
pub struct CanFdFrame {
    pub can_id: u32,
    pub data: Vec<u8>,
    pub flags: u8,
}

impl CanFdFrame {
    /// Create a new CAN-FD frame.
    pub fn new(can_id: u32, data: Vec<u8>, flags: u8) -> Self {
        Self { can_id, data, flags }
    }
}

/// Internal socket wrapper to handle both CAN and CAN-FD.
enum SocketInner {
    Can(CanSocket),
    CanFd(CanFdSocket),
}

/// Low-level CAN socket interface.
pub struct CANSocket {
    inner: Option<SocketInner>,
    interface: String,
    enable_fd: bool,
    recv_timeout_us: u64,
}

impl CANSocket {
    /// Create a new CAN socket.
    pub fn new(interface: String, enable_fd: bool, recv_timeout_us: u64) -> Result<Self> {
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
    pub fn initialize_socket(&mut self) -> Result<()> {
        if self.enable_fd {
            let sock = CanFdSocket::open(&self.interface).map_err(|e| {
                OpenArmError::SocketError(format!(
                    "Failed to open CAN-FD socket on {}: {}",
                    self.interface, e
                ))
            })?;

            sock.set_read_timeout(Duration::from_micros(self.recv_timeout_us))
                .map_err(|e| {
                    OpenArmError::SocketError(format!("Failed to set read timeout: {}", e))
                })?;

            self.inner = Some(SocketInner::CanFd(sock));
        } else {
            let sock = CanSocket::open(&self.interface).map_err(|e| {
                OpenArmError::SocketError(format!(
                    "Failed to open CAN socket on {}: {}",
                    self.interface, e
                ))
            })?;

            sock.set_read_timeout(Duration::from_micros(self.recv_timeout_us))
                .map_err(|e| {
                    OpenArmError::SocketError(format!("Failed to set read timeout: {}", e))
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
    pub fn interface(&self) -> &str {
        &self.interface
    }

    /// Check if CAN-FD is enabled.
    pub fn enable_fd(&self) -> bool {
        self.enable_fd
    }

    /// Write a standard CAN frame.
    pub fn write_can_frame(&self, frame: &CanFrame) -> Result<()> {
        let inner = self.inner.as_ref().ok_or(OpenArmError::SocketNotOpen)?;

        let can_frame = socketcan::CanFrame::new(
            socketcan::StandardId::new(frame.can_id as u16)
                .ok_or(OpenArmError::InvalidCanId(frame.can_id))?,
            &frame.data,
        )
        .ok_or_else(|| OpenArmError::SocketError("Failed to create CAN frame".to_string()))?;

        match inner {
            SocketInner::Can(sock) => {
                sock.write_frame(&can_frame).map_err(|e| {
                    OpenArmError::SocketError(format!("Failed to write CAN frame: {}", e))
                })?;
            }
            SocketInner::CanFd(sock) => {
                sock.write_frame(&can_frame).map_err(|e| {
                    OpenArmError::SocketError(format!("Failed to write CAN frame: {}", e))
                })?;
            }
        }
        Ok(())
    }

    /// Write a CAN-FD frame.
    pub fn write_canfd_frame(&self, frame: &CanFdFrame) -> Result<()> {
        let inner = self.inner.as_ref().ok_or(OpenArmError::SocketNotOpen)?;

        match inner {
            SocketInner::Can(_) => Err(OpenArmError::CanFdNotSupported),
            SocketInner::CanFd(sock) => {
                let fd_frame = socketcan::CanFdFrame::new(
                    socketcan::StandardId::new(frame.can_id as u16)
                        .ok_or(OpenArmError::InvalidCanId(frame.can_id))?,
                    &frame.data,
                )
                .ok_or_else(|| {
                    OpenArmError::SocketError("Failed to create CAN-FD frame".to_string())
                })?;

                sock.write_frame(&fd_frame).map_err(|e| {
                    OpenArmError::SocketError(format!("Failed to write CAN-FD frame: {}", e))
                })?;
                Ok(())
            }
        }
    }

    /// Read a standard CAN frame.
    pub fn read_can_frame(&self) -> Result<Option<CanFrame>> {
        let inner = self.inner.as_ref().ok_or(OpenArmError::SocketNotOpen)?;

        match inner {
            SocketInner::Can(sock) => match sock.read_frame() {
                Ok(frame) => Ok(Some(CanFrame {
                    can_id: frame.raw_id(),
                    data: frame.data().to_vec(),
                })),
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => Ok(None),
                Err(e) if e.kind() == std::io::ErrorKind::TimedOut => Ok(None),
                Err(e) => Err(OpenArmError::IoError(e)),
            },
            SocketInner::CanFd(sock) => match sock.read_frame() {
                Ok(frame) => Ok(Some(CanFrame {
                    can_id: frame.raw_id(),
                    data: frame.data().to_vec(),
                })),
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => Ok(None),
                Err(e) if e.kind() == std::io::ErrorKind::TimedOut => Ok(None),
                Err(e) => Err(OpenArmError::IoError(e)),
            },
        }
    }

    /// Read a CAN-FD frame.
    pub fn read_canfd_frame(&self) -> Result<Option<CanFdFrame>> {
        let inner = self.inner.as_ref().ok_or(OpenArmError::SocketNotOpen)?;

        match inner {
            SocketInner::Can(_) => Err(OpenArmError::CanFdNotSupported),
            SocketInner::CanFd(sock) => match sock.read_frame() {
                Ok(frame) => Ok(Some(CanFdFrame {
                    can_id: frame.raw_id(),
                    data: frame.data().to_vec(),
                    flags: 0, // socketcan crate doesn't expose flags directly
                })),
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => Ok(None),
                Err(e) if e.kind() == std::io::ErrorKind::TimedOut => Ok(None),
                Err(e) => Err(OpenArmError::IoError(e)),
            },
        }
    }

    /// Check if data is available on the socket with timeout.
    pub fn is_data_available(&self, timeout_us: u64) -> Result<bool> {
        let inner = self.inner.as_ref().ok_or(OpenArmError::SocketNotOpen)?;

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
                Err(OpenArmError::IoError(errno))
            }
        } else {
            Ok(result > 0)
        }
    }

    /// Set receive timeout.
    pub fn set_recv_timeout(&mut self, timeout_us: u64) -> Result<()> {
        self.recv_timeout_us = timeout_us;

        if let Some(inner) = &self.inner {
            let duration = Duration::from_micros(timeout_us);
            match inner {
                SocketInner::Can(sock) => {
                    sock.set_read_timeout(duration).map_err(|e| {
                        OpenArmError::SocketError(format!("Failed to set timeout: {}", e))
                    })?;
                }
                SocketInner::CanFd(sock) => {
                    sock.set_read_timeout(duration).map_err(|e| {
                        OpenArmError::SocketError(format!("Failed to set timeout: {}", e))
                    })?;
                }
            }
        }
        Ok(())
    }

    /// Get raw file descriptor.
    pub fn raw_fd(&self) -> Option<i32> {
        self.inner.as_ref().map(|inner| match inner {
            SocketInner::Can(sock) => sock.as_raw_fd(),
            SocketInner::CanFd(sock) => sock.as_raw_fd(),
        })
    }

    /// Write raw bytes as CAN frame.
    pub fn write_raw(&self, can_id: u32, data: &[u8]) -> Result<()> {
        let inner = self.inner.as_ref().ok_or(OpenArmError::SocketNotOpen)?;

        let frame = socketcan::CanFrame::new(
            socketcan::StandardId::new(can_id as u16)
                .ok_or(OpenArmError::InvalidCanId(can_id))?,
            data,
        )
        .ok_or_else(|| OpenArmError::SocketError("Failed to create frame".to_string()))?;

        match inner {
            SocketInner::Can(sock) => sock.write_frame(&frame)?,
            SocketInner::CanFd(sock) => sock.write_frame(&frame)?,
        }
        Ok(())
    }

    /// Write raw bytes as CAN-FD frame.
    pub fn write_raw_fd(&self, can_id: u32, data: &[u8]) -> Result<()> {
        let inner = self.inner.as_ref().ok_or(OpenArmError::SocketNotOpen)?;

        match inner {
            SocketInner::Can(_) => Err(OpenArmError::CanFdNotSupported),
            SocketInner::CanFd(sock) => {
                let frame = socketcan::CanFdFrame::new(
                    socketcan::StandardId::new(can_id as u16)
                        .ok_or(OpenArmError::InvalidCanId(can_id))?,
                    data,
                )
                .ok_or_else(|| OpenArmError::SocketError("Failed to create frame".to_string()))?;
                sock.write_frame(&frame)?;
                Ok(())
            }
        }
    }

    /// Read raw CAN frame.
    pub fn read_raw(&self) -> Result<Option<(u32, Vec<u8>)>> {
        let inner = self.inner.as_ref().ok_or(OpenArmError::SocketNotOpen)?;

        match inner {
            SocketInner::Can(sock) => match sock.read_frame() {
                Ok(frame) => Ok(Some((frame.raw_id(), frame.data().to_vec()))),
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => Ok(None),
                Err(e) if e.kind() == std::io::ErrorKind::TimedOut => Ok(None),
                Err(e) => Err(OpenArmError::IoError(e)),
            },
            SocketInner::CanFd(sock) => match sock.read_frame() {
                Ok(frame) => Ok(Some((frame.raw_id(), frame.data().to_vec()))),
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => Ok(None),
                Err(e) if e.kind() == std::io::ErrorKind::TimedOut => Ok(None),
                Err(e) => Err(OpenArmError::IoError(e)),
            },
        }
    }
}
