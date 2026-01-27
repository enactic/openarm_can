//! Error types for OpenArm.

use thiserror::Error;

/// Errors that can occur in OpenArm operations.
#[derive(Error, Debug)]
pub enum OpenArmError {
    /// Socket operation failed.
    #[error("Socket error: {0}")]
    SocketError(String),

    /// Socket is not open.
    #[error("Socket not open")]
    SocketNotOpen,

    /// Invalid CAN ID.
    #[error("Invalid CAN ID: 0x{0:X}")]
    InvalidCanId(u32),

    /// CAN-FD not supported on this socket.
    #[error("CAN-FD not supported on standard CAN socket")]
    CanFdNotSupported,

    /// I/O error.
    #[error("I/O error: {0}")]
    IoError(#[from] std::io::Error),

    /// Index out of range.
    #[error("Index {0} out of range")]
    IndexOutOfRange(usize),

    /// Parameter count mismatch.
    #[error("Parameter count mismatch: expected {expected}, got {actual}")]
    ParamCountMismatch { expected: usize, actual: usize },
}

/// Result type for OpenArm operations.
pub type Result<T> = std::result::Result<T, OpenArmError>;
