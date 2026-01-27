//! Error conversion between OpenArm errors and Python exceptions.

use openarm::OpenArmError;
use pyo3::exceptions::{PyException, PyIOError, PyIndexError, PyValueError};
use pyo3::prelude::*;

// Custom exception for CAN socket errors
pyo3::create_exception!(openarm_can, CANSocketException, PyException);

/// Trait for converting OpenArm Results to PyResults.
pub trait IntoPyResult<T> {
    fn into_py_result(self) -> PyResult<T>;
}

impl<T> IntoPyResult<T> for openarm::Result<T> {
    fn into_py_result(self) -> PyResult<T> {
        self.map_err(|e| match e {
            OpenArmError::SocketError(msg) => CANSocketException::new_err(msg),
            OpenArmError::SocketNotOpen => CANSocketException::new_err("Socket not open"),
            OpenArmError::InvalidCanId(id) => {
                CANSocketException::new_err(format!("Invalid CAN ID: 0x{:X}", id))
            }
            OpenArmError::CanFdNotSupported => {
                CANSocketException::new_err("CAN-FD not supported on standard CAN socket")
            }
            OpenArmError::IoError(io_err) => PyIOError::new_err(format!("I/O error: {}", io_err)),
            OpenArmError::IndexOutOfRange(idx) => {
                PyIndexError::new_err(format!("Index {} out of range", idx))
            }
            OpenArmError::ParamCountMismatch { expected, actual } => {
                PyValueError::new_err(format!("Expected {} params, got {}", expected, actual))
            }
        })
    }
}
