//! CAN bus interface implementation.

pub mod device;
pub mod device_collection;
pub mod socket;

pub use device::*;
pub use device_collection::CANDeviceCollection;
pub use socket::CANSocket;
pub use socket::CanFdFrame;
pub use socket::CanFrame;

#[cfg(feature = "remote")]
pub use device_collection::AnyCANDeviceCollection;
#[cfg(feature = "remote")]
pub use socket::AnyCANSocket;
