//! High-level components for OpenArm control.

pub mod arm;
pub mod gripper;
pub mod openarm;

pub use arm::ArmComponent;
pub use gripper::GripperComponent;
pub use openarm::OpenArm;

#[cfg(feature = "remote")]
pub use arm::AnyArmComponent;
#[cfg(feature = "remote")]
pub use gripper::AnyGripperComponent;
#[cfg(feature = "remote")]
pub use openarm::RemoteOpenArm;
