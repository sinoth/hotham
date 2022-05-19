use hecs::Entity;
use nalgebra::Isometry3;

/// A component that represents the "side" or "handedness" that an entity is on
/// Used by components such as `Hand` and `Pointer` to identify which controller they should map to
#[derive(Debug, PartialEq, Clone, Copy, Eq, PartialOrd, Ord)]
pub enum Handedness {
    /// Left hand side
    Left,
    /// Right hand side
    Right,
}

/// A component that's added to an entity to represent a "hand" presence.
/// Used to give the player a feeling of immersion by allowing them to grab objects in the world
/// Requires `hands_system`
#[derive(Clone)]
pub struct Hand {
    /// How much has this hand been gripped?
    pub grip_value: f32,
    /// Which side is this hand on?
    pub handedness: Handedness,
    /// Have we grabbed something?
    pub grabbed_entity: Option<Entity>,
    /// Offset isometry to adjust 3D model
    pub offset: Isometry3<f32>,
    /// Are we in calibration mode?
    pub calibration_mode: bool,
    /// snarf
    pub needs_snapshot: bool,
    pub debug_hand: bool,
    /// Snapshot used for calibration
    pub calibration_snapshot: Isometry3<f32>,
}

impl Hand {
    /// Simple constructor given only the handedness
    pub fn new(handedness: Handedness) -> Self {
        Hand {
            grip_value: 0.0,
            handedness,
            grabbed_entity: None,
            offset: Isometry3::identity(),
            calibration_mode: false,
            needs_snapshot: false,
            debug_hand: false,
            calibration_snapshot: Isometry3::identity(),
        }
    }

    pub fn begin_calibration(&mut self) -> () {
        self.calibration_mode = true;
        self.needs_snapshot = true;
    }

    pub fn end_calibration(&mut self) -> () {
        self.calibration_mode = false;
    }
}
