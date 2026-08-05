use atomic_float::AtomicF32;
use core::sync::atomic::{AtomicU32, Ordering};

pub enum MotionTarget {
    Base,
    Shoulder,
    Elbow,
    Hand,
}

pub struct DesiredMotion {
    angle: AtomicF32,
    duration_ms: AtomicU32,
}

impl DesiredMotion {
    pub const fn new() -> Self {
        Self {
            angle: AtomicF32::new(0.0),
            duration_ms: AtomicU32::new(0),
        }
    }

    pub fn set(&self, angle: f32, duration_ms: u64) {
        self.angle.store(angle, Ordering::Relaxed);
        self.duration_ms
            .store(duration_ms as u32, Ordering::Relaxed);
    }

    pub fn get(&self) -> (f32, u64) {
        (
            self.angle.load(Ordering::Relaxed),
            self.duration_ms.load(Ordering::Relaxed) as u64,
        )
    }
}

pub struct MotionTracker {
    // angle tracking
    current_base_angle: AtomicF32,
    current_shoulder_angle: AtomicF32,
    current_elbow_angle: AtomicF32,
    current_hand_angle: AtomicF32,

    // motion commands
    desired_base_motion: DesiredMotion,
    desired_shoulder_motion: DesiredMotion,
    desired_elbow_motion: DesiredMotion,
    desired_hand_motion: DesiredMotion,
}

impl MotionTracker {
    pub const fn new() -> Self {
        Self {
            current_base_angle: AtomicF32::new(0.0),
            current_shoulder_angle: AtomicF32::new(0.0),
            current_elbow_angle: AtomicF32::new(0.0),
            current_hand_angle: AtomicF32::new(0.0),

            desired_base_motion: DesiredMotion::new(),
            desired_shoulder_motion: DesiredMotion::new(),
            desired_elbow_motion: DesiredMotion::new(),
            desired_hand_motion: DesiredMotion::new(),
        }
    }

    pub fn set_base_angle(&self, angle: f32) {
        self.current_base_angle.store(angle, Ordering::Relaxed);
    }

    pub fn set_shoulder_angle(&self, angle: f32) {
        self.current_shoulder_angle.store(angle, Ordering::Relaxed);
    }

    pub fn set_elbow_angle(&self, angle: f32) {
        self.current_elbow_angle.store(angle, Ordering::Relaxed);
    }

    pub fn set_hand_angle(&self, angle: f32) {
        self.current_hand_angle.store(angle, Ordering::Relaxed);
    }

    /// Sets a current motion target angle and delay.
    ///
    /// - `target`: the `MotionTarget` to set
    /// - `angle`: the desired angle (in degrees)
    /// - `delay`: the desired delay (in milliseconds)
    pub fn set_target(&self, target: MotionTarget, angle: f32, delay: u64) {
        match target {
            MotionTarget::Base => self.desired_base_motion.set(angle, delay),
            MotionTarget::Shoulder => self.desired_shoulder_motion.set(angle, delay),
            MotionTarget::Elbow => self.desired_elbow_motion.set(angle, delay),
            MotionTarget::Hand => self.desired_hand_motion.set(angle, delay),
        }
    }

    pub fn get_base_angle(&self) -> f32 {
        self.current_base_angle.load(Ordering::Relaxed)
    }

    pub fn get_shoulder_angle(&self) -> f32 {
        self.current_shoulder_angle.load(Ordering::Relaxed)
    }

    pub fn get_elbow_angle(&self) -> f32 {
        self.current_elbow_angle.load(Ordering::Relaxed)
    }

    pub fn get_hand_angle(&self) -> f32 {
        self.current_hand_angle.load(Ordering::Relaxed)
    }

    /// Retrieves the current motion target angle and delay.
    ///
    /// Returns (angle, delay) where angle is in degrees and delay is in milliseconds.
    ///
    /// - `target`: the `MotionTarget` to get
    pub fn get_target(&self, target: MotionTarget) -> (f32, u64) {
        match target {
            MotionTarget::Base => self.desired_base_motion.get(),
            MotionTarget::Shoulder => self.desired_shoulder_motion.get(),
            MotionTarget::Elbow => self.desired_elbow_motion.get(),
            MotionTarget::Hand => self.desired_hand_motion.get(),
        }
    }
}
