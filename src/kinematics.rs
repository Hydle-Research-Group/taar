use micromath::F32Ext;

const ARM_LENGTH: f32 = 150.0;

/// Solves the rotations for each joint, performing inverse kinematics.
///
/// The returned value (base, shoulder, elbow, hand) is in degrees.
///
/// - `x`: the x coordinate (in millimeters)
/// - `y`: the y coordinate (in millimeters)
/// - `z`: the z coordinate (in millimeters)
pub fn inverse(x: f32, y: f32, z: f32) -> (f32, f32, f32, f32) {
    let h1 = (x.powi(2) + y.powi(2)).sqrt(); // base hypotenuse
    let base = y.atan2(x);

    let r = (h1.powi(2) + z.powi(2)).sqrt(); // shoulder + elbow hypotenuse
    let shoulder = z.atan2(h1) + (r / (2.0 * ARM_LENGTH)).acos();
    let elbow = (((2.0 * ARM_LENGTH.powi(2)) - r.powi(2)) / (2.0 * ARM_LENGTH.powi(2))).acos();
    let hand = -(shoulder + elbow); // hand is parallel to the ground

    (
        base.to_degrees(),
        shoulder.to_degrees(),
        elbow.to_degrees(),
        hand.to_degrees(),
    )
}

/// Solves the (x, y, z) position based on the current joint rotation, performing forward kinematics.
///
/// The returned value (x, y, z) is in millimeters.
///
/// - `base`: the base rotation (in degrees)
/// - `shoulder`: the shoulder rotation (in degrees)
/// - `elbow`: the elbow rotation (in degrees)
pub fn forward(base: f32, shoulder: f32, elbow: f32) -> (f32, f32, f32) {
    let base = base.to_radians();
    let shoulder = shoulder.to_radians();
    let elbow = elbow.to_radians();

    let r = ARM_LENGTH * shoulder.cos() + ARM_LENGTH * (shoulder + elbow).cos();
    let z = ARM_LENGTH * shoulder.sin() + ARM_LENGTH * (shoulder + elbow).sin();

    let x = r * base.cos();
    let y = r * base.sin();

    (x, y, z)
}
