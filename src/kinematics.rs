use micromath::F32Ext;

const ARM_LENGTH: f32 = 100.0;

/// Solves the delay for each joint, calculating a consistent delay across each angle.
///
/// The returned value (base, shoulder, elbow, hand) is in milliseconds.
///
/// - `shortest_delay`: the shortest delay allowed (in milliseconds)
/// - `base`: the base rotation (in degrees)
/// - `shoulder`: the shoulder rotation (in degrees)
/// - `elbow`: the elbow rotation (in degrees)
/// - `hand`: the hand rotation (in degrees)
pub fn delay(
    shortest_delay: u64,
    base: f32,
    shoulder: f32,
    elbow: f32,
    hand: f32,
) -> (u64, u64, u64, u64) {
    let mut joints = [(0, base), (1, shoulder), (2, elbow), (3, hand)];

    for i in 1..joints.len() {
        let mut j = i;
        while j > 0 && joints[j - 1].1 > joints[j].1 {
            joints.swap(j - 1, j);
            j -= 1;
        }
    }

    let largest = joints.last().unwrap().1;
    let mut delays = [0u64; 4];

    // sort by the angle
    for (index, angle) in joints {
        delays[index] = ((largest / angle) * shortest_delay as f32) as u64;
    }

    (delays[0] * 12, delays[1], delays[2], delays[3] * 12) // account for shoulder/elbow ratios (6 * 2 delays)
}

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
    let elbow = (1.0 - (r.powi(2) / (2.0 * ARM_LENGTH.powi(2)))).acos();
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
