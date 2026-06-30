#[derive(Clone, Copy)]
pub enum GCodeCommand {
    /// Pauses command execution for `ms` time
    G4 { ms: u64 },
    /// Stores the current position in memory
    G60,
    /// Loads a previously stored position
    G61,
    /// Moves the end effector to `x`, `y`, `z`
    G92 {
        x: Option<f32>,
        y: Option<f32>,
        z: Option<f32>,
    },
    /// Ends command execution
    M02,
}

/// Parses a GCODE sequence into an array of `GCodeCommand`.
///
/// - `src`: the GCODE sequence to parse
///
/// # Errors
///
/// This function will `Err` if the parsing fails, or a command is unknown.
pub fn parse(src: &str) -> Result<[GCodeCommand; 1024], &str> {
    let commands = [GCodeCommand::M02; 1024]; // allocate space for 1024 commands

    Ok(commands)
}
