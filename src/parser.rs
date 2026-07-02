use gcode::core::{
    BlockVisitor, CommandVisitor, ControlFlow, HasDiagnostics, Noop, Number, ProgramVisitor, Value,
};

/// A gcode parser, where `N` defines the allocation space for commands.
struct Parser<const N: usize> {
    commands: [Command; N],
    len: usize,

    x: Option<f32>,
    y: Option<f32>,
    z: Option<f32>,
    p: Option<f32>,

    error: Option<&'static str>,
    diag: Noop,
}

impl<const N: usize> Parser<N> {
    pub fn new() -> Self {
        Self {
            commands: [Command::M02; N],
            len: 0,
            x: None,
            y: None,
            z: None,
            p: None,
            error: None,
            diag: Noop::default(),
        }
    }
}

impl<const N: usize> HasDiagnostics for Parser<N> {
    fn diagnostics(&mut self) -> &mut dyn gcode::core::Diagnostics {
        &mut self.diag
    }
}

struct BlockCounter<'a, const N: usize>(&'a mut Parser<N>);

impl<const N: usize> ProgramVisitor for Parser<N> {
    fn start_block(&mut self) -> ControlFlow<BlockCounter<'_, N>> {
        ControlFlow::Continue(BlockCounter(&mut *self))
    }
}

impl<const N: usize> HasDiagnostics for BlockCounter<'_, N> {
    fn diagnostics(&mut self) -> &mut dyn gcode::core::Diagnostics {
        &mut self.0.diag
    }
}

struct CommandCounter<'a, const N: usize>(&'a mut Parser<N>);

impl<'a, const N: usize> CommandVisitor for CommandCounter<'a, N> {
    fn argument(&mut self, letter: char, value: Value<'_>, span: gcode::core::Span) {
        match (letter, value) {
            ('X', Value::Literal(v)) => self.0.x = Some(v),
            ('Y', Value::Literal(v)) => self.0.y = Some(v),
            ('Z', Value::Literal(v)) => self.0.z = Some(v),
            ('P', Value::Literal(v)) => self.0.p = Some(v),
            _ => {}
        }
    }
}

impl<const N: usize> BlockVisitor for BlockCounter<'_, N> {
    fn start_general_code(&mut self, _number: Number) -> ControlFlow<CommandCounter<'_, N>> {
        self.0.commands[self.0.len] = match _number.major() {
            4 => Command::G4 {
                ms: if let Some(p) = self.0.p {
                    p as u64
                } else {
                    self.0.error = Some("missing argument 'P'");

                    return ControlFlow::Break(());
                },
            },
            60 => Command::G60,
            61 => Command::G61,
            92 => Command::G92 {
                x: self.0.x,
                y: self.0.y,
                z: self.0.z,
            },
            _ => {
                self.0.error = Some("unknown command");

                return ControlFlow::Break(());
            }
        };

        self.0.len += 1;
        ControlFlow::Continue(CommandCounter(&mut *self.0))
    }

    fn start_miscellaneous_code(&mut self, _number: Number) -> ControlFlow<CommandCounter<'_, N>> {
        self.0.commands[self.0.len] = match _number.major() {
            2 => Command::M02,
            _ => {
                self.0.error = Some("unknown command");

                return ControlFlow::Break(());
            }
        };

        self.0.len += 1;
        ControlFlow::Continue(CommandCounter(&mut *self.0))
    }
}

impl<const N: usize> HasDiagnostics for CommandCounter<'_, N> {
    fn diagnostics(&mut self) -> &mut dyn gcode::core::Diagnostics {
        &mut self.0.diag
    }
}

/// Describes a single GCODE command.
#[derive(Clone, Copy)]
pub enum Command {
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

/// Parses a GCODE sequence into an array `N` size of [`Command`](taar::Command) objects.
///
/// - `src`: the sequence to parse
///
/// # Errors
///
/// This function will `Err` if the parsing fails, or a command is unknown.
pub fn parse<const N: usize>(src: &str) -> Result<[Command; N], &str> {
    let mut counter: Parser<N> = Parser::new();
    gcode::core::parse(src, &mut counter);

    if let Some(e) = counter.error {
        return Err(e);
    }

    Ok(counter.commands)
}
