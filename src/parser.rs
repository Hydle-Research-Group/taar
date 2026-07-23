use defmt::info;
use gcode::core::{
    BlockVisitor, CommandVisitor, ControlFlow, Diagnostics, HasDiagnostics, Noop, Number,
    ProgramVisitor, Span, Value,
};

enum Pending {
    G(u32),
    M(u32),
}

/// A gcode parser.
struct Parser {
    command: Command,

    // temporary parser parts
    current: Option<Pending>,
    x: Option<f32>,
    y: Option<f32>,
    z: Option<f32>,
    a: Option<f32>,
    p: Option<f32>,

    diagnostics: ParserDiagnostics,
}

impl Parser {
    pub fn new() -> Self {
        Self {
            command: Command::M02,
            current: None,
            x: None,
            y: None,
            z: None,
            a: None,
            p: None,
            diagnostics: ParserDiagnostics { error: None },
        }
    }

    pub fn reset(&mut self) {
        self.current = None;
        self.x = None;
        self.y = None;
        self.z = None;
        self.a = None;
        self.p = None;
    }
}

impl HasDiagnostics for Parser {
    fn diagnostics(&mut self) -> &mut dyn gcode::core::Diagnostics {
        &mut self.diagnostics
    }
}

struct ParserDiagnostics {
    error: Option<&'static str>,
}

impl Diagnostics for ParserDiagnostics {
    fn emit_unexpected(&mut self, actual: &str, expected: &[gcode::core::TokenType], span: Span) {
        self.error = Some("unexpected token or content");
    }

    fn emit_unknown_content(&mut self, text: &str, span: Span) {
        self.error = Some("invalid sequence or unknown content");
    }

    fn emit_parse_int_error(&mut self, value: &str, _error: core::num::ParseIntError, span: Span) {
        self.error = Some("error parsing integer");
    }
}

struct BlockCounter<'a>(&'a mut Parser);

impl ProgramVisitor for Parser {
    fn start_block(&mut self) -> ControlFlow<BlockCounter<'_>> {
        ControlFlow::Continue(BlockCounter(&mut *self))
    }
}

impl HasDiagnostics for BlockCounter<'_> {
    fn diagnostics(&mut self) -> &mut dyn gcode::core::Diagnostics {
        &mut self.0.diagnostics
    }
}

struct CommandCounter<'a>(&'a mut Parser);

impl<'a> CommandVisitor for CommandCounter<'a> {
    fn argument(&mut self, letter: char, value: Value<'_>, span: gcode::core::Span) {
        match (letter, value) {
            ('X', Value::Literal(v)) => self.0.x = Some(v),
            ('Y', Value::Literal(v)) => self.0.y = Some(v),
            ('Z', Value::Literal(v)) => self.0.z = Some(v),
            ('A', Value::Literal(v)) => self.0.a = Some(v),
            ('P', Value::Literal(v)) => self.0.p = Some(v),
            _ => {}
        }
    }

    fn end_command(self, span: Span) {
        if let Some(c) = &self.0.current {
            match c {
                Pending::G(g) => {
                    self.0.command = match g {
                        4 => Command::G4 {
                            ms: if let Some(p) = self.0.p {
                                p as u64
                            } else {
                                self.0.diagnostics.error = Some("missing argument 'P'");
                                self.0.reset();

                                return;
                            },
                        },
                        6 => Command::G6 {
                            x: self.0.x,
                            y: self.0.y,
                            z: self.0.z,
                            a: self.0.a,
                        },
                        60 => Command::G60,
                        61 => Command::G61,
                        92 => Command::G92 {
                            x: self.0.x,
                            y: self.0.y,
                            z: self.0.z,
                        },
                        _ => {
                            self.0.diagnostics.error = Some("unknown command");
                            self.0.reset();

                            return;
                        }
                    }
                }
                Pending::M(m) => {
                    self.0.command = match m {
                        2 => Command::M02,
                        _ => {
                            self.0.diagnostics.error = Some("unknown command");
                            self.0.reset();

                            return;
                        }
                    }
                }
            }
        }

        self.0.reset();
    }
}

impl BlockVisitor for BlockCounter<'_> {
    fn start_general_code(&mut self, _number: Number) -> ControlFlow<CommandCounter<'_>> {
        self.0.current = Some(Pending::G(_number.major()));

        ControlFlow::Continue(CommandCounter(&mut *self.0))
    }

    fn start_miscellaneous_code(&mut self, _number: Number) -> ControlFlow<CommandCounter<'_>> {
        self.0.current = Some(Pending::M(_number.major()));

        ControlFlow::Continue(CommandCounter(&mut *self.0))
    }
}

impl HasDiagnostics for CommandCounter<'_> {
    fn diagnostics(&mut self) -> &mut dyn gcode::core::Diagnostics {
        &mut self.0.diagnostics
    }
}

/// Describes a single GCODE command.
#[derive(Clone, Copy)]
pub enum Command {
    /// Pauses command execution for `ms` time
    G4 { ms: u64 },
    /// Moves an individual motor a single step
    G6 {
        x: Option<f32>,
        y: Option<f32>,
        z: Option<f32>,
        a: Option<f32>,
    },
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

/// Parses a gcode command into a [`Command`](taar::Command) object.
///
/// - `line`: the gcode line to parse
///
/// # Errors
///
/// This function will `Err` if the parsing fails, or a command is unknown.
pub fn parse(line: &str) -> Result<Command, &str> {
    let mut counter: Parser = Parser::new();
    gcode::core::parse(line, &mut counter);

    if let Some(e) = counter.diagnostics.error {
        return Err(e);
    }

    Ok(counter.command)
}
