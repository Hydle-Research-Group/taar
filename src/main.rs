#![no_std]
#![no_main]

use atomic_float::AtomicF32;
use core::f32;
use core::panic::PanicInfo;
use core::sync::atomic::{AtomicBool, AtomicI32, Ordering};
use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_stm32::gpio::{AnyPin, Input, Level, Output, Pull, Speed};
use embassy_stm32::peripherals::{DMA1_CH1, DMA1_CH2, USART2};
use embassy_stm32::usart::{Config, Uart};
use embassy_stm32::{Peri, bind_interrupts};
use embassy_time::Timer;
use taar::{
    kinematics::{forward, inverse},
    parser::{Command, parse},
};
use {defmt_rtt as _, panic_probe as _};

const BASE_STEPS_PER_REVOLUTION: u32 = 200 * 8; // 200 steps/rev * microsteps (direct drive)
const SHOULDER_STEPS_PER_REVOLUTION: u32 = 200 * 8 * 6; // 200 steps/rev * microsteps * 6:1 ratio
const ELBOW_STEPS_PER_REVOLUTION: u32 = 200 * 8 * 6; // 200 steps/rev * microsteps * 6:1 ratio
const HAND_STEPS_PER_REVOLUTION: u32 = 200 * 8; // 200 steps/rev * microsteps (direct drive)
/// Max = 90.1 degrees, Min = -90.1 degrees
const BASE_BOUNDS: (f32, f32) = (90.1, -90.1);
/// Max = 110.1 degrees, Min = -0.1 degrees
const SHOULDER_BOUNDS: (f32, f32) = (110.1, -0.1);
/// Max = 0.1 degrees, Min = -110.1 degrees
const ELBOW_BOUNDS: (f32, f32) = (0.1, -110.1);
/// Max = 90.1 degrees, Min = -90.1 degrees
const HAND_BOUNDS: (f32, f32) = (90.1, -90.1);
const CURRENT_BASE_ANGLE: AtomicF32 = AtomicF32::new(0.0);
const CURRENT_SHOULDER_ANGLE: AtomicF32 = AtomicF32::new(0.0);
const CURRENT_ELBOW_ANGLE: AtomicF32 = AtomicF32::new(0.0);
const CURRENT_HAND_ANGLE: AtomicF32 = AtomicF32::new(0.0);

bind_interrupts!(struct Irqs {
    USART2 => embassy_stm32::usart::InterruptHandler<USART2>;
    DMA1_CHANNEL1 => embassy_stm32::dma::InterruptHandler<DMA1_CH1>;
    DMA1_CHANNEL2 => embassy_stm32::dma::InterruptHandler<DMA1_CH2>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    spawner.spawn(update_angles().unwrap());

    let mut uart_config = Config::default();
    uart_config.baudrate = 115200;

    let mut uart = Uart::new(
        p.USART2,
        p.PA3,
        p.PA2,
        p.DMA1_CH1,
        p.DMA1_CH2,
        Irqs,
        uart_config,
    )
    .unwrap();

    loop {
        let mut buf = [0u8; 128];
        let n = uart.read_until_idle(&mut buf).await.unwrap();

        if let Ok(msg) = str::from_utf8(&buf[..n]) {
            match parse::<512>(msg) {
                Ok(commands) => {
                    for command in commands {
                        match command {
                            Command::G4 { ms } => Timer::after_millis(ms).await,
                            Command::M02 => {
                                break;
                            }
                            _ => {}
                        }
                    }
                }
                Err(e) => {
                    let mut buf = [0u8; 128];
                    let s =
                        format_no_std::show(&mut buf, format_args!("{{\"error\": \"{}\"}}\n", e))
                            .unwrap();

                    uart.write(s.as_bytes()).await.unwrap();

                    continue;
                }
            }
        } else {
            uart.write(b"{\"error\": \"invalid UTF-8 sequence\"}\n")
                .await
                .unwrap();

            continue;
        }
    }
}

#[embassy_executor::task]
async fn update_angles() {
    loop {}
}

async fn move_base_stepper(
    step_pin: &mut Output<'static>,
    dir_pin: &mut Output<'static>,
    delay_per_step: u64,
    angle: f32,
) {
    if angle < 0.0 {
        dir_pin.set_high();
    } else {
        dir_pin.set_low();
    }

    let steps = angle * (BASE_STEPS_PER_REVOLUTION as f32 / 360.0);

    for _ in 0..(steps.abs() as usize) {
        step_pin.set_high();
        Timer::after_millis(delay_per_step).await;
        step_pin.set_low();
        Timer::after_millis(delay_per_step).await;
    }
}

async fn move_shoulder_stepper(
    step_pin: &mut Output<'static>,
    dir_pin: &mut Output<'static>,
    delay_per_step: u64,
    angle: f32,
) {
    if angle < 0.0 {
        dir_pin.set_high();
    } else {
        dir_pin.set_low();
    }

    let steps = angle * (SHOULDER_STEPS_PER_REVOLUTION as f32 / 360.0);

    for _ in 0..(steps.abs() as usize) {
        step_pin.set_high();
        Timer::after_millis(delay_per_step).await;
        step_pin.set_low();
        Timer::after_millis(delay_per_step).await;
    }
}

async fn move_elbow_stepper(
    step_pin: &mut Output<'static>,
    dir_pin: &mut Output<'static>,
    delay_per_step: u64,
    angle: f32,
) {
    if angle < 0.0 {
        dir_pin.set_high();
    } else {
        dir_pin.set_low();
    }

    let steps = angle * (ELBOW_STEPS_PER_REVOLUTION as f32 / 360.0);

    for _ in 0..(steps.abs() as usize) {
        step_pin.set_high();
        Timer::after_millis(delay_per_step).await;
        step_pin.set_low();
        Timer::after_millis(delay_per_step).await;
    }
}

async fn move_hand_stepper(
    step_pin: &mut Output<'static>,
    dir_pin: &mut Output<'static>,
    delay_per_step: u64,
    angle: f32,
) {
    if angle < 0.0 {
        dir_pin.set_high();
    } else {
        dir_pin.set_low();
    }

    let steps = angle * (HAND_STEPS_PER_REVOLUTION as f32 / 360.0);

    for _ in 0..(steps.abs() as usize) {
        step_pin.set_high();
        Timer::after_millis(delay_per_step).await;
        step_pin.set_low();
        Timer::after_millis(delay_per_step).await;
    }
}

fn in_base_bounds(angle: f32) -> bool {
    (BASE_BOUNDS.1..BASE_BOUNDS.0).contains(&angle)
}

fn in_shoulder_bounds(angle: f32) -> bool {
    (SHOULDER_BOUNDS.1..SHOULDER_BOUNDS.0).contains(&angle)
}

fn in_elbow_bounds(angle: f32) -> bool {
    (ELBOW_BOUNDS.1..ELBOW_BOUNDS.0).contains(&angle)
}

fn in_hand_bounds(angle: f32) -> bool {
    (HAND_BOUNDS.1..HAND_BOUNDS.0).contains(&angle)
}

fn get_current_base_angle() -> f32 {
    CURRENT_BASE_ANGLE.load(Ordering::Relaxed)
}

fn get_current_shoulder_angle() -> f32 {
    CURRENT_SHOULDER_ANGLE.load(Ordering::Relaxed)
}

fn get_current_elbow_angle() -> f32 {
    CURRENT_ELBOW_ANGLE.load(Ordering::Relaxed)
}

fn get_current_hand_angle() -> f32 {
    CURRENT_HAND_ANGLE.load(Ordering::Relaxed)
}
