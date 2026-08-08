#![no_std]
#![no_main]

use core::f32;
use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::yield_now;
use embassy_stm32::adc::{Adc, AdcConfig, SampleTime};
use embassy_stm32::gpio::Output;
use embassy_stm32::peripherals::{ADC1, DMA1_CH1, DMA1_CH2, PA0, USART2};
use embassy_stm32::usart::{Config, Uart};
use embassy_stm32::{Peri, bind_interrupts};
use embassy_time::Timer;
use taar::{
    kinematics::{delay, forward, inverse},
    motion::{MotionTarget, MotionTracker},
    parser::{Command, parse},
};
use {defmt_rtt as _, panic_probe as _};

const BASE_STEPS_PER_REVOLUTION: u32 = 200 * 8; // 200 steps/rev * microsteps (direct drive)
const SHOULDER_STEPS_PER_REVOLUTION: u32 = 200 * 8 * 6; // 200 steps/rev * microsteps * 6:1 ratio
const ELBOW_STEPS_PER_REVOLUTION: u32 = 200 * 8 * 6; // 200 steps/rev * microsteps * 6:1 ratio
const HAND_STEPS_PER_REVOLUTION: u32 = 200 * 8; // 200 steps/rev * microsteps (direct drive)
/// Max = 90.0 degrees, Min = -90.0 degrees
const BASE_BOUNDS: (f32, f32) = (90.0, -90.0);
/// Max = 110.0 degrees, Min = 0.0 degrees
const SHOULDER_BOUNDS: (f32, f32) = (110.0, 0.0);
/// Max = 0.0 degrees, Min = -110.0 degrees
const ELBOW_BOUNDS: (f32, f32) = (0.0, -110.0);
/// Max = 90.0 degrees, Min = -90.0 degrees
const HAND_BOUNDS: (f32, f32) = (90.0, -90.0);
static MOTION_TRACKER: MotionTracker = MotionTracker::new();
const DAC_CONVERSION: f32 = 4095.0; // 12 bit @ 3.3V

bind_interrupts!(struct Irqs {
    USART2 => embassy_stm32::usart::InterruptHandler<USART2>;
    DMA1_CHANNEL1 => embassy_stm32::dma::InterruptHandler<DMA1_CH1>;
    DMA1_CHANNEL2 => embassy_stm32::dma::InterruptHandler<DMA1_CH2>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();
    config.rcc.mux.adc12sel = embassy_stm32::rcc::mux::Adcsel::SYS; // select an ADC clock

    let p = embassy_stm32::init(config);

    let shoulder_step_pin = Output::new(
        p.PC9,
        embassy_stm32::gpio::Level::Low,
        embassy_stm32::gpio::Speed::VeryHigh,
    );
    let shoulder_dir_pin = Output::new(
        p.PC8,
        embassy_stm32::gpio::Level::Low,
        embassy_stm32::gpio::Speed::VeryHigh,
    );
    let shoulder_en_pin = Output::new(
        p.PB8,
        embassy_stm32::gpio::Level::Low,
        embassy_stm32::gpio::Speed::Medium,
    );
    let shoulder_adc = Adc::new(p.ADC1, AdcConfig::default());

    spawner.spawn(update_shoulder_angle(shoulder_adc, p.PA0).unwrap());
    spawner.spawn(move_shoulder_stepper(shoulder_step_pin, shoulder_dir_pin).unwrap());

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
            let command = match parse(msg) {
                Ok(c) => c,
                Err(e) => {
                    let mut buf = [0u8; 128];
                    let s =
                        format_no_std::show(&mut buf, format_args!("{{\"error\": \"{}\"}}\n", e))
                            .unwrap();

                    uart.write(s.as_bytes()).await.unwrap();

                    continue;
                }
            };

            match command {
                Command::G4 { ms } => Timer::after_millis(ms).await,
                Command::G92 { x, y, z } => {
                    let (current_x, current_y, current_z) = forward(
                        MOTION_TRACKER.get_base_angle(),
                        MOTION_TRACKER.get_shoulder_angle(),
                        MOTION_TRACKER.get_elbow_angle(),
                    );
                    let (base, shoulder, elbow, hand) = inverse(
                        x.unwrap_or(current_x),
                        y.unwrap_or(current_y),
                        z.unwrap_or(current_z),
                    );
                    let (base_delay, shoulder_delay, elbow_delay, hand_delay) =
                        delay(5, base, shoulder, elbow, hand);

                    if !in_base_bounds(base)
                        || !in_shoulder_bounds(shoulder)
                        || !in_elbow_bounds(elbow)
                        || !in_hand_bounds(hand)
                    {
                        uart.write(b"{\"error\": \"desired position out of bounds\"}")
                            .await
                            .unwrap();

                        continue;
                    }

                    MOTION_TRACKER.set_target(MotionTarget::Base, base, base_delay);
                    MOTION_TRACKER.set_target(MotionTarget::Shoulder, shoulder, shoulder_delay);
                    MOTION_TRACKER.set_target(MotionTarget::Elbow, elbow, elbow_delay);
                    MOTION_TRACKER.set_target(MotionTarget::Hand, hand, hand_delay);
                }
                Command::M02 => {
                    uart.write(b"{\"queue\": \"quit\"}").await.unwrap();

                    continue;
                }
                _ => {}
            }

            uart.write(b"{\"queue\": \"continue\"}").await.unwrap();
        } else {
            uart.write(b"{\"error\": \"invalid UTF-8 sequence\"}\n")
                .await
                .unwrap();

            continue;
        }
    }
}

#[embassy_executor::task]
async fn update_shoulder_angle(
    mut shoulder_adc: Adc<'static, ADC1>,
    mut shoulder_peri: Peri<'static, PA0>,
) {
    let mut previous = 0.0;
    let mut continuous = 0.0;
    let mut first = true;

    loop {
        let raw = shoulder_adc.blocking_read(&mut shoulder_peri, SampleTime::CYCLES640_5);

        let angle = raw as f32 * 360.0 / DAC_CONVERSION;

        if first {
            previous = angle;
            continuous = angle;
            first = false;
        } else {
            let mut delta = angle - previous;

            if delta > 180.0 {
                delta -= 360.0;
            } else if delta < -180.0 {
                delta += 360.0;
            }

            continuous += delta;
            previous = angle;
        }

        let output_angle = continuous / 6.0;

        MOTION_TRACKER.set_shoulder_angle(output_angle);

        info!("Output angle: {}", &MOTION_TRACKER.get_shoulder_angle());

        yield_now().await;
    }
}

#[embassy_executor::task]
async fn move_base_stepper(mut step_pin: Output<'static>, mut dir_pin: Output<'static>) {
    loop {
        let (target, delay) = MOTION_TRACKER.get_target(MotionTarget::Base);

        if target < 0.0 {
            dir_pin.set_high();
        } else {
            dir_pin.set_low();
        }

        let steps = target * (BASE_STEPS_PER_REVOLUTION as f32 / 360.0);

        for _ in 0..(steps.abs() as usize) {
            step_pin.set_high();
            Timer::after_millis(delay).await;
            step_pin.set_low();
            Timer::after_millis(delay).await;
        }

        MOTION_TRACKER.set_base_angle(target);

        yield_now().await;
    }
}

#[embassy_executor::task]
async fn move_shoulder_stepper(mut step_pin: Output<'static>, mut dir_pin: Output<'static>) {
    loop {
        let (target, delay) = MOTION_TRACKER.get_target(MotionTarget::Shoulder);
        let current = MOTION_TRACKER.get_shoulder_angle();
        let error = shortest_angle(target, current);

        if error.abs() < 0.2 {
            yield_now().await;
            continue;
        }

        if error < 0.0 {
            dir_pin.set_high();
        } else {
            dir_pin.set_low();
        }

        step_pin.set_high();
        Timer::after_millis(delay).await;
        step_pin.set_low();
        Timer::after_millis(delay).await;
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

fn shortest_angle(target: f32, current: f32) -> f32 {
    let mut error = target - current;

    while error > 180.0 {
        error -= 360.0;
    }

    while error < -180.0 {
        error += 360.0;
    }

    error
}

fn in_base_bounds(angle: f32) -> bool {
    angle <= BASE_BOUNDS.0 && angle >= BASE_BOUNDS.1
}

fn in_shoulder_bounds(angle: f32) -> bool {
    angle <= SHOULDER_BOUNDS.0 && angle >= SHOULDER_BOUNDS.1
}

fn in_elbow_bounds(angle: f32) -> bool {
    angle <= ELBOW_BOUNDS.0 && angle >= ELBOW_BOUNDS.1
}

fn in_hand_bounds(angle: f32) -> bool {
    angle <= HAND_BOUNDS.0 && angle >= HAND_BOUNDS.1
}
