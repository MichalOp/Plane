#![feature(generic_arg_infer)]

use anyhow::{bail, ensure, Result};
use biquad::*;
use bytemuck::{self, bytes_of, bytes_of_mut, Zeroable};
use core::str;
use esp_idf_hal::{
    adc::{self, AdcChannelDriver, AdcDriver, Atten11dB, ADC1},
    i2c::{I2cConfig, I2cDriver},
    ledc::{LedcDriver, Resolution},
    prelude::*,
};
use esp_idf_svc::{
    eventloop::EspSystemEventLoop,
    hal::{
        ledc::{config::TimerConfig, LedcTimerDriver},
        peripherals::Peripherals,
    },
    timer::EspTimerService,
};
use esp_idf_sys::{esp_wifi_get_max_tx_power, ets_get_cpu_frequency, nvs_flash_init, ESP_OK};
use lsm6dso;
use pid::Pid;
use protocol::{self, Control, Telemetry};
use std::{
    net::{SocketAddr, UdpSocket},
    str::FromStr,
    sync::{atomic::AtomicU16, Arc, Mutex},
    thread::sleep,
    time::{Duration, SystemTime, SystemTimeError},
};

mod wifi;
use wifi::wifi;

use itertools::izip;

#[toml_cfg::toml_config]
pub struct Config {
    #[default("Wokwi-GUEST")]
    wifi_ssid: &'static str,
    #[default("")]
    wifi_psk: &'static str,
}

trait Controller {
    fn set_rates(&mut self, command: Control, voltage: u16, i: i32);
}

struct PlaneController<'a> {
    pitch_driver: LedcDriver<'a>,
    roll_driver: LedcDriver<'a>,
    throttle_driver: LedcDriver<'a>,
    min_duty: u32,
    max_duty: u32,
}

impl Controller for PlaneController<'_> {
    fn set_rates(&mut self, command: Control, voltage: u16, _i: i32) {
        let min_duty = self.min_duty;
        let max_duty = self.max_duty;
        let pitch_value =
            (((command.pitch + 1.0) / 2.0) * (max_duty - min_duty) as f32) as u32 + min_duty;
        let roll_value =
            (((command.roll + 1.0) / 2.0) * (max_duty - min_duty) as f32) as u32 + min_duty;
        let throttle_value = (command.throttle * (max_duty - min_duty) as f32) as u32 + min_duty;

        self.pitch_driver.set_duty(pitch_value).unwrap_or_default();
        self.roll_driver.set_duty(roll_value).unwrap_or_default();
        self.throttle_driver
            .set_duty(throttle_value)
            .unwrap_or_default();
    }
}

struct QuadController<'a> {
    left_front: LedcDriver<'a>,
    left_back: LedcDriver<'a>,
    right_front: LedcDriver<'a>,
    right_back: LedcDriver<'a>,
    imu: lsm6dso::Lsm6dso<I2cDriver<'a>>,
    rates_zero: [f32; 3],
    pids: [pid::Pid<f32>; 3],
    filters: [DirectForm2Transposed<f32>; 3],
}

impl Controller for QuadController<'_> {
    fn set_rates(&mut self, command: Control, voltage: u16, i: i32) {
        let rates: [f32; 3] = self.imu.read_gyro().unwrap().into();
        let command_rates: [f32; 3] = [command.pitch, command.roll, command.yaw];
        let control_outputs: [f32; 3] = izip!(
            self.pids.iter_mut(),
            rates,
            self.rates_zero,
            self.filters.iter_mut(),
            command_rates
        )
        .map(|(p, rate, rate_zero, filter, command_rate)| {
            let rate = filter.run(rate - rate_zero);
            p.setpoint(command_rate);
            let control = p.next_control_output(rate);
            control.output
        })
        .collect::<Vec<_>>()
        .try_into()
        .unwrap();

        let [pitch, roll, yaw] = control_outputs;

        // if i % 100 == 0 {
        //     println!(
        //         "{:.3} {:.3} {:.3}:{:.3} {:.3} {:.3}",
        //         command.pitch, command.roll, command.yaw, pitch, roll, yaw
        //     );
        // }

        let yaw_flip = -1.0;
        let left_front = pitch + roll + yaw_flip * yaw;
        let left_back = -pitch + roll - yaw_flip * yaw;
        let right_front = pitch - roll - yaw_flip * yaw;
        let right_back = -pitch - roll + yaw_flip * yaw;

        let control_max = [left_front, right_front, left_back, right_back]
            .iter()
            .map(|x| x.abs())
            .fold(f32::NEG_INFINITY, |a, b| a.max(b));

        let scaled_throttle = command.throttle * 0.9;
        let normalize = (control_max + 0.0001)
            .min(scaled_throttle + 0.0001)
            .min(1.0 - scaled_throttle + 0.0001)
            / (control_max + 0.0001);
        // println!("normalize {}", normalize);

        let voltage = voltage as f32 / 20816.4; // 5700.0 * 0.66 * ((150.0 + 680.0) / (150.0));
        let voltage_scaling = 3.5 / voltage.max(2.5);
        let scaled_throttle = scaled_throttle * voltage_scaling;
        let normalize = normalize * voltage_scaling;

        let left_front = scaled_throttle + left_front * normalize;
        let left_back = scaled_throttle + left_back * normalize;
        let right_back = scaled_throttle + right_back * normalize;
        let right_front = scaled_throttle + right_front * normalize;

        self.left_front
            .set_duty((left_front.clamp(0.0, 1.0) * 255.0).round() as u32)
            .unwrap();
        self.left_back
            .set_duty((left_back.clamp(0.0, 1.0) * 255.0).round() as u32)
            .unwrap();
        self.right_front
            .set_duty((right_front.clamp(0.0, 1.0) * 255.0).round() as u32)
            .unwrap();
        self.right_back
            .set_duty((right_back.clamp(0.0, 1.0) * 255.0).round() as u32)
            .unwrap();

        // println!(
        //     "{:.3} {:.3} {:.3} {:.3}",
        //     left_front, right_front, left_back, right_back
        // );
    }
}

fn build_quad_controller<'a>(
    left_front: LedcDriver<'a>,
    left_back: LedcDriver<'a>,
    right_front: LedcDriver<'a>,
    right_back: LedcDriver<'a>,
    imu: lsm6dso::Lsm6dso<I2cDriver<'a>>,
) -> QuadController<'a> {
    sleep(Duration::from_secs_f32(1.0));
    let mut imu = imu;
    let mut rates_zero: [f32; 3] = [0.0; 3];
    let samples = 1000;
    for i in 0..samples {
        let rates: [f32; 3] = imu.read_gyro().unwrap().into();
        rates_zero[0] += rates[0];
        rates_zero[1] += rates[1];
        rates_zero[2] += rates[2];
    }
    rates_zero.iter_mut().for_each(|x| *x /= samples as f32);
    print!("zero rates {:?}", rates_zero);
    // Cutoff and sampling frequencies
    let f0 = 125.hz();
    let fs = 1.66.khz();

    // Create coefficients for the biquads
    let coeffs =
        Coefficients::<f32>::from_params(Type::LowPass, fs, f0, Q_BUTTERWORTH_F32).unwrap();
    let controller = QuadController {
        left_front,
        left_back,
        right_front,
        right_back,
        imu,
        rates_zero,
        filters: [DirectForm2Transposed::<f32>::new(coeffs); 3],
        pids: [
            *Pid::new(0.0, 1.0).p(4.0, 1.0).d(15.0, 1.0).i(0.1, 0.0),
            *Pid::new(0.0, 1.0).p(4.0, 1.0).d(15.0, 1.0).i(0.1, 0.0),
            *Pid::new(0.0, 1.0).p(3.0, 1.0).d(1.0, 1.0).i(0.1, 0.0),
        ],
    };
    controller
}
fn build_plane_controller<'a>(
    pitch: LedcDriver<'a>,
    roll: LedcDriver<'a>,
    throttle: LedcDriver<'a>,
) -> PlaneController<'a> {
    let min_duty = pitch.get_max_duty() * 1 / 20;
    let max_duty = pitch.get_max_duty() * 2 / 20;

    let controller = PlaneController {
        pitch_driver: pitch,
        roll_driver: roll,
        throttle_driver: throttle,
        min_duty,
        max_duty,
    };
    controller
}

fn command_thread(command: Arc<Mutex<protocol::Control>>, socket: Arc<UdpSocket>) -> Result<()> {
    println!("command thread started!");
    socket
        .set_read_timeout(Some(Duration::from_micros(2000)))
        .unwrap();

    let mut last_command_time = SystemTime::now();
    loop {
        let mut new_command = protocol::Control::zeroed();
        let buf = bytes_of_mut(&mut new_command);
        let recv_value = socket.recv_from(buf);
        let ok = match recv_value {
            Ok((amt, _)) => amt == buf.len(),
            _ => false,
        };
        let current_time = SystemTime::now();
        if ok {
            let mut locked = command.lock().unwrap();
            *locked = new_command;
            last_command_time = current_time;
        };

        if current_time
            .duration_since(last_command_time)
            .unwrap()
            .as_secs_f32()
            > 0.2
        {
            let mut locked = command.lock().unwrap();
            *locked = protocol::Control {
                pitch: 0.0,
                roll: 0.0,
                yaw: 0.0,
                throttle: 0.0,
            };
        }
    }
}

fn main() -> Result<()> {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("Hello, world!");
    let peripherals = Peripherals::take().unwrap();
    let timer_driver = LedcTimerDriver::new(
        peripherals.ledc.timer1,
        &TimerConfig::default()
            .frequency(100000.Hz())
            .resolution(Resolution::Bits8),
    )
    .unwrap();

    let mut adc = AdcDriver::new(
        peripherals.adc1,
        &adc::config::Config::new().calibration(false),
    )?;
    let mut adc_pin: esp_idf_hal::adc::AdcChannelDriver<{ adc::attenuation::DB_11 }, _> =
        AdcChannelDriver::new(peripherals.pins.gpio8)?;

    // let driver_pitch = LedcDriver::new(
    //     peripherals.ledc.channel4,
    //     &timer_driver,
    //     peripherals.pins.gpio2,
    // )
    // .unwrap();
    // let driver_roll = LedcDriver::new(
    //     peripherals.ledc.channel5,
    //     &timer_driver,
    //     peripherals.pins.gpio16,
    // )
    // .unwrap();
    // let driver_throttle = LedcDriver::new(
    //     peripherals.ledc.channel6,
    //     &timer_driver,
    //     peripherals.pins.gpio1,
    // )
    // .unwrap();
    let driver_left_front = LedcDriver::new(
        peripherals.ledc.channel4,
        &timer_driver,
        peripherals.pins.gpio18,
    )
    .unwrap();
    let driver_left_back = LedcDriver::new(
        peripherals.ledc.channel5,
        &timer_driver,
        peripherals.pins.gpio17,
    )
    .unwrap();
    let driver_right_back = LedcDriver::new(
        peripherals.ledc.channel6,
        &timer_driver,
        peripherals.pins.gpio6,
    )
    .unwrap();
    let driver_right_front = LedcDriver::new(
        peripherals.ledc.channel7,
        &timer_driver,
        peripherals.pins.gpio5,
    )
    .unwrap();

    // let plane_controller = build_plane_controller(driver_pitch, driver_roll, driver_throttle);

    let sda = peripherals.pins.gpio15;
    let scl = peripherals.pins.gpio7;

    let config = I2cConfig::new().baudrate(1000.kHz().into());
    let i2cdriver = I2cDriver::new(peripherals.i2c0, sda, scl, &config).unwrap();

    let mut driver = lsm6dso::Lsm6dso::new(i2cdriver, 0x6a).unwrap();
    driver.set_low_power_mode(false).unwrap();
    driver
        .set_accelerometer_output(lsm6dso::AccelerometerOutput::Rate1_66k)
        .unwrap();
    driver
        .set_gyroscope_output(lsm6dso::GyroscopeOutput::Rate1_66k)
        .unwrap();
    driver
        .set_gyroscope_scale(lsm6dso::GyroscopeFullScale::Dps1000)
        .unwrap();

    let sysloop = EspSystemEventLoop::take()?;

    // The constant `CONFIG` is auto-generated by `toml_config`.
    let app_config = CONFIG;

    unsafe {
        ensure!(nvs_flash_init() == ESP_OK);
    };

    // Connect to the Wi-Fi network
    let mut wifi = wifi(
        app_config.wifi_ssid,
        app_config.wifi_psk,
        peripherals.modem,
        sysloop,
    )?;

    let socket = Arc::new(UdpSocket::bind("0.0.0.0:12987")?);

    let camera_config = esp_idf_sys::camera::camera_config_t {
        pin_pwdn: -1,
        pin_reset: -1,
        pin_xclk: 9,
        sccb_i2c_port: -1,
        __bindgen_anon_1: esp_idf_sys::camera::camera_config_t__bindgen_ty_1 { pin_sccb_sda: 39 },
        __bindgen_anon_2: esp_idf_sys::camera::camera_config_t__bindgen_ty_2 { pin_sscb_scl: 38 },
        pin_d7: 40,
        pin_d6: 10,
        pin_d5: 11,
        pin_d4: 13,
        pin_d3: 21,
        pin_d2: 48,
        pin_d1: 47,
        pin_d0: 14,
        pin_vsync: 42,
        pin_href: 41,
        pin_pclk: 12,
        xclk_freq_hz: 24000000,
        ledc_timer: esp_idf_sys::ledc_timer_t_LEDC_TIMER_0,
        ledc_channel: esp_idf_sys::ledc_channel_t_LEDC_CHANNEL_0,
        pixel_format: esp_idf_sys::camera::pixformat_t_PIXFORMAT_JPEG,
        frame_size: esp_idf_sys::camera::framesize_t_FRAMESIZE_CIF,
        jpeg_quality: 10,
        fb_count: 2,
        fb_location: esp_idf_sys::camera::camera_fb_location_t_CAMERA_FB_IN_PSRAM,
        grab_mode: esp_idf_sys::camera::camera_grab_mode_t_CAMERA_GRAB_WHEN_EMPTY,
    };

    let voltage: Arc<AtomicU16> = Arc::new(u16::MAX.into());
    let command: Arc<Mutex<protocol::Control>> = Mutex::new(protocol::Control::zeroed()).into();

    {
        let command = command.clone();
        let socket = socket.clone();
        std::thread::spawn(move || command_thread(command, socket));
    }
    let timer_service = EspTimerService::new().unwrap();
    let mut counter = 0;
    let start = SystemTime::now();

    let timer = {
        let mut controller = build_quad_controller(
            driver_left_front,
            driver_left_back,
            driver_right_front,
            driver_right_back,
            driver,
        );
        let command = command.clone();
        let voltage = voltage.clone();
        unsafe {
            timer_service
                .timer_nonstatic(move || {
                    counter += 1;
                    controller.set_rates(
                        command.lock().unwrap().clone(),
                        voltage.load(std::sync::atomic::Ordering::Relaxed),
                        counter,
                    );
                    if counter % 1660 == 0 {
                        println!(
                            "{}",
                            SystemTime::now()
                                .duration_since(start)
                                .unwrap()
                                .as_secs_f32()
                        );
                    }
                })
                .unwrap()
        }
    };

    timer.every(Duration::from_secs_f32(1.0 / 1660.0)).unwrap();

    let server = SocketAddr::from_str("192.168.1.197:12892")?;
    println!("server {:?}", server);
    let mut message_id = 0;
    unsafe {
        if esp_idf_sys::camera::esp_camera_init(&camera_config) != 0 {
            bail!("camera init failed!");
        }
        println!("camera ready!");
    }

    let mut tprev = SystemTime::now();
    let mut avg_capture = 0.0;
    let mut avg_send = 0.0;
    let mut signal_strength = 0;

    let driver = wifi.driver_mut();
    loop {
        let t1 = SystemTime::now();

        let fb = unsafe { esp_idf_sys::camera::esp_camera_fb_get() };
        let data = unsafe { std::slice::from_raw_parts((*fb).buf, (*fb).len) };

        let t2 = SystemTime::now();

        let mut v = 0;
        for _ in 0..10 {
            v += adc.read(&mut adc_pin).unwrap_or_default();
        }

        let v_c = voltage.load(std::sync::atomic::Ordering::Relaxed);
        let v_c = ((v_c as u32 * 19 + v as u32) / 20) as u16;
        // let tadc = SystemTime::now();
        // println!("ADC time: {}", tadc.duration_since(t2).unwrap().as_micros());
        let tele = Telemetry {
            voltage: v_c,
            signal_strength,
        };

        voltage.store(v_c, std::sync::atomic::Ordering::Relaxed);

        let data_vec: Vec<u8> = bytes_of(&tele)
            .iter()
            .chain(data.iter().to_owned())
            .map(|x| *x)
            .collect();

        while let Err(x) = socket.send_to(&data_vec, server) {
            println!("Failed to send: {}", x);
            continue;
        }

        let t3 = SystemTime::now();

        avg_capture += t2.duration_since(t1).unwrap_or_default().as_secs_f32();
        avg_send += t3.duration_since(t2).unwrap_or_default().as_secs_f32();
        if message_id % 100 == 0 {
            signal_strength = driver.get_ap_info()?.signal_strength as i16;
            let tnow = SystemTime::now();
            println!(
                "id: {} capture: {} send: {} FPS: {}",
                message_id,
                avg_capture / 100.0 * 1000.0,
                avg_send / 100.0 * 1000.0,
                100.0 / tnow.duration_since(tprev).unwrap_or_default().as_secs_f64(),
            );
            tprev = tnow;
            avg_capture = 0.0;
            avg_send = 0.0
        }
        message_id += 1;
        unsafe {
            esp_idf_sys::camera::esp_camera_fb_return(fb);
        }
    }
}
