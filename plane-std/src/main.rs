use anyhow::{bail, ensure, Result};
use bytemuck::{self, bytes_of_mut, Zeroable};
use core::str;
use esp_idf_hal::{
    i2c::{I2cConfig, I2cDriver, I2C1},
    ledc::{LedcDriver, Resolution},
    prelude::*,
};
use esp_idf_svc::{
    eventloop::EspSystemEventLoop,
    hal::{
        ledc::{config::TimerConfig, LedcTimerDriver},
        peripheral,
        peripherals::Peripherals,
    },
    wifi::{AuthMethod, BlockingWifi, ClientConfiguration, Configuration, EspWifi},
};
use esp_idf_sys::{nvs_flash_init, ESP_OK};
use log::info;
use lsm6dso;
use protocol;
use std::{
    net::{SocketAddr, UdpSocket},
    str::FromStr,
    sync::Arc,
    time::{Duration, SystemTime},
};

pub fn wifi(
    ssid: &str,
    pass: &str,
    modem: impl peripheral::Peripheral<P = esp_idf_svc::hal::modem::Modem> + 'static,
    sysloop: EspSystemEventLoop,
) -> Result<Box<EspWifi<'static>>> {
    let mut auth_method = AuthMethod::WPA2Personal;

    if ssid.is_empty() {
        bail!("Missing WiFi name")
    }
    if pass.is_empty() {
        auth_method = AuthMethod::None;
        info!("Wifi password is empty");
    }
    let mut esp_wifi = EspWifi::new(modem, sysloop.clone(), None)?;
    let mut wifi = BlockingWifi::wrap(&mut esp_wifi, sysloop)?;

    wifi.set_configuration(&Configuration::Client(ClientConfiguration::default()))?;
    info!("Starting wifi...");

    wifi.start()?;
    info!("Scanning...");
    let ap_infos = wifi.scan()?;
    let ours = ap_infos.into_iter().find(|a| a.ssid == ssid);
    let channel = if let Some(ours) = ours {
        info!(
            "Found configured access point {} on channel {}",
            ssid, ours.channel
        );
        Some(ours.channel)
    } else {
        info!(
            "Configured access point {} not found during scanning, will go with unknown channel",
            ssid
        );
        None
    };
    wifi.set_configuration(&Configuration::Client(ClientConfiguration {
        ssid: ssid.try_into().unwrap(),
        password: pass.try_into().unwrap(),
        channel,
        auth_method,
        ..Default::default()
    }))?;

    info!("Connecting wifi...");
    wifi.connect()?;

    info!("Waiting for DHCP lease...");
    wifi.wait_netif_up()?;

    let ip_info = wifi.wifi().sta_netif().get_ip_info()?;

    info!("Wifi DHCP info: {:?}", ip_info);

    Ok(Box::new(esp_wifi))
}

#[toml_cfg::toml_config]
pub struct Config {
    #[default("Wokwi-GUEST")]
    wifi_ssid: &'static str,
    #[default("")]
    wifi_psk: &'static str,
}

fn command_thread(
    socket: Arc<UdpSocket>,
    mut pitch: LedcDriver,
    mut roll: LedcDriver,
    mut throttle: LedcDriver,
) -> Result<()> {
    println!("command thread started!");
    socket
        .set_read_timeout(Some(Duration::from_millis(200)))
        .unwrap();
    let min_duty = pitch.get_max_duty() * 1 / 20;
    let max_duty = pitch.get_max_duty() * 2 / 20;
    println!("{} {}", min_duty, max_duty);

    loop {
        let mut command = protocol::Control::zeroed();
        let buf = bytes_of_mut(&mut command);
        let recv_value = socket.recv_from(buf);
        let ok = match recv_value {
            Ok((amt, _)) => amt == buf.len(),
            _ => {
                command = protocol::Control {
                    pitch: 0.5,
                    roll: 0.5,
                    throttle: 0.0,
                };
                true
            }
        };
        if ok {
            let pitch_value =
                (((command.pitch + 1.0) / 2.0) * (max_duty - min_duty) as f32) as u32 + min_duty;
            let roll_value =
                (((command.roll + 1.0) / 2.0) * (max_duty - min_duty) as f32) as u32 + min_duty;
            let throttle_value =
                (command.throttle * (max_duty - min_duty) as f32) as u32 + min_duty;

            pitch.set_duty(pitch_value).unwrap_or_default();
            roll.set_duty(roll_value).unwrap_or_default();
            throttle.set_duty(throttle_value).unwrap_or_default();
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
            .frequency(50.Hz())
            .resolution(Resolution::Bits14),
    )
    .unwrap();

    let driver_pitch = LedcDriver::new(
        peripherals.ledc.channel4,
        &timer_driver,
        peripherals.pins.gpio2,
    )
    .unwrap();
    let driver_roll = LedcDriver::new(
        peripherals.ledc.channel5,
        &timer_driver,
        peripherals.pins.gpio16,
    )
    .unwrap();

    let driver_throttle = LedcDriver::new(
        peripherals.ledc.channel6,
        &timer_driver,
        peripherals.pins.gpio1,
    )
    .unwrap();
    let sda = peripherals.pins.gpio15;
    let scl = peripherals.pins.gpio7;

    let config = I2cConfig::new().baudrate(100.kHz().into());
    let i2cdriver = I2cDriver::new(peripherals.i2c0, sda, scl, &config).unwrap();

    let mut driver = lsm6dso::Lsm6dso::new(i2cdriver, 0x6a).unwrap();
    driver.set_low_power_mode(false).unwrap();
    driver
        .set_accelerometer_output(lsm6dso::AccelerometerOutput::Rate208)
        .unwrap();
    driver.set_low_power_mode(false).unwrap();
    driver
        .set_gyroscope_output(lsm6dso::GyroscopeOutput::Rate208)
        .unwrap();

    for _ in 0..1000 {
        println!(" temp {}", driver.read_temperature().unwrap());
        println!("{:?}", driver.read_all());
    }

    let sysloop = EspSystemEventLoop::take()?;

    // The constant `CONFIG` is auto-generated by `toml_config`.
    let app_config = CONFIG;

    unsafe {
        ensure!(nvs_flash_init() == ESP_OK);
    };

    // Connect to the Wi-Fi network
    let _wifi = wifi(
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
        xclk_freq_hz: 20000000,
        ledc_timer: esp_idf_sys::ledc_timer_t_LEDC_TIMER_0,
        ledc_channel: esp_idf_sys::ledc_channel_t_LEDC_CHANNEL_0,
        pixel_format: esp_idf_sys::camera::pixformat_t_PIXFORMAT_JPEG,
        frame_size: esp_idf_sys::camera::framesize_t_FRAMESIZE_SVGA,
        jpeg_quality: 10,
        fb_count: 2,
        fb_location: esp_idf_sys::camera::camera_fb_location_t_CAMERA_FB_IN_PSRAM,
        grab_mode: esp_idf_sys::camera::camera_grab_mode_t_CAMERA_GRAB_WHEN_EMPTY,
    };

    {
        let socket = socket.clone();
        std::thread::spawn(move || {
            command_thread(socket, driver_pitch, driver_roll, driver_throttle)
        });
    }

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

    loop {
        let t1 = SystemTime::now();
        let fb = unsafe { esp_idf_sys::camera::esp_camera_fb_get() };
        let data = unsafe { std::slice::from_raw_parts((*fb).buf, (*fb).len) };

        let t2 = SystemTime::now();

        while let Err(x) = socket.send_to(data, server) {
            println!("Failed to send: {}", x);
            continue;
        }

        let t3 = SystemTime::now();

        avg_capture += t2.duration_since(t1).unwrap_or_default().as_secs_f32();
        avg_send += t3.duration_since(t2).unwrap_or_default().as_secs_f32();
        if message_id % 100 == 0 {
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
