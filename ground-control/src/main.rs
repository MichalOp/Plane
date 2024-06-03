use std::{
    f32::consts::PI,
    mem::size_of,
    net::SocketAddr,
    path::Path,
    sync::{Arc, Mutex},
    thread,
    time::Duration,
};

use anyhow;
use bytemuck::{bytes_of, bytes_of_mut, Zeroable};
use eframe::egui;
use egui::{Color32, TextureHandle};
use ndarray::Array3;
use protocol::{Control, Telemetry};
use stick::{Controller, Event, Listener};
use tokio::net::UdpSocket;
use tokio::time;
use tracing::{info, warn};
use turbojpeg;

use chrono::{DateTime, Local};

use video_rs::encode::{Encoder, Settings};
use video_rs::time::Time;

struct DisplayData {
    image_data: egui::ImageData,
    telemetry: Telemetry,
    voltage_avg: f32,
    recording: bool,
}

async fn video_server(
    socket: Arc<UdpSocket>,
    buffer: Arc<Mutex<DisplayData>>,
    plane_ip: Arc<Mutex<Option<SocketAddr>>>,
) -> anyhow::Result<()> {
    // Receives a single datagram message on the socket. If `buf` is too small to hold
    // the message, it will be cut off.

    let mut encoder: Option<Encoder> = None;
    let mut position = Time::zero();
    let duration: Time = Time::from_nth_of_a_second(66);

    loop {
        let mut buf = [0; 65536];
        let (amt, src) = socket.recv_from(&mut buf).await.unwrap();
        *plane_ip.lock().unwrap() = Some(src);
        if amt > 0 {
            // info!("Parsing! {}", payload.len());

            let mut telemetry = Telemetry::zeroed();

            let tele_buf = bytes_of_mut(&mut telemetry);
            tele_buf.clone_from_slice(&buf[..size_of::<Telemetry>()]);

            let img: Result<image::RgbImage, _> =
                turbojpeg::decompress_image(&buf[size_of::<Telemetry>()..]);
            if img.is_err() {
                warn!("eh {}", img.err().unwrap());
                continue;
            }
            let img = img.unwrap();
            let mut locked = buffer.lock().unwrap();

            *locked = DisplayData {
                image_data: egui::ImageData::Color(
                    egui::ColorImage {
                        size: [img.width() as usize, img.height() as usize],
                        pixels: img
                            .pixels()
                            .map(|x| Color32::from_rgb(x.0[0], x.0[1], x.0[2]))
                            .collect(),
                    }
                    .into(),
                ),
                telemetry,
                recording: locked.recording,
                voltage_avg: locked.voltage_avg,
            };
            let recording = locked.recording;
            drop(locked);

            if encoder.is_none() && recording {
                let settings = Settings::preset_h264_yuv420p(
                    img.width() as usize,
                    img.height() as usize,
                    false,
                );

                let current_local: DateTime<Local> = Local::now();
                let custom_format = current_local.format("%Y-%m-%d-%H-%M-%S");
                let filename = format!("/home/michal/flightrecordings/{}.mp4", custom_format);
                encoder = Some(
                    Encoder::new(Path::new(&filename), settings).expect("failed to create encoder"),
                );
                position = Time::zero();
            }
            if encoder.is_some() {
                let (width, height) = img.dimensions();
                // Create a vector to hold the data
                let mut data = Vec::with_capacity((width * height * 3) as usize);

                // Copy the pixel data into the vector
                for pixel in img.pixels() {
                    data.extend_from_slice(&pixel.0);
                }

                let array = Array3::from_shape_vec((height as usize, width as usize, 3), data)
                    .expect("failed to convert");
                encoder
                    .as_mut()
                    .unwrap()
                    .encode(&array, position)
                    .expect("failed to encode frame");

                position = position.aligned_with(duration).add();

                if !recording {
                    encoder
                        .as_mut()
                        .unwrap()
                        .finish()
                        .expect("failed to finish recording");
                    encoder = None;
                }
            }
        }
    }
}

fn control_modifier(x: f32) -> f32 {
    (x.powi(2) * x.signum() * 0.8 + x * 0.2) * PI * 1.5
}

async fn track_events(
    controller: Controller,
    control: Arc<Mutex<Control>>,
    display_data: Arc<Mutex<DisplayData>>,
) {
    let mut controller = controller;
    loop {
        let event = (&mut controller).await;
        match event {
            Event::Trigger(x) => {
                if x {
                    let mut d = display_data.lock().unwrap();
                    d.recording = !d.recording;
                }
            }
            Event::JoyX(x) => {
                control.lock().unwrap().roll = control_modifier(x as f32);
            }
            Event::JoyY(y) => {
                control.lock().unwrap().pitch = control_modifier(y as f32);
            }
            Event::CamZ(z) => {
                // info!("yaw {}", y);
                let normalized_val = ((z + 0.9843740462674724) / (1.0 - 0.9843740462674724)) as f32;
                control.lock().unwrap().yaw = -control_modifier(normalized_val);
            }
            Event::Throttle(t) => {
                info!("throttle {}", t);
                control.lock().unwrap().throttle = (1.0 - t) as f32;
            }
            _ => {}
        }
    }
}

async fn send_command(
    socket: Arc<UdpSocket>,
    control: Arc<Mutex<Control>>,
    plane_ip: Arc<Mutex<Option<SocketAddr>>>,
) -> anyhow::Result<()> {
    let mut interval = time::interval(Duration::from_millis(10));
    loop {
        interval.tick().await;
        let ip = plane_ip.lock().unwrap().clone();
        if let Some(ip) = ip {
            let bytes_to_send: Vec<u8> = {
                let lock = control.lock().unwrap();
                bytes_of(&*lock).to_vec()
            };
            socket.send_to(&bytes_to_send, ip).await?;
        }
    }
}

async fn tokio_main(buffer: Arc<Mutex<DisplayData>>) -> anyhow::Result<()> {
    let control = Arc::new(Mutex::new(Control::zeroed()));
    let plane_ip: Arc<Mutex<Option<SocketAddr>>> = Arc::new(Mutex::new(None));
    let socket = Arc::new(UdpSocket::bind("0.0.0.0:12892").await.unwrap());

    {
        let control = control.clone();
        let plane_ip = plane_ip.clone();
        let socket = socket.clone();
        tokio::task::spawn(async move { send_command(socket, control, plane_ip).await });
    }
    let buffer_cloned = buffer.clone();
    tokio::task::spawn(async move { video_server(socket, buffer_cloned, plane_ip).await });
    let mut listener = Listener::default();
    loop {
        let controller = (&mut listener).await;
        println!("{:?}", controller);
        if controller.name() == "Thrustmaster T.16000M" {
            let control = control.clone();
            let buffer = buffer.clone();
            tokio::task::spawn(async move { track_events(controller, control, buffer).await });
        }
    }
}

fn make_debug_image() -> egui::ImageData {
    let size = [800, 600];
    let mut pixels = Vec::new();
    for i in 0..size[0] {
        for j in 0..size[1] {
            let color = Color32::from_rgb(i as u8, j as u8, 0);
            pixels.push(color);
        }
    }
    let image = egui::ColorImage { size, pixels };
    egui::ImageData::Color(image.into())
}

fn main() {
    env_logger::init();
    tracing::subscriber::set_global_default(tracing_subscriber::FmtSubscriber::new())
        .expect("setting tracing default failed");
    let buffer = Arc::new(Mutex::new(DisplayData {
        image_data: make_debug_image(),
        telemetry: Telemetry::zeroed(),
        voltage_avg: 0.0,
        recording: false,
    }));
    let buffer_cloned = buffer.clone();

    let tokio_thread = thread::spawn(move || {
        tokio::runtime::Builder::new_multi_thread()
            .enable_all()
            .build()
            .unwrap()
            .block_on(async move { tokio_main(buffer_cloned).await.unwrap() })
    });

    let native_options = eframe::NativeOptions::default();
    eframe::run_native(
        "My egui App",
        native_options,
        Box::new(|cc| Box::new(MyEguiApp::new(cc, buffer))),
    )
    .unwrap();
    tokio_thread.join().unwrap();
}

struct MyEguiApp {
    display_info: Arc<Mutex<DisplayData>>,
    texture: TextureHandle,
    i: u32,
}

impl MyEguiApp {
    fn new(cc: &eframe::CreationContext<'_>, display_info: Arc<Mutex<DisplayData>>) -> Self {
        // Customize egui here with cc.egui_ctx.set_fonts and cc.egui_ctx.set_visuals.
        // Restore app state using cc.storage (requires the "persistence" feature).
        // Use the cc.gl (a glow::Context) to create graphics shaders and buffers that you can use
        // for e.g. egui::PaintCallback.
        let texture = cc
            .egui_ctx
            .load_texture("display", make_debug_image(), Default::default());

        Self {
            display_info,
            texture,
            i: 0,
        }
    }
}

impl eframe::App for MyEguiApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::CentralPanel::default().show(ctx, |ui| {
            let mut locked_display_info = self.display_info.lock().unwrap();
            self.texture
                .set(locked_display_info.image_data.clone(), Default::default());
            let v = locked_display_info.telemetry.voltage / 10;
            let voltage = v as f32 / 570.0 * 0.66 * ((150.0 + 680.0) / (150.0));
            locked_display_info.voltage_avg =
                locked_display_info.voltage_avg * 0.95 + voltage * 0.05;
            ui.heading(format!(
                "{:.02}V            {}",
                locked_display_info.voltage_avg,
                if locked_display_info.recording {
                    "RECORDING"
                } else {
                    ""
                }
            ));
            ui.centered_and_justified(|ui| {
                ui.add(
                    egui::Image::new(&self.texture)
                        .maintain_aspect_ratio(true)
                        .fit_to_exact_size(ui.available_size()),
                )
            });
            self.i += 1;
            ctx.request_repaint();
        });
    }
}
