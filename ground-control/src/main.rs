use std::{
    mem::size_of,
    net::SocketAddr,
    sync::{Arc, Mutex},
    thread,
    time::Duration,
};

use anyhow;
use bytemuck::{bytes_of, bytes_of_mut, Zeroable};
use eframe::egui;
use egui::{Color32, TextureHandle};
use protocol::{Control, Telemetry};
use stick::{Controller, Event, Listener};
use tokio::net::UdpSocket;
use tokio::time;
use tracing::{info, warn};
use turbojpeg;

struct DisplayData {
    image_data: egui::ImageData,
    telemetry: Telemetry,
    voltage_avg: f32,
}

async fn video_server(
    socket: Arc<UdpSocket>,
    buffer: Arc<Mutex<DisplayData>>,
    plane_ip: Arc<Mutex<Option<SocketAddr>>>,
) -> anyhow::Result<()> {
    // Receives a single datagram message on the socket. If `buf` is too small to hold
    // the message, it will be cut off.
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
            // let img = resize(
            //     &img,
            //     img.width() * 2,
            //     img.height() * 2,
            //     image::imageops::FilterType::Nearest,
            // );
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
                voltage_avg: locked.voltage_avg,
            }
        }
    }
}

async fn track_events(controller: Controller, control: Arc<Mutex<Control>>) {
    let mut controller = controller;
    loop {
        let event = (&mut controller).await;
        match event {
            Event::JoyX(x) => {
                // info!("roll {}", x);
                control.lock().unwrap().roll = x as f32 * 5.0;
            }
            Event::JoyY(y) => {
                // info!("pitch {}", y);
                control.lock().unwrap().pitch = y as f32 * 5.0;
            }
            Event::CamZ(z) => {
                // info!("yaw {}", y);
                let normalized_val = ((z + 0.9843740462674724) / (1.0 - 0.9843740462674724)) as f32;
                control.lock().unwrap().yaw = normalized_val * -2.0;
            }
            Event::Throttle(t) => {
                // info!("throttle {}", t);
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
    tokio::task::spawn(async move { video_server(socket, buffer, plane_ip).await });
    let mut listener = Listener::default();
    loop {
        let controller = (&mut listener).await;
        println!("{:?}", controller);
        if controller.name() == "Thrustmaster T.16000M" {
            let control = control.clone();
            tokio::task::spawn(async move { track_events(controller, control).await });
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
    image_buffer: Arc<Mutex<DisplayData>>,
    texture: TextureHandle,
    i: u32,
}

impl MyEguiApp {
    fn new(cc: &eframe::CreationContext<'_>, image_buffer: Arc<Mutex<DisplayData>>) -> Self {
        // Customize egui here with cc.egui_ctx.set_fonts and cc.egui_ctx.set_visuals.
        // Restore app state using cc.storage (requires the "persistence" feature).
        // Use the cc.gl (a glow::Context) to create graphics shaders and buffers that you can use
        // for e.g. egui::PaintCallback.
        let texture = cc
            .egui_ctx
            .load_texture("display", make_debug_image(), Default::default());

        Self {
            image_buffer,
            texture,
            i: 0,
        }
    }
}

impl eframe::App for MyEguiApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::CentralPanel::default().show(ctx, |ui| {
            let mut thing = self.image_buffer.lock().unwrap();
            self.texture
                .set(thing.image_data.clone(), Default::default());
            let v = thing.telemetry.voltage / 10;
            let voltage = v as f32 / 570.0 * 0.66 * ((150.0 + 680.0) / (150.0));
            thing.voltage_avg = thing.voltage_avg * 0.95 + voltage * 0.05;
            ui.heading(format!("{:.02}V", thing.voltage_avg));
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
