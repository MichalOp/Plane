use std::{
    io::Cursor,
    net::UdpSocket,
    sync::{Arc, Mutex},
    thread,
};

use anyhow;
use bytemuck;
use eframe::egui;
use egui::{Color32, TextureHandle};
use image::{io::Reader as ImageReader, GenericImageView};
use protocol::{self, Packet, Packets, ParseError};
use tracing::{info, warn};
use turbojpeg;

fn server_simple(buffer: Arc<Mutex<egui::ImageData>>) -> anyhow::Result<()> {
    let socket = UdpSocket::bind("0.0.0.0:12892").unwrap();
    // Receives a single datagram message on the socket. If `buf` is too small to hold
    // the message, it will be cut off.
    loop {
        let mut buf = [0; 65536];
        let (amt, _src) = socket.recv_from(&mut buf).unwrap();
        if amt > 0 {
            // info!("Parsing! {}", payload.len());

            let img: Result<image::RgbImage, _> = turbojpeg::decompress_image(&buf);

            if img.is_err() {
                warn!("eh {}", img.err().unwrap());
                continue;
            }
            let img = img.unwrap();

            // info!("Parsed!");

            let mut locked = buffer.lock().unwrap();
            *locked = egui::ImageData::Color(
                egui::ColorImage {
                    size: [img.width() as usize, img.height() as usize],
                    pixels: img
                        .pixels()
                        .map(|x| Color32::from_rgb(x.0[0], x.0[1], x.0[2]))
                        .collect(),
                }
                .into(),
            );
        }
    }
}

fn server(buffer: Arc<Mutex<egui::ImageData>>) -> anyhow::Result<()> {
    let socket = UdpSocket::bind("0.0.0.0:12892").unwrap();
    // Receives a single datagram message on the socket. If `buf` is too small to hold
    // the message, it will be cut off.

    let mut packets = Packets(Vec::new());

    loop {
        let mut to_receive = Packet::new();
        let buf = bytemuck::bytes_of_mut(&mut to_receive);
        let (amt, _src) = socket.recv_from(buf).unwrap();
        packets.0.push(to_receive);
        match packets.parse_and_clear() {
            Ok(payload) => {
                // info!("Parsing! {}", payload.len());

                let img: Result<image::RgbImage, _> = turbojpeg::decompress_image(&payload);

                if img.is_err() {
                    warn!("eh {}", img.err().unwrap());
                    continue;
                }
                let img = img.unwrap();

                // info!("Parsed!");

                let mut locked = buffer.lock().unwrap();
                *locked = egui::ImageData::Color(
                    egui::ColorImage {
                        size: [img.width() as usize, img.height() as usize],
                        pixels: img
                            .pixels()
                            .map(|x| Color32::from_rgb(x.0[0], x.0[1], x.0[2]))
                            .collect(),
                    }
                    .into(),
                )
            }
            Err(ParseError::DroppedPacket(x)) => {
                warn!("Message {} dropped!", x)
            }
            Err(ParseError::NoFullMessage) => {
                // info!("No full message")
            }
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
    let buffer = Arc::new(Mutex::new(make_debug_image()));

    let buffer_cloned = buffer.clone();
    let recv_server = thread::spawn(move || server_simple(buffer_cloned));

    let mut native_options = eframe::NativeOptions::default();
    eframe::run_native(
        "My egui App",
        native_options,
        Box::new(|cc| Box::new(MyEguiApp::new(cc, buffer))),
    )
    .unwrap();
    recv_server.join().unwrap().unwrap();
}

struct MyEguiApp {
    image_buffer: Arc<Mutex<egui::ImageData>>,
    texture: TextureHandle,
    i: u32,
}

impl MyEguiApp {
    fn new(cc: &eframe::CreationContext<'_>, image_buffer: Arc<Mutex<egui::ImageData>>) -> Self {
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
            let thing = self.image_buffer.lock().unwrap();
            self.texture.set(thing.clone(), Default::default());
            ui.image(&self.texture);
            ui.heading(format!("Hello World! {}", self.i));
            self.i += 1;
            ctx.request_repaint();
        });
    }
}
