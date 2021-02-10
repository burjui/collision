use crate::config::Config;
use crate::fps::FpsCalculator;
use sdl2::event::Event;
use sdl2::gfx::primitives::DrawRenderer;
use sdl2::keyboard::Keycode;
use sdl2::pixels::Color;
use sdl2::rect::Rect;
use sdl2::render::Texture;
use std::path::Path;
use std::process::exit;

mod config;
mod fps;

fn main() {
    let config = Config::from_file(Path::new("config.toml")).unwrap_or_else(|error| {
        println!("{}", error);
        exit(1);
    });

    let sdl_context = sdl2::init().unwrap();
    let video_subsystem = sdl_context.video().unwrap();
    let title = format!("{} v{}", env!("CARGO_PKG_NAME"), env!("CARGO_PKG_VERSION"));
    let window = video_subsystem
        .window(&title, config.screen_width, config.screen_height)
        .position(1000, 0)
        .build()
        .unwrap();
    let mut canvas = window.into_canvas().accelerated().build().unwrap();
    let mut event_pump = sdl_context.event_pump().unwrap();
    let ttf_context = sdl2::ttf::init().unwrap();
    let font = ttf_context
        .load_font("/usr/share/fonts/TTF/JetBrainsMono-Regular.ttf", 20)
        .unwrap();
    let texture_creator = canvas.texture_creator();
    let mut stats_text: Option<(Texture<'_>, Rect)> = None;

    let mut frame_count = 0usize;
    let mut fps_calculator = FpsCalculator::new();

    'running: loop {
        canvas.set_draw_color(Color::RGB(0, 0, 0));
        canvas.clear();

        for event in event_pump.poll_iter() {
            match event {
                Event::Quit { .. }
                | Event::KeyDown {
                    keycode: Some(Keycode::Escape),
                    ..
                } => break 'running,

                _ => {}
            }
        }

        if let Some(fps) = fps_calculator.update(frame_count) {
            let stats_string = format!("FPS: {}", fps);
            let rendered = font
                .render(&stats_string)
                .blended_wrapped(Color::RGB(255, 255, 255), config.screen_width)
                .unwrap();
            let rect = rendered.rect();
            let texture = texture_creator
                .create_texture_from_surface(&rendered)
                .unwrap();
            stats_text = Some((texture, rect));
        }

        if let Some((stats_text_texture, stats_text_rect)) = &stats_text {
            canvas
                .copy(stats_text_texture, *stats_text_rect, *stats_text_rect)
                .unwrap();
        }

        canvas
            .aa_circle(100, 100, 10, Color::RGB(100, 100, 255))
            .unwrap();

        canvas.present();
        frame_count += 1;
    }
}
