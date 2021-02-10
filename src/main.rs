use crate::config::Config;
use crate::fps::FpsCalculator;
use crate::physics::{CollisionDetector, PhysicsObject};
use anyhow::anyhow;
use anyhow::{Context, Result};
use cgmath::Vector2;
use rand::random;
use sdl2::event::Event;
use sdl2::gfx::primitives::DrawRenderer;
use sdl2::keyboard::Keycode;
use sdl2::pixels::Color;
use sdl2::rect::Rect;
use sdl2::render::{Texture, TextureCreator, WindowCanvas};
use sdl2::ttf::Font;
use sdl2::video::WindowContext;
use std::path::Path;
use std::process::exit;
use std::time::Instant;

mod config;
mod fps;
mod physics;

fn main() -> Result<()> {
    let config = Config::from_file(Path::new("config.toml")).unwrap_or_else(|error| {
        println!("{}", error);
        exit(1);
    });

    let sdl_context = sdl2::init()
        .map_err(string_to_anyhow)
        .context("init SDL2")?;
    let video_subsystem = sdl_context
        .video()
        .map_err(string_to_anyhow)
        .context("get video context")?;
    let title = format!("{} v{}", env!("CARGO_PKG_NAME"), env!("CARGO_PKG_VERSION"));
    let window = video_subsystem
        .window(&title, config.screen_width, config.screen_height)
        .position(1000, 0)
        .build()
        .context("create window")?;
    let mut canvas = window
        .into_canvas()
        .accelerated()
        .build()
        .context("get canvas")?;
    let mut event_pump = sdl_context
        .event_pump()
        .map_err(string_to_anyhow)
        .context("get event pump")?;
    let ttf_context = sdl2::ttf::init().context("init TTF")?;
    let font = ttf_context
        .load_font("/usr/share/fonts/TTF/JetBrainsMono-Regular.ttf", 20)
        .map_err(string_to_anyhow)
        .context("load font")?;
    let texture_creator = canvas.texture_creator();
    let mut stats_text: Option<(Texture<'_>, Rect)> = None;

    let mut frame_count = 0usize;
    let mut fps_calculator = FpsCalculator::new();

    let mut collision_detector = CollisionDetector::new();
    for i in 0..10 {
        collision_detector.add(PhysicsObject {
            position: Vector2::new(100.0 + i as f32 * 10.0, 100.0 + i as f32 * 50.0),
            velocity: Vector2::new(random::<f32>() * 40.0 - 20.0, random::<f32>() * 40.0 - 20.0),
            size: 30.0,
        });
    }
    collision_detector.update();

    let mut last_measured_time = Instant::now();

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

        let now = Instant::now();
        let dt = (now - last_measured_time).as_secs_f32();
        if dt >= 0.1 {
            last_measured_time = now;
            collision_detector.advance(dt);
            collision_detector.update();
        }

        draw_physics(&collision_detector, &mut canvas, &font)?;

        if let Some(fps) = fps_calculator.update(frame_count)? {
            let stats_string = format!("FPS: {}", fps);
            stats_text = Some(render_text(&stats_string, &font, &texture_creator)?);
        }

        if let Some((stats_text_texture, stats_text_rect)) = &stats_text {
            canvas
                .copy(stats_text_texture, *stats_text_rect, *stats_text_rect)
                .map_err(string_to_anyhow)
                .context("render stats")?;
        }

        canvas.present();
        frame_count += 1;
    }
    Ok(())
}

fn draw_physics(
    collision_detector: &CollisionDetector,
    canvas: &mut WindowCanvas,
    font: &Font,
) -> Result<()> {
    let grid_position = collision_detector
        .grid_position()
        .ok_or_else(|| anyhow!("draw_physics(): grid_position()"))?;
    let grid_size = collision_detector.grid_size();
    let cell_size = collision_detector.grid_cell_size();

    let mut x = grid_position.x;
    for _ in 0..=grid_size.x {
        canvas
            .line(
                x as i16,
                grid_position.y as i16,
                x as i16,
                (grid_position.y + cell_size * grid_size.y as f32) as i16,
                Color::RGB(100, 100, 100),
            )
            .map_err(string_to_anyhow)
            .context("render grid")?;
        x += cell_size;
    }

    let mut y = grid_position.y;
    for _ in 0..=grid_size.y {
        canvas
            .line(
                grid_position.x as i16,
                y as i16,
                (grid_position.x + cell_size * grid_size.x as f32) as i16,
                y as i16,
                Color::RGB(100, 100, 100),
            )
            .map_err(string_to_anyhow)
            .context("render grid")?;
        y += cell_size;
    }

    for (id, object) in collision_detector.objects() {
        let texture_creator = canvas.texture_creator();
        let (id_text_texture, id_text_rect) = render_text(&id.to_string(), font, &texture_creator)
            .context("draw_physics(): render object id text")?;
        let mut dst_rect = id_text_rect;
        dst_rect.x += (object.position.x - object.size / 2.0) as i32;
        dst_rect.y += (object.position.y - object.size / 2.0) as i32;
        canvas
            .copy(&id_text_texture, id_text_rect, dst_rect)
            .map_err(string_to_anyhow)
            .context("copy object id text to the window surface")?;
        canvas
            .aa_circle(
                object.position.x as i16,
                object.position.y as i16,
                object.size as i16 / 2,
                Color::RGB(100, 100, 255),
            )
            .map_err(string_to_anyhow)
            .context("render object circle")?;
        canvas
            .aa_line(
                object.position.x as i16,
                object.position.y as i16,
                (object.position.x + object.velocity.x * 3.0) as i16,
                (object.position.y + object.velocity.y * 3.0) as i16,
                Color::RGB(100, 255, 255),
            )
            .map_err(string_to_anyhow)
            .context("render object velocity vector")?;
    }

    Ok(())
}

fn render_text<'texture>(
    s: &str,
    font: &Font,
    texture_creator: &'texture TextureCreator<WindowContext>,
) -> Result<(Texture<'texture>, Rect)> {
    let rendered = font
        .render(&s)
        .blended(Color::RGB(255, 255, 255))
        .context("render text")?;
    let rect = rendered.rect();
    let texture = texture_creator
        .create_texture_from_surface(&rendered)
        .context("convert text surface to texture")?;
    Ok((texture, rect))
}

fn string_to_anyhow(s: String) -> anyhow::Error {
    anyhow!("{}", s)
}
