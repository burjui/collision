use std::fs::File;
use std::path::Path;

use anyhow::anyhow;
use anyhow::{Context, Result};
use cgmath::{InnerSpace, Vector2};
use itertools::Itertools;
use sdl2::event::Event;
use sdl2::gfx::primitives::DrawRenderer;
use sdl2::keyboard::Keycode;
use sdl2::mouse::MouseButton;
use sdl2::pixels::Color;
use sdl2::rect::Rect;
use sdl2::render::{Texture, TextureCreator, WindowCanvas};
use sdl2::ttf::Font;
use sdl2::video::WindowContext;
use sdl2::EventPump;

use physics::object::{Object, ObjectId};

use crate::config::Config;
use crate::fenv::{feenableexcept, FE_INVALID};
use crate::fps::FpsCalculator;
use crate::physics::CollisionDetector;
use crate::scene::*;

mod config;
mod fenv;
mod fps;
mod physics;

#[macro_use]
mod scene;

fn main() -> Result<()> {
    let _ = File::create("collision.log").context("truncate log")?;
    log4rs::init_file("log4rs.yml", Default::default()).context("init logger")?;

    let config = Config::from_file(Path::new("config.toml")).context("load config")?;

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
        .load_font("/usr/share/fonts/TTF/JetBrainsMono-Regular.ttf", 16)
        .map_err(string_to_anyhow)
        .context("load font")?;
    let texture_creator = canvas.texture_creator();
    let mut stats_text: Option<(Texture<'_>, Rect)> = None;
    let mut frame_count = 0usize;
    let mut fps_calculator = FpsCalculator::new();
    let mut render_settings = RenderSettings {
        with_grid: false,
        details: RenderDetails {
            circle: false,
            id: false,
            velocity: false,
        },
    };

    #[cfg(unix)]
    unsafe {
        // Catch NaNs as SIGFPE
        feenableexcept(FE_INVALID);
    }

    let mut collision_detector = CollisionDetector::new();
    create_scene(&mut collision_detector);

    let mut advance_time = true;
    let mut min_fps = std::u128::MAX;

    'running: loop {
        match process_events(
            &mut event_pump,
            &mut render_settings,
            &mut collision_detector,
            &mut advance_time,
        ) {
            EventResponse::Continue => {}
            EventResponse::Quit => break 'running,
        }

        if advance_time {
            collision_detector.advance(0.003);
        }

        canvas.set_draw_color(Color::RGB(0, 0, 0));
        canvas.clear();

        render_physics(
            &collision_detector,
            config.screen_width,
            &mut canvas,
            &font,
            &render_settings,
        )?;

        if let Some(fps) = fps_calculator.update(frame_count)? {
            min_fps = min_fps.min(fps);
            let stats_string = format!(
                "FPS: {fps} (min {min_fps})\ntime: {}",
                collision_detector.time()
            );
            stats_text = Some(render_text(
                &stats_string,
                config.screen_width,
                &font,
                Color::RGB(255, 255, 255),
                &texture_creator,
            )?);
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

enum EventResponse {
    Continue,
    Quit,
}

macro_rules! keydown {
    ($keycode: pat) => {
        Event::KeyDown {
            keycode: Some($keycode),
            ..
        }
    };
}

fn process_events(
    event_pump: &mut EventPump,
    render_settings: &mut RenderSettings,
    collision_detector: &mut CollisionDetector,
    advance_time: &mut bool,
) -> EventResponse {
    for event in event_pump.poll_iter() {
        match event {
            Event::Quit { .. } | keydown!(Keycode::Escape) => return EventResponse::Quit,
            keydown!(Keycode::P) => emit_scene(collision_detector.objects()).unwrap(),
            keydown!(Keycode::G) => render_settings.with_grid = !render_settings.with_grid,
            keydown!(Keycode::Space) => *advance_time = !*advance_time,

            keydown!(Keycode::D) => {
                render_settings.details = RenderDetails {
                    circle: true,
                    id: true,
                    velocity: true,
                };
            }

            keydown!(Keycode::S) => {
                render_settings.details = RenderDetails {
                    circle: false,
                    id: false,
                    velocity: false,
                };
            }

            keydown!(Keycode::C) => {
                render_settings.details.circle = !render_settings.details.circle;
            }

            keydown!(Keycode::I) => {
                render_settings.details.id = !render_settings.details.id;
            }

            keydown!(Keycode::V) => {
                render_settings.details.velocity = !render_settings.details.velocity;
            }

            Event::MouseButtonDown {
                mouse_btn: MouseButton::Left,
                x,
                y,
                ..
            } => {
                for (id, object) in collision_detector.objects().collect_vec() {
                    let click_position = Vector2::new(x as f64, y as f64);
                    let direction = object.position - click_position;
                    if direction.magnitude() > 1.0 && direction.magnitude() < 70.0 {
                        collision_detector.object_mut(id).velocity += direction.normalize_to(100.0);
                    }
                }
            }

            _ => {}
        }
    }

    EventResponse::Continue
}

fn emit_scene(objects: impl Iterator<Item = (ObjectId, Object)>) -> std::io::Result<()> {
    use std::io::Write;

    let file = &mut File::create(emitted_scene_path!())?;
    writeln!(file, "{{")?;

    for (_, object) in objects {
        let Vector2 { x: px, y: py } = object.position;
        let Vector2 { x: vx, y: vy } = object.velocity;
        writeln!(file, "collision_detector.add(Object {{")?;
        writeln!(file, "    position: Vector2::new({:?}, {:?}),", px, py)?;
        writeln!(file, "    velocity: Vector2::new({:?}, {:?}),", vx, vy)?;
        writeln!(file, "    size: {:?},", object.size)?;
        writeln!(file, "    mass: {:?},", object.mass)?;
        writeln!(file, "}});")?;
    }

    writeln!(file, "}}")?;
    println!("Emitted scene to \"{}\"", emitted_scene_path!());
    Ok(())
}

#[derive(Copy, Clone)]
struct RenderDetails {
    circle: bool,
    id: bool,
    velocity: bool,
}

struct RenderSettings {
    with_grid: bool,
    details: RenderDetails,
}

fn render_physics(
    collision_detector: &CollisionDetector,
    screen_width: u32,
    canvas: &mut WindowCanvas,
    font: &Font,
    settings: &RenderSettings,
) -> Result<()> {
    if settings.with_grid {
        render_grid(collision_detector, canvas)?;
    }

    let texture_creator = canvas.texture_creator();
    for (id, object) in collision_detector.objects() {
        render_object(
            id,
            &object,
            &settings.details,
            screen_width,
            canvas,
            font,
            &texture_creator,
        )?;
    }

    Ok(())
}

fn render_object(
    id: ObjectId,
    object: &Object,
    details: &RenderDetails,
    screen_width: u32,
    canvas: &mut WindowCanvas,
    font: &Font,
    texture_creator: &TextureCreator<WindowContext>,
) -> Result<()> {
    let particle_color = Color::GREEN;
    if details.circle {
        canvas
            .aa_circle(
                object.position.x as i16,
                object.position.y as i16,
                (object.size * 0.5) as i16,
                particle_color,
            )
            .map_err(string_to_anyhow)
            .context("render object circle")?;
    } else {
        canvas
            .pixel(
                object.position.x as i16,
                object.position.y as i16,
                particle_color,
            )
            .map_err(string_to_anyhow)
            .context("render object as pixel")?;
    }

    if details.id {
        let (id_text_texture, id_text_rect) = render_text(
            &id.to_string(),
            screen_width,
            font,
            Color::RGB(100, 100, 100),
            texture_creator,
        )
        .context("draw_physics(): render object id text")?;
        let mut dst_rect = id_text_rect;
        dst_rect.x += (object.position.x - object.size / 2.0) as i32;
        dst_rect.y += (object.position.y - object.size / 2.0) as i32;
        canvas
            .copy(&id_text_texture, id_text_rect, dst_rect)
            .map_err(string_to_anyhow)
            .context("copy object id text to the window surface")?;
    }

    if details.velocity {
        let magnitude = object.velocity.magnitude() + 0.0000001;
        let scale_factor = 4.0 * magnitude.sqrt() / magnitude;
        canvas
            .aa_line(
                object.position.x as i16,
                object.position.y as i16,
                (object.position.x + object.velocity.x * scale_factor) as i16,
                (object.position.y + object.velocity.y * scale_factor) as i16,
                Color::RGB(127, 0, 127),
            )
            .map_err(string_to_anyhow)
            .context("render object velocity vector")?;
    }

    Ok(())
}

fn render_grid(collision_detector: &CollisionDetector, canvas: &mut WindowCanvas) -> Result<()> {
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
                (grid_position.y + cell_size * grid_size.y as f64) as i16,
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
                (grid_position.x + cell_size * grid_size.x as f64) as i16,
                y as i16,
                Color::RGB(100, 100, 100),
            )
            .map_err(string_to_anyhow)
            .context("render grid")?;
        y += cell_size;
    }

    Ok(())
}

fn render_text<'texture>(
    s: &str,
    wrapping_width: u32,
    font: &Font,
    color: Color,
    texture_creator: &'texture TextureCreator<WindowContext>,
) -> Result<(Texture<'texture>, Rect)> {
    let rendered = font
        .render(s)
        .blended_wrapped(color, wrapping_width)
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
