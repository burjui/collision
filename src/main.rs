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
use sdl2::pixels::{Color, PixelFormatEnum};
use sdl2::rect::Rect;
use sdl2::render::{Texture, TextureCreator, WindowCanvas};
use sdl2::ttf::Font;
use sdl2::video::WindowContext;
use sdl2::EventPump;

use physics::object::{Object, ObjectId};
use video_rs::encode::Settings;
use video_rs::{Encoder, Frame, Time};

use crate::config::Config;
use crate::fps::FpsCalculator;
use crate::physics::CollisionDetector;
use crate::scene::*;

mod config;
mod fps;
mod physics;

#[macro_use]
mod scene;

use clap::Parser;

#[derive(Parser, Debug)]
struct Args {
    #[arg(short)]
    video_output_path: Option<String>,
}

fn main() -> anyhow::Result<()> {
    let args = Args::parse();

    env_logger::init();
    video_rs::init().expect("init video-rs");

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
            circle: true,
            id: false,
            velocity: false,
        },
    };

    // Interferes with ffmpeg, disable for now
    // #[cfg(unix)]
    // unsafe {
    //     // Catch NaNs as SIGFPE
    //     feenableexcept(FE_INVALID);
    // }

    let mut collision_detector = CollisionDetector::new();
    create_scene(&mut collision_detector);

    let mut advance_time = false;
    let mut min_fps = u128::MAX;

    let mut video_encoder = args
        .video_output_path
        .map(|video_output_path| {
            Encoder::new(
                Path::new(&video_output_path),
                Settings::preset_h264_yuv420p(
                    config.screen_width as usize,
                    config.screen_height as usize,
                    false,
                ),
            )
            .context("failed to create encoder")
        })
        .transpose()?;

    let frame_duration: Time = Time::from_nth_of_a_second(60);
    let mut last_frame_position = Time::zero();
    let mut last_frame_timestamp = collision_detector.time();

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
            collision_detector.advance(1.0 / 600.0);
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
                "FPS: {fps} (min {min_fps})\ntime: {}\ntotal particles: {}",
                collision_detector.time(),
                collision_detector.object_count()
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

        let frame_sim_duration = 0.3 / 60.0;
        if let Some(video_encoder) = &mut video_encoder {
            if collision_detector.time() - last_frame_timestamp >= frame_sim_duration {
                last_frame_timestamp += frame_sim_duration;
                encode_frame(&canvas, &config, video_encoder, last_frame_position)?;
                last_frame_position = last_frame_position.aligned_with(frame_duration).add();
            }
        }

        frame_count += 1;
    }

    if let Some(video_encoder) = &mut video_encoder {
        video_encoder.finish().expect("failed to finish encoder");
    }

    Ok(())
}

fn encode_frame(
    canvas: &sdl2::render::Canvas<sdl2::video::Window>,
    config: &Config,
    encoder: &mut Encoder,
    position: Time,
) -> Result<(), anyhow::Error> {
    let frame = canvas
        .read_pixels(
            Rect::new(0, 0, config.screen_width, config.screen_height),
            PixelFormatEnum::RGB24,
        )
        .expect("read pixels");
    let frame = Frame::from_shape_vec(
        (
            config.screen_width as usize,
            config.screen_height as usize,
            3,
        ),
        frame,
    )
    .context("frame conversion")?;
    encoder
        .encode(&frame, position)
        .expect("failed to encode frame");
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
            keydown!(Keycode::Space) => {
                *advance_time = !*advance_time;
                if *advance_time {
                    collision_detector.calculate_collisions();
                }
            }

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
    let spectrum_position = object.velocity.magnitude().sqrt().min(15.0) / 15.0;
    let particle_color = spectrum(spectrum_position);
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

fn spectrum(position: f64) -> Color {
    Color::RGB(
        ((1.0 - position) * 255.0) as u8,
        ((0.5 - (position - 0.5).abs()) * 255.0 * 2.0) as u8,
        (position * 255.0) as u8,
    )
}

#[test]
fn spectrum_test() {
    assert_eq!(spectrum(0.0), Color::RGB(255, 0, 0));
    assert_eq!(spectrum(0.5), Color::RGB(127, 255, 127));
    assert_eq!(spectrum(1.0), Color::RGB(0, 0, 255));
}
