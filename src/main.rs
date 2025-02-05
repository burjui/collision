#![feature(get_many_mut)]
#![feature(anonymous_lifetime_in_impl_trait)]

use std::{fs::File, path::Path};

use anyhow::{anyhow, Context, Result};
use collision::physics::{grid::cell_at, object::Object, ConstraintBox, PhysicsEngine};
use itertools::Itertools;
use nalgebra::Vector2;
use sdl2::{
    event::Event,
    gfx::primitives::DrawRenderer,
    keyboard::{Keycode, Mod},
    mouse::MouseButton,
    pixels::{Color, PixelFormatEnum},
    rect::Rect,
    render::{Texture, TextureCreator, WindowCanvas},
    ttf::Font,
    video::WindowContext,
    EventPump,
};
use video_rs::{encode::Settings, Encoder, Frame, Time};

use crate::{config::Config, fps::FpsCalculator, scene::*};

mod config;
mod fps;

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

    let sdl_context = sdl2::init().map_err(string_to_anyhow).context("init SDL2")?;
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
    let mut canvas = window.into_canvas().accelerated().build().context("get canvas")?;
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
            as_circle: true,
            id: false,
            energy: false,
            momentum: false,
            velocity: false,
        },
    };

    // Interferes with ffmpeg, disable for now
    // #[cfg(unix)]
    // unsafe {
    //     // Catch NaNs as SIGFPE
    //     feenableexcept(FE_INVALID);
    // }

    let constraints = ConstraintBox::new(
        Vector2::new(0.0, 0.0),
        Vector2::new(config.screen_width as f64, config.screen_height as f64),
    );
    let mut physics = PhysicsEngine::new(constraints)?;
    create_scene(&mut physics);

    let mut advance_time = false;
    let mut min_fps = u128::MAX;

    let mut video_encoder = args
        .video_output_path
        .map(|video_output_path| {
            Encoder::new(
                Path::new(&video_output_path),
                Settings::preset_h264_yuv420p(config.screen_width as usize, config.screen_height as usize, false),
            )
            .context("failed to create encoder")
        })
        .transpose()?;

    let frame_duration: Time = Time::from_nth_of_a_second(60);
    let mut last_frame_position = Time::zero();
    let mut last_frame_timestamp = physics.time();
    let mut mouse_position = Vector2::new(0.0, 0.0);

    const DEFAULT_DT: f64 = 1.0 / 60.0 / 16.0;
    'running: loop {
        match process_events(
            &mut event_pump,
            &mut render_settings,
            &mut physics,
            &mut advance_time,
            &mut mouse_position,
        ) {
            EventResponse::Continue => {}
            EventResponse::Quit => break 'running,
        }

        if advance_time {
            physics.advance(DEFAULT_DT, 2);
        }

        canvas.set_draw_color(Color::RGB(0, 0, 0));
        canvas.clear();

        render_physics(
            &physics,
            config.screen_width,
            &mut canvas,
            &font,
            &render_settings,
            mouse_position,
        )?;

        if let Some(fps) = fps_calculator.update(frame_count)? {
            min_fps = min_fps.min(fps);
            let stats_string = format!(
                "FPS: {fps} (min {min_fps})\ntime: {}\ntotal particles: {}\nsolver: {}",
                physics.time(),
                physics.objects().len(),
                physics.solver_kind
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

        let frame_sim_duration = 0.1 / 60.0;
        if let Some(video_encoder) = &mut video_encoder {
            if physics.time() - last_frame_timestamp >= frame_sim_duration {
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
    let frame = Frame::from_shape_vec((config.screen_width as usize, config.screen_height as usize, 3), frame)
        .context("frame conversion")?;
    encoder.encode(&frame, position).expect("failed to encode frame");
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
    ($keycode: pat, $keymod: pat) => {
        Event::KeyDown {
            keycode: Some($keycode),
            keymod: $keymod,
            ..
        }
    };
}

fn process_events(
    event_pump: &mut EventPump,
    render_settings: &mut RenderSettings,
    physics: &mut PhysicsEngine,
    advance_time: &mut bool,
    mouse_position: &mut Vector2<f64>,
) -> EventResponse {
    for event in event_pump.poll_iter() {
        match event {
            Event::Quit { .. } | keydown!(Keycode::Escape) => return EventResponse::Quit,
            keydown!(Keycode::P) => emit_scene(physics.objects()).unwrap(),
            keydown!(Keycode::G) => render_settings.with_grid = !render_settings.with_grid,
            keydown!(Keycode::Space) => *advance_time = !*advance_time,

            keydown!(Keycode::D, Mod::LSHIFTMOD | Mod::RSHIFTMOD) => {
                render_settings.details = RenderDetails {
                    as_circle: false,
                    id: false,
                    energy: false,
                    momentum: false,
                    velocity: false,
                };
            }

            keydown!(Keycode::D) => {
                render_settings.details = RenderDetails {
                    as_circle: true,
                    id: true,
                    energy: true,
                    momentum: true,
                    velocity: true,
                };
            }

            keydown!(Keycode::S) => physics.solver_kind = physics.solver_kind.next(),

            keydown!(Keycode::C) => {
                render_settings.details.as_circle = !render_settings.details.as_circle;
            }

            keydown!(Keycode::I) => {
                render_settings.details.id = !render_settings.details.id;
            }

            keydown!(Keycode::E) => {
                render_settings.details.energy = !render_settings.details.energy;
            }

            keydown!(Keycode::M) => {
                render_settings.details.momentum = !render_settings.details.momentum;
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
                for object in physics.objects_mut() {
                    let click_position = Vector2::new(x as f64, y as f64);
                    let direction = object.position - click_position;
                    if direction.magnitude() > 0.0 && direction.magnitude() < 70.0 {
                        object.set_velocity(object.velocity() + direction.normalize());
                    }
                }
            }

            Event::MouseMotion { x, y, .. } => *mouse_position = Vector2::new(x as f64, y as f64),

            _ => {}
        }
    }

    EventResponse::Continue
}

fn emit_scene(objects: &[Object]) -> std::io::Result<()> {
    use std::io::Write;

    let file = &mut File::create(emitted_scene_path!())?;
    writeln!(file, "{{")?;

    for object in objects {
        let (ppx, ppy) = (object.position.x, object.position.y);
        let (cpx, cpy) = (object.velocity().x, object.velocity().y);
        let (ax, ay) = (object.acceleration.x, object.acceleration.y);
        writeln!(file, "physics.add(Object {{")?;
        writeln!(file, "    previous_position: Vector2::new({ppx:?}, {ppy:?}),")?;
        writeln!(file, "    position: Vector2::new({cpx:?}, {cpy:?}),")?;
        writeln!(file, "    acceleration: Vector2::new({ax:?}, {ay:?}),")?;
        writeln!(file, "    radius: {:?},", object.radius)?;
        writeln!(file, "    mass: {:?},", object.mass)?;
        writeln!(file, "}});")?;
    }
    writeln!(file, "}}")?;
    println!("Emitted scene to \"{}\"", emitted_scene_path!());
    Ok(())
}

#[derive(Copy, Clone)]
struct RenderDetails {
    as_circle: bool,
    id: bool,
    energy: bool,
    momentum: bool,
    velocity: bool,
}

struct RenderSettings {
    with_grid: bool,
    details: RenderDetails,
}

fn render_physics(
    physics: &PhysicsEngine,
    screen_width: u32,
    canvas: &mut WindowCanvas,
    font: &Font,
    settings: &RenderSettings,
    mouse_position: Vector2<f64>,
) -> Result<()> {
    let texture_creator = canvas.texture_creator();
    for (object_index, object) in physics.objects().iter().enumerate() {
        render_object(
            object_index,
            object,
            physics,
            &settings.details,
            screen_width,
            canvas,
            font,
            &texture_creator,
        )?;
    }

    if settings.with_grid {
        render_grid(physics, canvas, mouse_position, &settings.details)?;
    }

    let p1 = physics.constraints().topleft;
    let p2 = physics.constraints().bottomright;
    canvas.set_draw_color(Color::WHITE);
    canvas
        .draw_rect(Rect::new(
            p1.x as i32,
            p1.y as i32,
            (p2.x - p1.x) as u32,
            (p2.y - p1.y) as u32,
        ))
        .map_err(string_to_anyhow)
        .context("render constraints")?;

    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn render_object(
    object_index: usize,
    object: &Object,
    physics: &PhysicsEngine,
    details: &RenderDetails,
    screen_width: u32,
    canvas: &mut WindowCanvas,
    font: &Font,
    texture_creator: &TextureCreator<WindowContext>,
) -> Result<()> {
    let spectrum_position = object.velocity().magnitude().min(1.0);
    let particle_color = object.color.unwrap_or_else(|| spectrum(spectrum_position));
    render_object_outline(object, particle_color, canvas, details.as_circle)?;

    if details.id && physics.objects()[object_index].is_planet {
        let (id_text_texture, id_text_rect) = render_text(
            &object_index.to_string(),
            screen_width,
            font,
            Color::WHITE,
            texture_creator,
        )
        .context("draw_physics(): render object id text")?;
        let mut dst_rect = id_text_rect;
        dst_rect.x += (object.position.x - object.radius / 2.0) as i32;
        dst_rect.y += (object.position.y - object.radius / 2.0) as i32;
        canvas
            .copy(&id_text_texture, id_text_rect, dst_rect)
            .map_err(string_to_anyhow)
            .context("copy object id text to the window surface")?;
    }

    let velocity_magnitude = object.velocity().magnitude();
    if details.energy {
        let energy = 0.5 * velocity_magnitude * velocity_magnitude * object.mass;
        const SCALE_FACTOR: f64 = 100.0;
        let factor = (energy * SCALE_FACTOR).min(1.0);
        canvas
            .filled_circle(
                object.position.x as i16,
                object.position.y as i16,
                3,
                Color::RGBA(255, 0, 255, (factor * 255.0) as u8),
            )
            .map_err(string_to_anyhow)
            .context("render object velocity vector")?;
    }

    if details.momentum {
        let momentum = velocity_magnitude * object.mass;
        const SCALE_FACTOR: f64 = 2.0;
        let factor = (momentum * SCALE_FACTOR).min(1.0);
        canvas
            .filled_circle(
                object.position.x as i16,
                object.position.y as i16,
                3,
                Color::RGBA(0, 0, 255, (factor * 255.0) as u8),
            )
            .map_err(string_to_anyhow)
            .context("render object velocity vector")?;
    }

    if details.velocity {
        const SCALE_FACTOR: f64 = 1.5;
        let factor = (velocity_magnitude * SCALE_FACTOR).min(1.0);
        canvas
            .filled_circle(
                object.position.x as i16,
                object.position.y as i16,
                3,
                Color::RGBA(0, 255, 255, (factor * 255.0) as u8),
            )
            .map_err(string_to_anyhow)
            .context("render object velocity vector")?;
    }

    Ok(())
}

fn render_object_outline(
    object: &Object,
    color: Color,
    canvas: &mut WindowCanvas,
    as_circle: bool,
) -> Result<(), anyhow::Error> {
    if as_circle {
        canvas
            .filled_circle(
                object.position.x as i16,
                object.position.y as i16,
                object.radius as i16,
                color,
            )
            .map_err(string_to_anyhow)
            .context("render object circle")
    } else {
        canvas
            .pixel(object.position.x as i16, object.position.y as i16, color)
            .map_err(string_to_anyhow)
            .context("render object as pixel")
    }
}

fn render_grid(
    physics: &PhysicsEngine,
    canvas: &mut WindowCanvas,
    mouse_position: Vector2<f64>,
    details: &RenderDetails,
) -> Result<()> {
    let grid_position = physics.grid().position();
    let grid_size = physics.grid().size();
    let cell_size = physics.grid().cell_size();

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

    let cell @ (x, y) = cell_at(mouse_position, grid_position, cell_size);
    if x < grid_size.x && y < grid_size.y {
        canvas.set_draw_color(Color::WHITE);
        let highlight_rect = Rect::new(
            (grid_position.x + x as f64 * cell_size) as i32,
            (grid_position.y + y as f64 * cell_size) as i32,
            cell_size as u32,
            cell_size as u32,
        );
        canvas
            .draw_rect(highlight_rect)
            .map_err(string_to_anyhow)
            .with_context(|| format!("highlight mouse cell {cell:?}"))?;

        for adjacent_cell @ (x, y) in (x.saturating_sub(1)..=x + 1).cartesian_product(y.saturating_sub(1)..=y + 1) {
            if x < grid_size.x && y < grid_size.y {
                for &object_index in &physics.grid().cells()[(x, y)] {
                    let object = &physics.objects()[object_index];
                    render_object_outline(object, Color::YELLOW, canvas, details.as_circle)
                        .context("highlight object")?;
                }
                if adjacent_cell != cell {
                    canvas.set_draw_color(Color::MAGENTA);
                    let highlight_rect = Rect::new(
                        (grid_position.x + x as f64 * cell_size) as i32,
                        (grid_position.y + y as f64 * cell_size) as i32,
                        cell_size as u32,
                        cell_size as u32,
                    );
                    canvas
                        .draw_rect(highlight_rect)
                        .map_err(string_to_anyhow)
                        .with_context(|| format!("highlight mouse cell {adjacent_cell:?}"))?;
                }
            }
        }
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
