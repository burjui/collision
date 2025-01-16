#![feature(get_many_mut)]
#![feature(anonymous_lifetime_in_impl_trait)]

use std::{fs::File, path::Path};

use anyhow::{anyhow, Context, Result};
use itertools::Itertools;
use nalgebra::Vector2;
use physics::{grid::Cell, object::Object, ConstraintBox};
use sdl2::{
    event::Event,
    gfx::primitives::DrawRenderer,
    keyboard::Keycode,
    mouse::MouseButton,
    pixels::{Color, PixelFormatEnum},
    rect::Rect,
    render::{Texture, TextureCreator, WindowCanvas},
    ttf::Font,
    video::WindowContext,
    EventPump,
};
use video_rs::{encode::Settings, Encoder, Frame, Time};

use crate::{
    config::Config,
    fps::FpsCalculator,
    physics::{grid::cell_at, PhysicsEngine},
    scene::*,
};

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
            velocity: false,
        },
    };

    // Interferes with ffmpeg, disable for now
    // #[cfg(unix)]
    // unsafe {
    //     // Catch NaNs as SIGFPE
    //     feenableexcept(FE_INVALID);
    // }

    let mut physics = PhysicsEngine::new();
    create_scene(&mut physics);
    physics.constraints = Some(ConstraintBox::new(
        Vector2::new(0.0, 0.0),
        Vector2::new(config.screen_width as f64, config.screen_height as f64),
    ));

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
            physics.advance(1.0 / 60.0);
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
                "FPS: {fps} (min {min_fps})\ntime: {}\ntotal particles: {}",
                physics.time(),
                physics.object_count()
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

            keydown!(Keycode::D) => {
                render_settings.details = RenderDetails {
                    as_circle: true,
                    id: true,
                    velocity: true,
                };
            }

            keydown!(Keycode::S) => {
                render_settings.details = RenderDetails {
                    as_circle: false,
                    id: false,
                    velocity: false,
                };
            }

            keydown!(Keycode::C) => {
                render_settings.details.as_circle = !render_settings.details.as_circle;
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
                for (id, object) in physics.objects().collect_vec() {
                    let click_position = Vector2::new(x as f64, y as f64);
                    let direction = object.position - click_position;
                    if direction.magnitude() > 1.0 && direction.magnitude() < 70.0 {
                        let object = physics.object_mut(id);
                        object.set_velocity(object.velocity() + direction.normalize() * 1.0, 1.0);
                    }
                }
            }

            Event::MouseMotion { x, y, .. } => *mouse_position = Vector2::new(x as f64, y as f64),

            _ => {}
        }
    }

    EventResponse::Continue
}

fn emit_scene(objects: impl Iterator<Item = (usize, Object)>) -> std::io::Result<()> {
    use std::io::Write;

    let file = &mut File::create(emitted_scene_path!())?;
    writeln!(file, "{{")?;

    for (_, object) in objects {
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
    for (id, object) in physics.objects() {
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

    if settings.with_grid {
        render_grid(physics, canvas, mouse_position, &settings.details)?;
    }

    Ok(())
}

fn render_object(
    object_index: usize,
    object: &Object,
    details: &RenderDetails,
    screen_width: u32,
    canvas: &mut WindowCanvas,
    font: &Font,
    texture_creator: &TextureCreator<WindowContext>,
) -> Result<()> {
    if details.velocity {
        let magnitude = object.velocity().magnitude() + 0.0000001;
        let scale_factor = 4.0 * magnitude.sqrt() / magnitude;
        canvas
            .aa_line(
                object.position.x as i16,
                object.position.y as i16,
                (object.position.x + object.velocity().x * scale_factor) as i16,
                (object.position.y + object.velocity().y * scale_factor) as i16,
                Color::RGB(127, 0, 127),
            )
            .map_err(string_to_anyhow)
            .context("render object velocity vector")?;
    }

    let spectrum_position = object.velocity().magnitude().min(1.0);
    let particle_color = object.color.unwrap_or_else(|| spectrum(spectrum_position));
    render_object_outline(object, particle_color, canvas, details.as_circle)?;

    if details.id {
        let (id_text_texture, id_text_rect) = render_text(
            &object_index.to_string(),
            screen_width,
            font,
            Color::RGB(100, 100, 100),
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

    Ok(())
}

fn render_object_outline(
    object: &Object,
    color: Color,
    canvas: &mut WindowCanvas,
    as_circle: bool,
) -> Result<(), anyhow::Error> {
    Ok(if as_circle {
        canvas
            .aa_circle(
                object.position.x as i16,
                object.position.y as i16,
                object.radius as i16,
                color,
            )
            .map_err(string_to_anyhow)
            .context("render object circle")?;
    } else {
        canvas
            .pixel(object.position.x as i16, object.position.y as i16, color)
            .map_err(string_to_anyhow)
            .context("render object as pixel")?;
    })
}

fn render_grid(
    physics: &PhysicsEngine,
    canvas: &mut WindowCanvas,
    mouse_position: Vector2<f64>,
    details: &RenderDetails,
) -> Result<()> {
    let grid_position = physics.grid.position;
    let grid_size = physics.grid.size;
    let cell_size = physics.grid.cell_size;

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

    let cell = cell_at(mouse_position, grid_position, cell_size);
    if cell.x < grid_size.x && cell.y < grid_size.y {
        canvas.set_draw_color(Color::WHITE);
        let highlight_rect = Rect::new(
            (grid_position.x + cell.x as f64 * cell_size) as i32,
            (grid_position.y + cell.y as f64 * cell_size) as i32,
            cell_size as u32,
            cell_size as u32,
        );
        canvas
            .draw_rect(highlight_rect)
            .map_err(string_to_anyhow)
            .with_context(|| format!("highlight mouse cell {cell:?}"))?;

        for &object_index in &physics.grid.cells[(cell.x, cell.y)] {
            let object = &physics.grid.objects[object_index];
            render_object_outline(object, Color::YELLOW, canvas, details.as_circle).context("highlight object")?;
        }

        for adjacent_cell in (cell.x - 1..=cell.x + 1)
            .cartesian_product(cell.y - 1..=cell.y + 1)
            .map(|(x, y)| Cell::new(x, y))
        {
            if adjacent_cell != cell && adjacent_cell.x < grid_size.x && adjacent_cell.y < grid_size.y {
                canvas.set_draw_color(Color::MAGENTA);
                let highlight_rect = Rect::new(
                    (grid_position.x + adjacent_cell.x as f64 * cell_size) as i32,
                    (grid_position.y + adjacent_cell.y as f64 * cell_size) as i32,
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
