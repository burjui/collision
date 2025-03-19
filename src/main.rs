use core::f64;
use std::{iter::zip, num::NonZero, sync::Arc, time::Instant};

use anyhow::{anyhow, Ok};
use collision::{
    app_config::{config, config_mut, ColorSource, TimeLimitAction},
    fps::FpsCalculator,
    physics::{DurationStat, PhysicsEngine},
    simple_text::SimpleText,
    vector2::Vector2,
};
use demo::create_demo;
use itertools::Itertools;
use vello::{
    kurbo::{Affine, Circle, Line, Rect, Stroke},
    peniko::{color::palette::css, Color, Fill},
    util::{DeviceHandle, RenderContext, RenderSurface},
    wgpu::{Maintain, PresentMode},
    AaConfig, AaSupport, RenderParams, Renderer, RendererOptions, Scene,
};
use winit::{
    application::ApplicationHandler,
    dpi::PhysicalSize,
    event::{ElementState, MouseButton, MouseScrollDelta, WindowEvent},
    event_loop::{ActiveEventLoop, EventLoop},
    keyboard::{Key, NamedKey},
    window::{Window, WindowId},
};

mod demo;

pub fn main() -> anyhow::Result<()> {
    enable_floating_point_exceptions();
    let event_loop = EventLoop::new()?;
    let render_context = RenderContext::new();
    let renderers: Vec<Option<Renderer>> = vec![];
    let render_state = None::<RenderState<'_>>;
    let config = config();
    let mut physics = PhysicsEngine::new()?;
    create_demo(&mut physics);
    let advance_time = config.simulation.time_limit.is_some();
    let mut app = VelloApp {
        context: render_context,
        renderers,
        state: render_state,
        cached_window: None,
        scene: Scene::new(),
        frame_count: 0,
        fps_calculator: FpsCalculator::default(),
        last_fps: 0,
        min_fps: usize::MAX,
        physics,
        advance_time,
        mouse_position: Vector2::new(0.0, 0.0),
        mouse_influence_radius: 50.0,
        draw_grid: false,
        draw_ids: false,
        text: SimpleText::new(),
        last_redraw: Instant::now(),
        time_limit_action_executed: false,
        jerk_applied: false,
    };

    event_loop.run_app(&mut app).expect("run to completion");

    let mut stats_buffer = String::new();
    write_stats(&mut stats_buffer, (app.last_fps, app.min_fps), &app.physics)?;
    print!("{}", stats_buffer);

    Ok(())
}

fn enable_floating_point_exceptions() {
    unsafe extern "C" {
        fn feenableexcept(excepts: i32) -> i32;
    }
    unsafe {
        feenableexcept(1);
    }
}

struct RenderState<'s> {
    // SAFETY: We MUST drop the surface before the `window`, so the fields
    // must be in this order
    surface: RenderSurface<'s>,
    window: Arc<Window>,
}

struct VelloApp<'s> {
    context: RenderContext,
    renderers: Vec<Option<Renderer>>,
    state: Option<RenderState<'s>>,
    // Whilst suspended, we drop `render_state`, but need to keep the same window.
    // If render_state exists, we must store the window in it, to maintain drop order
    cached_window: Option<Arc<Window>>,
    scene: Scene,
    frame_count: usize,
    fps_calculator: FpsCalculator,
    last_fps: usize,
    min_fps: usize,
    physics: PhysicsEngine,
    advance_time: bool,
    mouse_position: Vector2<f64>,
    mouse_influence_radius: f64,
    draw_grid: bool,
    draw_ids: bool,
    text: SimpleText,
    last_redraw: Instant,
    time_limit_action_executed: bool,
    jerk_applied: bool,
}

impl ApplicationHandler<()> for VelloApp<'_> {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.state.is_some() {
            return;
        }
        let window = self.cached_window.take().unwrap_or_else(|| {
            Arc::new(
                event_loop
                    .create_window(
                        Window::default_attributes()
                            .with_inner_size(PhysicalSize::new(config().window.width, config().window.height))
                            .with_resizable(false)
                            .with_title(format!("{} v{}", env!("CARGO_PKG_NAME"), env!("CARGO_PKG_VERSION"))),
                    )
                    .unwrap(),
            )
        });
        let size = window.inner_size();
        let surface_future =
            self.context
                .create_surface(window.clone(), size.width, size.height, PresentMode::AutoNoVsync);
        // We need to block here, in case a Suspended event appeared
        let surface = pollster::block_on(surface_future).expect("Error creating surface");
        self.state = {
            let render_state = RenderState { surface, window };
            self.renderers.resize_with(self.context.devices.len(), || None);
            let id = render_state.surface.dev_id;
            self.renderers[id].get_or_insert_with(|| {
                Renderer::new(
                    &self.context.devices[id].device,
                    RendererOptions {
                        surface_format: Some(render_state.surface.format),
                        use_cpu: false,
                        antialiasing_support: AaSupport::area_only(),
                        num_init_threads: NonZero::new(2),
                    },
                )
                .map_err(|e| anyhow!("{e}"))
                .expect("Failed to create renderer")
            });
            Some(render_state)
        };
    }

    fn window_event(&mut self, event_loop: &ActiveEventLoop, window_id: WindowId, event: WindowEvent) {
        let Some(render_state) = &mut self.state else {
            return;
        };
        if render_state.window.id() != window_id {
            return;
        }
        match event {
            WindowEvent::CloseRequested => event_loop.exit(),
            WindowEvent::KeyboardInput { event, .. } => {
                if event.state == ElementState::Pressed {
                    match event.logical_key.as_ref() {
                        Key::Named(NamedKey::Escape) => event_loop.exit(),
                        Key::Named(NamedKey::Space) => {
                            self.advance_time = !self.advance_time;
                            request_redraw(&self.state);
                        }
                        Key::Character("a") => {
                            config_mut().simulation.enable_gpu = !config().simulation.enable_gpu;
                            request_redraw(&self.state);
                        }
                        Key::Character("g") => {
                            self.draw_grid = !self.draw_grid;
                            request_redraw(&self.state);
                        }
                        Key::Character("i") => {
                            self.draw_ids = !self.draw_ids;
                            request_redraw(&self.state);
                        }
                        Key::Character("v") => {
                            config_mut().rendering.color_source =
                                Some(match config().rendering.color_source.unwrap_or_default() {
                                    ColorSource::Demo => ColorSource::Velocity,
                                    ColorSource::Velocity => ColorSource::Demo,
                                });
                            request_redraw(&self.state);
                        }
                        _ => {}
                    }
                }
            }
            WindowEvent::Resized(size) => {
                if let Some(RenderState { surface, window }) = &mut self.state {
                    self.context.resize_surface(surface, size.width, size.height);
                    window.request_redraw();
                }
            }
            WindowEvent::RedrawRequested => {
                if let Some(RenderState { surface, .. }) = &self.state {
                    self.last_fps = self.fps_calculator.update(self.frame_count).unwrap_or(self.last_fps);
                    self.min_fps = self.min_fps.min(self.last_fps);

                    let now = Instant::now();
                    if (now - self.last_redraw).as_secs_f64() > 1.0 / 60.0 {
                        self.last_redraw = now;
                        self.scene.reset();
                        draw_physics(&self.physics, &mut self.scene, DrawIds(self.draw_ids));
                        draw_mouse_influence(&mut self.scene, self.mouse_position, self.mouse_influence_radius);
                        if self.draw_grid && self.physics.grid().cell_size() > 0.0 {
                            draw_grid(
                                &mut self.scene,
                                self.physics.constraints().topleft,
                                self.physics.constraints().bottomright,
                                self.physics.grid().cell_size(),
                            );
                        }
                        draw_stats(
                            &mut self.scene,
                            &mut self.text,
                            (self.last_fps, self.min_fps),
                            &self.physics,
                        )
                        .expect("failed to draw stats");
                        let renderer = self.renderers[surface.dev_id].as_mut().expect("failed to get renderer");
                        let device_handle = &self.context.devices[surface.dev_id];
                        render_scene(&self.scene, surface, renderer, device_handle);
                        self.frame_count += 1;
                    }
                }
            }
            WindowEvent::CursorMoved { position, .. } => {
                self.mouse_position = Vector2::new(position.x as f64, position.y as f64);
                request_redraw(&self.state);
            }
            WindowEvent::MouseInput { state, button, .. } => {
                if state == ElementState::Pressed {
                    if let MouseButton::Left = button {
                        let objects = self.physics.objects_mut();
                        for (&position, velocity) in zip(&objects.positions, &mut objects.velocities) {
                            let from_mouse_to_object = position - self.mouse_position;
                            if (from_mouse_to_object).magnitude() < self.mouse_influence_radius {
                                *velocity += from_mouse_to_object.normalize() * 2000.0;
                            }
                        }
                        request_redraw(&self.state);
                    }
                }
            }
            WindowEvent::MouseWheel {
                delta: MouseScrollDelta::LineDelta(_, dy),
                ..
            } => {
                self.mouse_influence_radius = (self.mouse_influence_radius + dy as f64 * 3.0).max(0.0);
                request_redraw(&self.state);
            }
            _ => {}
        }
    }

    fn about_to_wait(&mut self, event_loop: &ActiveEventLoop) {
        let config = config();
        if config
            .simulation
            .time_limit
            .is_some_and(|limit| self.physics.time() > limit)
            && !self.time_limit_action_executed
        {
            self.time_limit_action_executed = true;
            match config.simulation.time_limit_action.unwrap_or_default() {
                TimeLimitAction::Exit => event_loop.exit(),
                TimeLimitAction::Pause => {
                    self.advance_time = false;
                    request_redraw(&self.state);
                }
            }
        }

        if self.advance_time {
            self.physics.advance(config.simulation.speed_factor);
            if config
                .simulation
                .jerk_at
                .is_some_and(|jerk_at| self.physics.time() >= jerk_at)
                && !self.jerk_applied
            {
                self.jerk_applied = true;
                let first_non_planet = self.physics.planet_count();
                self.physics.objects_mut().velocities[first_non_planet] += Vector2::new(1000.0, 1000.0);
            }
            request_redraw(&self.state);
        }
        let now = Instant::now();
        if (now - self.last_redraw).as_secs_f64() > 1.0 / 60.0 {
            request_redraw(&self.state);
        }
    }

    fn suspended(&mut self, _event_loop: &ActiveEventLoop) {
        // When we suspend, we need to remove the `wgpu` Surface
        if let Some(render_state) = self.state.take() {
            self.cached_window = Some(render_state.window);
        }
    }
}

fn request_redraw(render_state: &Option<RenderState<'_>>) {
    if let Some(render_state) = render_state {
        render_state.window.request_redraw();
    }
}

struct DrawIds(bool);

fn draw_physics(physics: &PhysicsEngine, scene: &mut Scene, DrawIds(draw_ids): DrawIds) {
    let config = config();
    let transform = Affine::IDENTITY;
    let objects = physics.objects();
    let chunk_size = physics.objects().len() / 20;
    let chunks = (0..physics.objects().len())
        .chunks(if chunk_size > 0 {
            chunk_size
        } else {
            physics.objects().len()
        })
        .into_iter()
        .map(|chunk| chunk.collect_vec())
        .collect_vec();
    std::thread::scope(|scope| {
        let handles = chunks
            .iter()
            .map(|chunk| {
                scope.spawn(move || {
                    let mut scene = Scene::new();
                    let mut text = SimpleText::new();
                    for &object_index in chunk {
                        if !objects.is_planet[object_index] {
                            let particle_position = objects.positions[object_index];
                            let particle_radius = objects.radii[object_index];
                            let color = match config.rendering.color_source.unwrap_or_default() {
                                ColorSource::Demo => objects.colors[object_index],
                                ColorSource::Velocity => {
                                    const SCALE_FACTOR: f64 = 0.0004;
                                    let velocity = objects.velocities[object_index];
                                    let spectrum_position =
                                        (velocity.magnitude() * SCALE_FACTOR).powf(0.6).clamp(0.0, 1.0) as f32;
                                    Some(spectrum(spectrum_position))
                                }
                            };
                            scene.fill(
                                Fill::NonZero,
                                transform,
                                color.unwrap_or(css::GRAY),
                                None,
                                &Circle::new((particle_position.x, particle_position.y), particle_radius.into()),
                            );
                            if draw_ids {
                                text.add(
                                    &mut scene,
                                    10.0,
                                    None,
                                    Affine::translate((particle_position.x as f64, particle_position.y as f64)),
                                    &format!("{}", object_index),
                                );
                            }
                        }
                    }
                    scene
                })
            })
            .collect_vec();
        for handle in handles {
            scene.append(&handle.join().unwrap(), None);
        }
    });

    for ((&planet_position, &planet_radius), color) in zip(
        zip(
            &objects.positions[..objects.planet_count],
            &objects.radii[..objects.planet_count],
        ),
        &objects.colors[..objects.planet_count],
    ) {
        scene.fill(
            Fill::NonZero,
            transform,
            color.unwrap_or(css::WHITE),
            None,
            &Circle::new((planet_position.x, planet_position.y), planet_radius.into()),
        );
    }

    let topleft = physics.constraints().topleft;
    let bottomright = physics.constraints().bottomright;
    scene.stroke(
        &Stroke::default(),
        transform,
        css::WHITE,
        None,
        &Rect::new(
            topleft.x as f64,
            topleft.y as f64,
            bottomright.x as f64,
            bottomright.y as f64,
        ),
    );
}

fn draw_mouse_influence(scene: &mut Scene, mouse_position: Vector2<f64>, mouse_influence_radius: f64) {
    scene.fill(
        Fill::NonZero,
        Affine::IDENTITY,
        Color::new([1.0, 1.0, 1.0, 0.3]),
        None,
        &Circle::new((mouse_position.x, mouse_position.y), mouse_influence_radius as f64),
    );
}

fn draw_grid(scene: &mut Scene, topleft: Vector2<f64>, bottomright: Vector2<f64>, cell_size: f64) {
    for i in 0..((bottomright.x - topleft.x) / cell_size) as usize + 1 {
        scene.stroke(
            &Stroke::default(),
            Affine::IDENTITY,
            css::LIGHT_GRAY,
            None,
            &Line::new(
                (topleft.x + i as f64 * cell_size, topleft.y),
                (topleft.x + i as f64 * cell_size, bottomright.y),
            ),
        );
    }
    for j in 0..((bottomright.y - topleft.y) / cell_size) as usize + 1 {
        scene.stroke(
            &Stroke::default(),
            Affine::IDENTITY,
            css::LIGHT_GRAY,
            None,
            &Line::new(
                (topleft.x, topleft.y + j as f64 * cell_size),
                (bottomright.x, topleft.y + j as f64 * cell_size),
            ),
        )
    }
}

use std::fmt::Write;

fn draw_stats(
    scene: &mut Scene,
    text: &mut SimpleText,
    (fps, min_fps): (usize, usize),
    physics: &PhysicsEngine,
) -> anyhow::Result<()> {
    let buffer = &mut String::new();
    write_stats(buffer, (fps, min_fps), physics)?;

    const TEXT_SIZE: f32 = 16.0;
    text.add(
        scene,
        TEXT_SIZE,
        None,
        Affine::translate((0.0, TEXT_SIZE as f64)),
        buffer,
    );

    Ok(())
}

fn write_stats(buffer: &mut String, (fps, min_fps): (usize, usize), physics: &PhysicsEngine) -> anyhow::Result<()> {
    let config = config();
    writeln!(buffer, "FPS: {:?} (min {:?})", fps, min_fps)?;
    write!(buffer, "sim time: {}", physics.time())?;
    if let Some(time_limit) = config.simulation.time_limit {
        let action = config.simulation.time_limit_action.unwrap_or_default().to_string();
        write!(buffer, " ({action} at {time_limit})")?;
    }
    writeln!(buffer)?;
    writeln!(
        buffer,
        "gpu compute: {}",
        if config.simulation.enable_gpu {
            "enabled"
        } else {
            "disabled"
        }
    )?;
    writeln!(buffer, "objects: {}", physics.objects().len())?;
    let stats = physics.stats();
    write_duration_stat(buffer, "updates", &stats.updates_duration)?;
    write_duration_stat(buffer, "grid", &stats.grid_duration)?;
    write_duration_stat(buffer, "collisions", &stats.collisions_duration)?;
    write_duration_stat(buffer, "constraints", &stats.constraints_duration)?;
    write_duration_stat(buffer, "total", &stats.total_duration)?;
    Ok(())
}

fn write_duration_stat(buffer: &mut String, name: &str, stat: &DurationStat) -> anyhow::Result<()> {
    writeln!(
        buffer,
        "{}: {:?} (min {:?}, max {:?})",
        name, stat.current, stat.lowest, stat.highest
    )?;
    Ok(())
}

fn render_scene(scene: &Scene, surface: &RenderSurface, renderer: &mut Renderer, device_handle: &DeviceHandle) {
    let width = surface.config.width;
    let height = surface.config.height;
    let surface_texture = surface
        .surface
        .get_current_texture()
        .expect("failed to get surface texture");
    let render_params = RenderParams {
        base_color: css::BLACK,
        width,
        height,
        antialiasing_method: AaConfig::Area,
    };
    renderer
        .render_to_surface(
            &device_handle.device,
            &device_handle.queue,
            scene,
            &surface_texture,
            &render_params,
        )
        .expect("failed to render to surface");
    surface_texture.present();
    device_handle.device.poll(Maintain::Poll);
}

fn spectrum(position: f32) -> Color {
    Color::new([1.0 - position, (1.0 - (position - 0.5).abs() * 2.0), position, 1.0])
}
