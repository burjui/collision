use core::f64;
use std::{
    fmt::{Debug, Write as _},
    iter::zip,
    mem::replace,
    num::NonZero,
    sync::{mpsc, Arc, Mutex},
    thread::{self},
    time::{Duration, Instant},
};

use anyhow::{anyhow, Ok};
use collision::{
    app_config::{config, config_mut, ColorSource, TimeLimitAction},
    fps::FpsCalculator,
    physics::{DurationStat, PhysicsEngine, Stats},
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
    let event_loop = EventLoop::with_user_event().build()?;
    let render_context = RenderContext::new();
    let renderers: Vec<Option<Renderer>> = vec![];
    let render_state = None::<RenderState<'_>>;
    let config = config();
    let (physics_thread_sender, physics_event_receiver) = mpsc::channel();
    let sim_total_duration = Arc::new(Mutex::new(Duration::ZERO));
    let physics_thread = {
        let mut physics = PhysicsEngine::new()?;
        create_demo(&mut physics);
        println!("{} objects", physics.objects().len());

        let sim_total_duration = sim_total_duration.clone();
        let event_loop_proxy = event_loop.create_proxy();
        let mut advance_time = config.simulation.auto_start;
        let mut time_limit_action_executed = false;
        let mut jerk_applied = false;
        let mut last_redraw = Instant::now();
        let mut waiting_for_redraw = false;
        let mut show_grid = false;
        let mut show_ids = false;
        let mut waiting_for_exit = false;
        thread::spawn(move || {
            'main_loop: loop {
                fn send_event(event_loop_proxy: &winit::event_loop::EventLoopProxy<AppEvent>, mut event: AppEvent) {
                    let _ = event_loop_proxy
                        .send_event(event.take())
                        .map_err(|e| eprintln!("Failed to send event {event:?}: {}", &e));
                }

                if waiting_for_exit {
                    if let Result::Ok(PhysicsThreadEvent::Exit) = physics_event_receiver.try_recv() {
                        break 'main_loop;
                    } else {
                        continue;
                    }
                }

                while let Result::Ok(event) = physics_event_receiver.try_recv() {
                    match event {
                        PhysicsThreadEvent::ToggleAdvanceTime => {
                            advance_time = !advance_time;
                        }
                        PhysicsThreadEvent::ToggleShowIds => {
                            show_ids = !show_ids;
                        }
                        PhysicsThreadEvent::ToggleShowGrid => {
                            show_grid = !show_grid;
                        }
                        PhysicsThreadEvent::UnidirectionalKick {
                            mouse_position,
                            mouse_influence_radius,
                        } => {
                            let objects = physics.objects_mut();
                            for (&position, velocity) in zip(&objects.positions, &mut objects.velocities) {
                                let from_mouse_to_object = position - mouse_position;
                                if (from_mouse_to_object).magnitude() < mouse_influence_radius {
                                    *velocity += from_mouse_to_object.normalize() * 2000.0;
                                }
                            }
                            send_event(&event_loop_proxy, AppEvent::RequestRedraw);
                        }
                        PhysicsThreadEvent::RedrawComplete => {
                            waiting_for_redraw = false;
                        }
                        PhysicsThreadEvent::WaitForExit => {
                            waiting_for_exit = true;
                            send_event(&event_loop_proxy, AppEvent::PhysicsThreadWaitingForExit);
                            continue 'main_loop;
                        }
                        PhysicsThreadEvent::Exit => break 'main_loop,
                    }
                }

                if config.simulation.time_limit.is_some_and(|limit| physics.time() > limit)
                    && !time_limit_action_executed
                {
                    println!("Time limit reached");
                    time_limit_action_executed = true;
                    advance_time = false;
                    match config.simulation.time_limit_action {
                        TimeLimitAction::Exit => {
                            waiting_for_exit = true;
                            send_event(&event_loop_proxy, AppEvent::PhysicsThreadWaitingForExit);
                            continue;
                        }
                        TimeLimitAction::Pause => send_event(&event_loop_proxy, AppEvent::RequestRedraw),
                    }
                }

                let now = Instant::now();
                if (now - last_redraw).as_secs_f64() > 1.0 / 60.0 && !waiting_for_redraw {
                    last_redraw = now;
                    let mut scenes = draw_physics(&physics, DrawIds(show_ids));
                    let mut scene = scenes.remove(0);
                    for subscene in scenes {
                        scene.append(&subscene, None);
                    }
                    if show_grid && physics.grid().cell_size() > 0.0 {
                        draw_grid(
                            &mut scene,
                            physics.constraints().topleft,
                            physics.constraints().bottomright,
                            physics.grid().cell_size(),
                        );
                    }
                    send_event(&event_loop_proxy, AppEvent::RequestRedrawPhysics(scene));
                }

                if advance_time {
                    let start = Instant::now();
                    physics.advance(config.simulation.speed_factor);
                    *sim_total_duration.lock().unwrap() += start.elapsed();
                    if config
                        .simulation
                        .jerk_at
                        .is_some_and(|jerk_at| physics.time() >= jerk_at)
                        && !jerk_applied
                    {
                        jerk_applied = true;
                        let first_non_planet = physics.planet_count();
                        physics.objects_mut().velocities[first_non_planet] += Vector2::new(1000.0, 1000.0);
                    }
                    send_event(&event_loop_proxy, AppEvent::StatsUpdated(*physics.stats()));
                }
            }
            physics
        })
    };

    let mut app = VelloApp {
        context: render_context,
        renderers,
        state: render_state,
        cached_window: None,
        scene: Scene::new(),
        physics_scene: Scene::new(),
        frame_count: 0,
        fps_calculator: FpsCalculator::default(),
        last_fps: 0,
        min_fps: usize::MAX,
        mouse_position: Vector2::new(0.0, 0.0),
        mouse_influence_radius: 50.0,
        text: SimpleText::new(),
        physics_thread_sender,
        redraw_physics: true,
        stats: Stats::default(),
    };
    event_loop.run_app(&mut app).expect("run to completion");

    app.physics_thread_sender
        .send(PhysicsThreadEvent::Exit)
        .expect("failed to send exit event to physics thread");
    let physics = physics_thread.join().expect("failed to join physics thread");
    let mut stats_buffer = String::new();
    write_stats(&mut stats_buffer, (app.last_fps, app.min_fps), physics.stats())?;
    print!("{}", stats_buffer);
    let sim_total_duration_guard = sim_total_duration.lock().unwrap();
    println!("total simulation duration: {:?}", *sim_total_duration_guard);
    if *sim_total_duration_guard > Duration::ZERO {
        println!(
            "relative speed: {}",
            physics.time() / sim_total_duration_guard.as_secs_f64()
        );
    }

    Ok(())
}

enum AppEvent {
    RequestRedraw,
    RequestRedrawPhysics(Scene),
    StatsUpdated(Stats),
    PhysicsThreadWaitingForExit,
    Exit,
}

impl AppEvent {
    fn take(&mut self) -> AppEvent {
        match self {
            Self::RequestRedraw => AppEvent::RequestRedraw,
            Self::RequestRedrawPhysics(scene) => AppEvent::RequestRedrawPhysics(replace(scene, Scene::new())),
            Self::StatsUpdated(stats) => AppEvent::StatsUpdated(replace(stats, Stats::default())),
            Self::PhysicsThreadWaitingForExit => AppEvent::PhysicsThreadWaitingForExit,
            Self::Exit => AppEvent::Exit,
        }
    }
}

impl Debug for AppEvent {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "AppEvent::")?;
        match self {
            Self::RequestRedraw => write!(f, "RequestRedraw"),
            Self::RequestRedrawPhysics(_) => write!(f, "RequestRedrawPhysics"),
            Self::StatsUpdated(_) => write!(f, "StatsUpdated"),
            Self::PhysicsThreadWaitingForExit => write!(f, "PhysicsThreadWaitingForExit"),
            Self::Exit => write!(f, "Exit"),
        }
    }
}

enum PhysicsThreadEvent {
    ToggleAdvanceTime,
    UnidirectionalKick {
        mouse_position: Vector2<f64>,
        mouse_influence_radius: f64,
    },
    RedrawComplete,
    WaitForExit,
    Exit,
    ToggleShowIds,
    ToggleShowGrid,
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
    physics_scene: Scene,
    frame_count: usize,
    fps_calculator: FpsCalculator,
    last_fps: usize,
    min_fps: usize,
    mouse_position: Vector2<f64>,
    mouse_influence_radius: f64,
    text: SimpleText,
    physics_thread_sender: mpsc::Sender<PhysicsThreadEvent>,
    redraw_physics: bool,
    stats: Stats,
}

impl ApplicationHandler<AppEvent> for VelloApp<'_> {
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
                        Key::Named(NamedKey::Escape) => {
                            self.physics_thread_sender
                                .send(PhysicsThreadEvent::WaitForExit)
                                .unwrap();
                        }
                        Key::Named(NamedKey::Space) => {
                            self.physics_thread_sender
                                .send(PhysicsThreadEvent::ToggleAdvanceTime)
                                .unwrap();
                        }
                        Key::Character("a") => {
                            config_mut().simulation.enable_gpu = !config().simulation.enable_gpu;
                            request_redraw(&self.state);
                        }
                        Key::Character("g") => {
                            self.physics_thread_sender
                                .send(PhysicsThreadEvent::ToggleShowGrid)
                                .unwrap();
                            // request_redraw(&self.state);
                        }
                        Key::Character("i") => {
                            self.physics_thread_sender
                                .send(PhysicsThreadEvent::ToggleShowIds)
                                .unwrap();
                            // request_redraw(&self.state);
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
                    if let Some(fps) = self.fps_calculator.update(self.frame_count) {
                        self.last_fps = fps;
                        self.min_fps = self.min_fps.min(fps);
                    }
                    self.scene.reset();
                    self.scene.append(&self.physics_scene, None);
                    draw_mouse_influence(&mut self.scene, self.mouse_position, self.mouse_influence_radius);
                    draw_stats(
                        &mut self.scene,
                        &mut self.text,
                        (self.last_fps, self.min_fps),
                        &self.stats,
                    )
                    .expect("failed to draw stats");

                    let renderer = self.renderers[surface.dev_id].as_mut().expect("failed to get renderer");
                    let device_handle = &self.context.devices[surface.dev_id];
                    render_scene(&self.scene, surface, renderer, device_handle);

                    self.frame_count += 1;
                    let _ = self
                        .physics_thread_sender
                        .send(PhysicsThreadEvent::RedrawComplete)
                        .map_err(|e| eprintln!("Failed to send redraw complete: {e}"));
                }
            }
            WindowEvent::CursorMoved { position, .. } => {
                self.mouse_position = Vector2::new(position.x, position.y);
                request_redraw(&self.state);
            }
            WindowEvent::MouseInput { state, button, .. } => {
                if state == ElementState::Pressed {
                    if let MouseButton::Left = button {
                        self.physics_thread_sender
                            .send(PhysicsThreadEvent::UnidirectionalKick {
                                mouse_position: self.mouse_position,
                                mouse_influence_radius: self.mouse_influence_radius,
                            })
                            .unwrap();
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

    fn user_event(&mut self, event_loop: &ActiveEventLoop, event: AppEvent) {
        match event {
            AppEvent::RequestRedraw => {
                request_redraw(&self.state);
            }
            AppEvent::RequestRedrawPhysics(scene) => {
                self.redraw_physics = true;
                self.physics_scene = scene;
                request_redraw(&self.state);
            }
            AppEvent::StatsUpdated(stats) => {
                self.stats = stats;
                request_redraw(&self.state);
            }
            AppEvent::PhysicsThreadWaitingForExit => event_loop.exit(),
            AppEvent::Exit => self.physics_thread_sender.send(PhysicsThreadEvent::Exit).unwrap(),
        }
    }

    fn suspended(&mut self, _: &ActiveEventLoop) {
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

fn draw_physics(physics: &PhysicsEngine, DrawIds(draw_ids): DrawIds) -> Vec<Scene> {
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
    let mut scenes = std::thread::scope(|scope| {
        chunks
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
                                &Circle::new((particle_position.x, particle_position.y), particle_radius.max(1.0)),
                            );
                            if draw_ids {
                                text.add(
                                    &mut scene,
                                    10.0,
                                    None,
                                    Affine::translate((particle_position.x, particle_position.y)),
                                    &format!("{}", object_index),
                                );
                            }
                        }
                    }
                    scene
                })
            })
            .map(|handle| handle.join().unwrap())
            .collect_vec()
    });

    let scene = scenes.last_mut().unwrap();
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
            &Circle::new((planet_position.x, planet_position.y), planet_radius.max(1.0)),
        );
    }

    let topleft = physics.constraints().topleft;
    let bottomright = physics.constraints().bottomright;
    scene.stroke(
        &Stroke::default(),
        transform,
        css::WHITE,
        None,
        &Rect::new(topleft.x, topleft.y, bottomright.x, bottomright.y),
    );

    scenes
}

fn draw_mouse_influence(scene: &mut Scene, mouse_position: Vector2<f64>, mouse_influence_radius: f64) {
    scene.fill(
        Fill::NonZero,
        Affine::IDENTITY,
        Color::new([1.0, 1.0, 1.0, 0.3]),
        None,
        &Circle::new((mouse_position.x, mouse_position.y), mouse_influence_radius),
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

fn draw_stats(
    scene: &mut Scene,
    text: &mut SimpleText,
    (fps, min_fps): (usize, usize),
    stats: &Stats,
) -> anyhow::Result<()> {
    let buffer = &mut String::new();
    write_stats(buffer, (fps, min_fps), stats)?;

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

fn write_stats(buffer: &mut String, (fps, min_fps): (usize, usize), stats: &Stats) -> anyhow::Result<()> {
    let config = config();
    writeln!(buffer, "FPS: {:?} (min {:?})", fps, min_fps)?;
    write!(buffer, "sim time: {}", stats.sim_time)?;
    if let Some(time_limit) = config.simulation.time_limit {
        let action = config.simulation.time_limit_action.to_string();
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
    writeln!(buffer, "objects: {}", stats.object_count)?;
    write_duration_stat(buffer, "updates", &stats.updates_duration)?;
    write_duration_stat(buffer, "grid", &stats.grid_duration)?;
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
    let render_params = RenderParams {
        base_color: Color::new([1.0, 1.0, 1.0, 0.0]),
        width,
        height,
        antialiasing_method: AaConfig::Area,
    };
    let surface_texture = surface
        .surface
        .get_current_texture()
        .expect("failed to get current surface texture");
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
