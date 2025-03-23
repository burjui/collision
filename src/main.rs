#![allow(clippy::too_many_lines)]

use std::{
    fmt::{self, Debug, Write as _},
    iter::zip,
    mem::take,
    num::NonZero,
    ops::Add,
    sync::{Arc, Barrier, Mutex, mpsc},
    thread,
    time::{Duration, Instant},
};

use anyhow::anyhow;
use collision::{
    app_config::{CONFIG, ColorSource, TimeLimitAction},
    fps::FpsCalculator,
    physics::{ConstraintBox, DurationStat, GpuComputeOptions, PhysicsEngine, Stats},
    simple_text::SimpleText,
    vector2::Vector2,
};
use demo::create_demo;
use itertools::Itertools;
use vello::{
    AaConfig, AaSupport, RenderParams, Renderer, RendererOptions, Scene,
    kurbo::{Affine, Circle, Line, Rect, Stroke},
    peniko::{Color, Fill, color::palette::css},
    util::{DeviceHandle, RenderContext, RenderSurface},
    wgpu::{Maintain, PresentMode},
};
use winit::{
    application::ApplicationHandler,
    dpi::PhysicalSize,
    event::{ElementState, MouseButton, MouseScrollDelta, WindowEvent},
    event_loop::{ActiveEventLoop, EventLoop, EventLoopProxy},
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
    let (simulation_event_sender, simulation_event_receiver) = mpsc::channel();
    let (rendering_event_sender, rendering_event_receiver) = mpsc::channel();
    let sim_total_duration = Arc::new(Mutex::new(Duration::ZERO));
    let wait_for_exit_barrier = Arc::new(Barrier::new(3));
    let gpu_compute_options = GpuComputeOptions {
        integration: CONFIG.simulation.gpu_integration,
    };
    let rendering_start_barrier = Arc::new(Barrier::new(3));
    let simulation_thread = {
        let sim_total_duration = sim_total_duration.clone();
        let wait_for_exit_barrier = wait_for_exit_barrier.clone();
        let app_event_loop_proxy = event_loop.create_proxy();
        let rendering_event_sender = rendering_event_sender.clone();
        let rendering_start_barrier = rendering_start_barrier.clone();
        thread::spawn(move || -> PhysicsEngine {
            simulation_thread(
                &sim_total_duration,
                &app_event_loop_proxy,
                &simulation_event_receiver,
                &wait_for_exit_barrier,
                gpu_compute_options,
                &rendering_event_sender,
                &rendering_start_barrier,
            )
        })
    };
    let rendering_thread = {
        let app_event_loop_proxy = event_loop.create_proxy();
        let wait_for_exit_barrier = wait_for_exit_barrier.clone();
        let rendering_start_barrier = rendering_start_barrier.clone();
        thread::spawn(move || {
            rendering_thread(
                &rendering_event_receiver,
                &app_event_loop_proxy,
                &wait_for_exit_barrier,
                &rendering_start_barrier,
            )
        })
    };
    let mut app = VelloApp {
        context: render_context,
        renderers,
        state: render_state,
        cached_window: None,
        scene: Scene::new(),
        simulation_scene: Scene::new(),
        frame_count: 0,
        fps_calculator: FpsCalculator::default(),
        last_fps: 0,
        min_fps: usize::MAX,
        mouse_position: Vector2::new(0.0, 0.0),
        mouse_influence_radius: 50.0,
        text: SimpleText::new(),
        simulation_event_sender,
        rendering_event_sender,
        stats: Stats::default(),
        wait_for_exit_barrier,
        gpu_compute_options,
    };

    rendering_start_barrier.wait();
    let start = Instant::now();
    event_loop.run_app(&mut app).expect("run to completion");

    rendering_thread.join().expect("failed to join rendering thread");
    let physics = simulation_thread.join().expect("failed to join simulation thread");
    let mut stats_buffer = String::new();
    write_stats(
        &mut stats_buffer,
        (app.last_fps, app.min_fps),
        physics.stats(),
        app.gpu_compute_options,
    )?;
    print!("{stats_buffer}");
    let sim_total_duration_guard = sim_total_duration.lock().unwrap();
    println!("total simulation duration: {:?}", *sim_total_duration_guard);
    if *sim_total_duration_guard > Duration::ZERO {
        println!(
            "relative speed: {}",
            physics.time() / sim_total_duration_guard.as_secs_f64()
        );
    }
    println!("Total app running duration: {:?}", start.elapsed());

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

fn simulation_thread(
    sim_total_duration: &Arc<Mutex<Duration>>,
    app_event_loop_proxy: &EventLoopProxy<AppEvent>,
    simulation_event_receiver: &mpsc::Receiver<SimulationThreadEvent>,
    wait_for_exit_barrier: &Arc<Barrier>,
    mut gpu_compute_options: GpuComputeOptions,
    rendering_event_sender: &mpsc::Sender<RenderingThreadEvent>,
    rendering_start_barrier: &Arc<Barrier>,
) -> PhysicsEngine {
    let mut advance_time = CONFIG.simulation.auto_start;
    let mut time_limit_action_executed = false;
    let mut draw_grid = false;
    let mut draw_ids = false;
    let mut physics = PhysicsEngine::new().unwrap();
    let mut color_source = ColorSource::Velocity;
    create_demo(&mut physics);
    println!("{} objects", physics.objects().len());
    let mut redraw_needed = false;
    let mut last_redraw = Instant::now();
    rendering_start_barrier.wait();
    'main_loop: loop {
        fn send_app_event(event_loop_proxy: &EventLoopProxy<AppEvent>, mut event: AppEvent) {
            let _ = event_loop_proxy
                .send_event(event.take())
                .map_err(|e| eprintln!("Failed to send event {event:?}: {}", &e));
        }

        while let Result::Ok(event) = simulation_event_receiver.try_recv() {
            match event {
                SimulationThreadEvent::Exit => {
                    wait_for_exit_barrier.wait();
                    break 'main_loop;
                }
                SimulationThreadEvent::ToggleAdvanceTime => advance_time = !advance_time,
                SimulationThreadEvent::ToggleDrawIds => {
                    draw_ids = !draw_ids;
                    redraw_needed = true;
                }
                SimulationThreadEvent::ToggleDrawGrid => {
                    draw_grid = !draw_grid;
                    redraw_needed = true;
                }
                SimulationThreadEvent::ToggleColorSource => {
                    color_source = match color_source {
                        ColorSource::Demo => ColorSource::Velocity,
                        ColorSource::Velocity => ColorSource::Demo,
                    };
                    redraw_needed = true;
                }
                SimulationThreadEvent::SetGpuComputeOptions(options) => gpu_compute_options = options,
                SimulationThreadEvent::UnidirectionalKick {
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
                    redraw_needed = true;
                }
            }
        }

        if CONFIG.simulation.time_limit.is_some_and(|limit| physics.time() > limit) && !time_limit_action_executed {
            println!("Time limit reached");
            time_limit_action_executed = true;

            match CONFIG.simulation.time_limit_action {
                TimeLimitAction::Exit => {
                    send_app_event(app_event_loop_proxy, AppEvent::Exit);
                    rendering_event_sender
                        .send(RenderingThreadEvent::Exit)
                        .expect("failed to send rendering thread Exit event");
                    wait_for_exit_barrier.wait();
                    break 'main_loop;
                }
                TimeLimitAction::Pause => advance_time = false,
            }
        }

        let now = Instant::now();
        if (now - last_redraw).as_secs_f64() > 1.0 / 60.0 {
            last_redraw = now;
            redraw_needed = true;
        }
        if redraw_needed {
            redraw_needed = false;
            rendering_event_sender
                .send(RenderingThreadEvent::Draw(RenderingData {
                    positions: physics.objects().positions.clone(),
                    velocities: physics.objects().velocities.clone(),
                    radii: physics.objects().radii.clone(),
                    colors: physics.objects().colors.clone(),
                    planet_count: physics.objects().planet_count,
                    color_source,
                    draw_ids,
                    draw_grid,
                    grid_cell_size: physics.grid().cell_size(),
                    constraints: physics.constraints(),
                }))
                .expect("failed to send rendering thread SetData event");
        }

        if advance_time {
            let start = Instant::now();
            physics.advance(CONFIG.simulation.speed_factor, gpu_compute_options);
            *sim_total_duration.lock().unwrap() += start.elapsed();
            send_app_event(app_event_loop_proxy, AppEvent::StatsUpdated(physics.stats().clone()));
        }
    }

    physics
}

fn rendering_thread(
    event_receiver: &mpsc::Receiver<RenderingThreadEvent>,
    app_event_loop_proxy: &EventLoopProxy<AppEvent>,
    wait_for_exit_barrier: &Arc<Barrier>,
    rendering_start_barrier: &Arc<Barrier>,
) {
    let mut last_redraw = Instant::now();
    let mut rendering_data = RenderingData::default();
    rendering_start_barrier.wait();
    'main_loop: loop {
        while let Result::Ok(event) = event_receiver.try_recv() {
            match event {
                RenderingThreadEvent::Draw(data) => rendering_data = data,
                RenderingThreadEvent::Exit => {
                    wait_for_exit_barrier.wait();
                    break 'main_loop;
                }
            }
        }
        let now = Instant::now();
        if (now - last_redraw).as_secs_f64() > 1.0 / 60.0 {
            last_redraw = now;
            if !rendering_data.positions.is_empty() {
                let mut scenes = draw_physics(&rendering_data);
                let mut scene = scenes.remove(0);
                for subscene in scenes {
                    scene.append(&subscene, None);
                }
                if rendering_data.draw_grid && rendering_data.grid_cell_size > 0.0 {
                    draw_grid(
                        &mut scene,
                        rendering_data.constraints.topleft,
                        rendering_data.constraints.bottomright,
                        rendering_data.grid_cell_size,
                    );
                }
                app_event_loop_proxy
                    .send_event(AppEvent::SceneUpdated(scene.clone()))
                    .expect("failed to send");
            }
        }
    }
}

enum AppEvent {
    SceneUpdated(Scene),
    StatsUpdated(Stats),
    Exit,
}

impl AppEvent {
    fn take(&mut self) -> AppEvent {
        match self {
            Self::SceneUpdated(scene) => AppEvent::SceneUpdated(take(scene)),
            Self::StatsUpdated(stats) => AppEvent::StatsUpdated(take(stats)),
            Self::Exit => AppEvent::Exit,
        }
    }
}

impl Debug for AppEvent {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "AppEvent::")?;
        match self {
            Self::SceneUpdated(_) => write!(f, "SceneUpdated(...)"),
            Self::StatsUpdated(_) => write!(f, "StatsUpdated(...)"),
            Self::Exit => write!(f, "Exit"),
        }
    }
}

#[derive(Debug)]
enum SimulationThreadEvent {
    Exit,
    ToggleAdvanceTime,
    ToggleDrawIds,
    ToggleDrawGrid,
    ToggleColorSource,
    SetGpuComputeOptions(GpuComputeOptions),
    UnidirectionalKick {
        mouse_position: Vector2<f64>,
        mouse_influence_radius: f64,
    },
}

enum RenderingThreadEvent {
    Draw(RenderingData),
    Exit,
}

impl Debug for RenderingThreadEvent {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            RenderingThreadEvent::Draw(_) => "SetData(...)",
            RenderingThreadEvent::Exit => "Exit",
        })
    }
}

#[derive(Default)]
struct RenderingData {
    positions: Vec<Vector2<f64>>,
    velocities: Vec<Vector2<f64>>,
    radii: Vec<f64>,
    colors: Vec<Option<Color>>,
    planet_count: usize,
    color_source: ColorSource,
    draw_ids: bool,
    draw_grid: bool,
    grid_cell_size: f64,
    constraints: ConstraintBox,
}

struct VelloApp<'s> {
    context: RenderContext,
    renderers: Vec<Option<Renderer>>,
    state: Option<RenderState<'s>>,
    // Whilst suspended, we drop `render_state`, but need to keep the same window.
    // If render_state exists, we must store the window in it, to maintain drop order
    cached_window: Option<Arc<Window>>,
    scene: Scene,
    simulation_scene: Scene,
    frame_count: usize,
    fps_calculator: FpsCalculator,
    last_fps: usize,
    min_fps: usize,
    mouse_position: Vector2<f64>,
    mouse_influence_radius: f64,
    text: SimpleText,
    simulation_event_sender: mpsc::Sender<SimulationThreadEvent>,
    rendering_event_sender: mpsc::Sender<RenderingThreadEvent>,
    stats: Stats,
    wait_for_exit_barrier: Arc<Barrier>,
    gpu_compute_options: GpuComputeOptions,
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
                            .with_inner_size(PhysicalSize::new(CONFIG.window.width, CONFIG.window.height))
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
        let surface = pollster::block_on(surface_future).expect("failed to create surface");
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
                .expect("failed to create renderer")
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
                            self.simulation_event_sender
                                .send(SimulationThreadEvent::Exit)
                                .expect("failed to send simulation thread Exit event");
                            self.rendering_event_sender
                                .send(RenderingThreadEvent::Exit)
                                .expect("failed to send rendering thread Exit event");
                            self.wait_for_exit_barrier.wait();
                            event_loop.exit();
                        }
                        Key::Named(NamedKey::Space) => {
                            self.simulation_event_sender
                                .send(SimulationThreadEvent::ToggleAdvanceTime)
                                .unwrap();
                        }
                        Key::Character("g") => {
                            self.simulation_event_sender
                                .send(SimulationThreadEvent::ToggleDrawGrid)
                                .unwrap();
                        }
                        Key::Character("i") => {
                            self.simulation_event_sender
                                .send(SimulationThreadEvent::ToggleDrawIds)
                                .unwrap();
                        }
                        Key::Character("v") => {
                            self.simulation_event_sender
                                .send(SimulationThreadEvent::ToggleColorSource)
                                .unwrap();
                        }
                        Key::Character("a") => {
                            self.gpu_compute_options.integration = !self.gpu_compute_options.integration;
                            self.simulation_event_sender
                                .send(SimulationThreadEvent::SetGpuComputeOptions(self.gpu_compute_options))
                                .unwrap();
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
                    self.scene.append(&self.simulation_scene, None);
                    draw_mouse_influence(&mut self.scene, self.mouse_position, self.mouse_influence_radius);
                    draw_stats(
                        &mut self.scene,
                        &mut self.text,
                        (self.last_fps, self.min_fps),
                        &self.stats,
                        self.gpu_compute_options,
                    )
                    .expect("failed to draw stats");

                    let renderer = self.renderers[surface.dev_id].as_mut().expect("failed to get renderer");
                    let device_handle = &self.context.devices[surface.dev_id];
                    render_scene(&self.scene, surface, renderer, device_handle);
                    self.frame_count += 1;
                }
            }
            WindowEvent::CursorMoved { position, .. } => {
                self.mouse_position = Vector2::new(position.x, position.y);
                request_redraw(self.state.as_ref());
            }
            WindowEvent::MouseInput { state, button, .. } => {
                if state == ElementState::Pressed {
                    if let MouseButton::Left = button {
                        self.simulation_event_sender
                            .send(SimulationThreadEvent::UnidirectionalKick {
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
                self.mouse_influence_radius = (self.mouse_influence_radius + f64::from(dy) * 3.0).max(0.0);
                request_redraw(self.state.as_ref());
            }
            _ => {}
        }
    }

    fn user_event(&mut self, event_loop: &ActiveEventLoop, event: AppEvent) {
        match event {
            AppEvent::SceneUpdated(scene) => {
                self.simulation_scene = scene;
                request_redraw(self.state.as_ref());
            }
            AppEvent::StatsUpdated(stats) => {
                self.stats = stats;
                request_redraw(self.state.as_ref());
            }
            AppEvent::Exit => {
                self.wait_for_exit_barrier.wait();
                event_loop.exit();
            }
        }
    }

    fn suspended(&mut self, _: &ActiveEventLoop) {
        // When we suspend, we need to remove the `wgpu` Surface
        if let Some(render_state) = self.state.take() {
            self.cached_window = Some(render_state.window);
        }
    }
}

struct RenderState<'s> {
    // SAFETY: We MUST drop the surface before the `window`, so the fields
    // must be in this order
    surface: RenderSurface<'s>,
    window: Arc<Window>,
}

fn request_redraw(render_state: Option<&RenderState<'_>>) {
    if let Some(render_state) = render_state {
        render_state.window.request_redraw();
    }
}

fn draw_physics(
    RenderingData {
        positions,
        velocities,
        radii,
        colors,
        planet_count,
        color_source,
        draw_ids,
        constraints,
        ..
    }: &RenderingData,
) -> Vec<Scene> {
    fn draw_circle(scene: &mut Scene, transform: Affine, position: Vector2<f64>, radius: f64, color: Color) {
        scene.fill(
            Fill::NonZero,
            transform,
            color,
            None,
            &Circle::new((position.x, position.y), radius.max(1.0)),
        );
    }

    fn draw_text(scene: &mut Scene, transform: Affine, text: &mut SimpleText, position: Vector2<f64>, s: &str) {
        text.add(
            scene,
            10.0,
            None,
            Affine::translate((position.x, position.y)) * transform,
            s,
        );
    }

    let transform = Affine::IDENTITY;
    let chunk_size = positions.len().div_ceil(16);
    let chunks = (*planet_count..positions.len())
        .chunks(if chunk_size > 0 { chunk_size } else { positions.len() })
        .into_iter()
        .map(Itertools::collect_vec)
        .collect_vec();
    let mut scenes = std::thread::scope(|scope| {
        chunks
            .iter()
            .map(|chunk| {
                scope.spawn(move || {
                    let mut scene = Scene::new();
                    let mut text = SimpleText::new();
                    for &object_index in chunk {
                        let particle_position = positions[object_index];
                        let particle_radius = radii[object_index];
                        let color = match color_source {
                            ColorSource::Demo => colors[object_index],
                            ColorSource::Velocity => {
                                const SCALE_FACTOR: f64 = 0.0004;
                                let velocity = velocities[object_index];
                                #[allow(clippy::cast_possible_truncation)]
                                let spectrum_position =
                                    (velocity.magnitude() * SCALE_FACTOR).powf(0.6).clamp(0.0, 1.0) as f32;
                                Some(spectrum(spectrum_position))
                            }
                        }
                        .unwrap_or(css::GRAY);
                        draw_circle(
                            &mut scene,
                            transform,
                            particle_position,
                            particle_radius.max(1.0),
                            color,
                        );
                        if *draw_ids {
                            draw_text(
                                &mut scene,
                                transform,
                                &mut text,
                                particle_position,
                                &format!("{object_index}"),
                            );
                        }
                    }
                    scene
                })
            })
            .map(|handle| handle.join().unwrap())
            .collect_vec()
    });

    let scene = scenes.last_mut().unwrap();
    let mut text = SimpleText::new();
    for (object_index, ((&planet_position, &planet_radius), color)) in zip(
        zip(&positions[..*planet_count], &radii[..*planet_count]),
        &colors[..*planet_count],
    )
    .enumerate()
    {
        draw_circle(
            scene,
            transform,
            planet_position,
            planet_radius.max(1.0),
            color.unwrap_or(css::WHITE),
        );
        if *draw_ids {
            draw_text(scene, transform, &mut text, planet_position, &format!("{object_index}"));
        }
    }

    let topleft = constraints.topleft;
    let bottomright = constraints.bottomright;
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
    #![allow(clippy::cast_possible_truncation)]
    #![allow(clippy::cast_precision_loss)]
    #![allow(clippy::cast_sign_loss)]
    assert!(bottomright.x >= topleft.x);
    for i in 0..=((bottomright.x - topleft.x) / cell_size) as usize {
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
    assert!(bottomright.y >= topleft.y);
    for j in 0..=((bottomright.y - topleft.y) / cell_size) as usize {
        scene.stroke(
            &Stroke::default(),
            Affine::IDENTITY,
            css::LIGHT_GRAY,
            None,
            &Line::new(
                (topleft.x, topleft.y + j as f64 * cell_size),
                (bottomright.x, topleft.y + j as f64 * cell_size),
            ),
        );
    }
}

fn draw_stats(
    scene: &mut Scene,
    text: &mut SimpleText,
    (fps, min_fps): (usize, usize),
    stats: &Stats,
    gpu_compute_options: GpuComputeOptions,
) -> anyhow::Result<()> {
    const TEXT_SIZE: f32 = 16.0;

    let buffer = &mut String::new();
    write_stats(buffer, (fps, min_fps), stats, gpu_compute_options)?;
    text.add(
        scene,
        TEXT_SIZE,
        None,
        Affine::translate((0.0, f64::from(TEXT_SIZE))),
        buffer,
    );

    Ok(())
}

fn write_stats(
    buffer: &mut String,
    (fps, min_fps): (usize, usize),
    stats: &Stats,
    gpu_compute_options: GpuComputeOptions,
) -> anyhow::Result<()> {
    writeln!(buffer, "FPS: {fps} (min {min_fps})")?;
    write!(buffer, "sim time: {}", stats.sim_time)?;
    if let Some(time_limit) = CONFIG.simulation.time_limit {
        let action = CONFIG.simulation.time_limit_action.to_string();
        write!(buffer, " ({action} at {time_limit})")?;
    }
    writeln!(buffer)?;
    writeln!(
        buffer,
        "gpu compute: {}",
        if gpu_compute_options.integration {
            "integration"
        } else {
            "off"
        }
    )?;
    writeln!(buffer, "objects: {}", stats.object_count)?;
    write_duration_stat(buffer, "integration", &stats.integration_duration)?;
    write_duration_stat(buffer, "collisions", &stats.collisions_duration)?;
    write_duration_stat(buffer, "grid", &stats.grid_duration)?;
    write_duration_stat(buffer, "constraints", &stats.constraints_duration)?;
    Ok(())
}

fn write_duration_stat(buffer: &mut String, name: &str, stat: &DurationStat) -> anyhow::Result<()> {
    let average = if stat.average.is_empty() {
        Duration::MAX
    } else {
        let sum = stat.average.clone().fold(Duration::ZERO, Add::add);
        let count = u32::try_from(stat.average.len()).unwrap();
        sum / count
    };
    writeln!(
        buffer,
        "{}: {:?} (min {:?}, max {:?}, avg {:?})",
        name, stat.current, stat.lowest, stat.highest, average
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
