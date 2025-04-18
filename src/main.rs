#![allow(clippy::too_many_lines)]
#![allow(clippy::too_many_arguments)]
#![allow(clippy::large_enum_variant)]
#![feature(let_chains)]

use std::{
    fmt::{self, Debug, Write},
    iter::zip,
    num::NonZero,
    ops::{Add, Range},
    sync::{Arc, Barrier, Mutex, mpsc},
    thread::{self, yield_now},
    time::{Duration, Instant},
};

use anyhow::anyhow;
use collision::{
    app_config::{CONFIG, ColorSource, TimeLimitAction},
    array2::Array2,
    bvh::{AABB, Bvh, Node, NodeKind},
    demo::create_demo,
    fps::FpsCalculator,
    physics::{DurationStat, GpuComputeOptions, PhysicsEngine, Stats},
    simple_text::SimpleText,
    vector2::Vector2,
};
use crossbeam::queue::{ArrayQueue, SegQueue};
use itertools::Itertools;
use rayon::{
    ThreadPoolBuilder,
    iter::{IndexedParallelIterator, IntoParallelRefIterator, IntoParallelRefMutIterator, ParallelIterator},
};
use vello::{
    AaConfig, AaSupport, RenderParams, Renderer, RendererOptions, Scene,
    kurbo::{Affine, Circle, Rect, Stroke},
    peniko::{Blob, Color, Fill, Image, ImageFormat, color::palette::css},
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

pub fn main() -> anyhow::Result<()> {
    enable_floating_point_exceptions();
    let event_loop = EventLoop::with_user_event().build()?;
    let (simulation_event_sender, simulation_event_receiver) = mpsc::channel();
    let rendering_event_queue = Box::leak(Box::new(SegQueue::new()));
    let rendering_event_queue = &*rendering_event_queue;
    let sim_total_duration = Arc::new(Mutex::new(Duration::ZERO));
    let ready_to_exit = Arc::new(Barrier::new(3));
    let gpu_compute_options = GpuComputeOptions {
        integration: CONFIG.simulation.gpu_integration,
        bvh: CONFIG.simulation.gpu_bvh,
    };
    let rendering_thread_ready = Arc::new(Barrier::new(3));
    let (rendering_result_sender, rendering_result_receiver) = mpsc::channel();
    let simulation_thread = {
        let sim_total_duration = sim_total_duration.clone();
        let ready_to_exit = ready_to_exit.clone();
        let app_event_loop_proxy = event_loop.create_proxy();
        let rendering_thread_ready = rendering_thread_ready.clone();
        thread::spawn(move || -> PhysicsEngine {
            simulation_thread(
                &sim_total_duration,
                &app_event_loop_proxy,
                &simulation_event_receiver,
                &ready_to_exit,
                gpu_compute_options,
                rendering_event_queue,
                &rendering_thread_ready,
                &rendering_result_receiver,
            )
        })
    };
    let redraw_job_queue = Box::leak(Box::new(ArrayQueue::new(1)));
    let redraw_job_queue = &*redraw_job_queue;
    let redraw_result_queue = Box::leak(Box::new(ArrayQueue::new(1)));
    let redraw_result_queue = &*redraw_result_queue;
    let rendering_thread = {
        let ready_to_exit = ready_to_exit.clone();
        let rendering_thread_ready = rendering_thread_ready.clone();
        let app_event_loop_proxy = event_loop.create_proxy();
        thread::spawn(move || {
            rendering_thread(
                rendering_event_queue,
                &ready_to_exit,
                &rendering_thread_ready,
                &rendering_result_sender,
                redraw_job_queue,
                app_event_loop_proxy,
            );
        })
    };
    let render_context = RenderContext::new();
    let renderers: Vec<Option<Renderer>> = vec![];
    let render_state = None::<RenderState<'_>>;
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
        rendering_event_queue,
        stats: Stats::default(),
        ready_to_exit,
        gpu_compute_options,
        redraw_job_queue,
        redraw_result_queue,
    };

    rendering_thread_ready.wait();
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
    ready_to_exit: &Arc<Barrier>,
    mut gpu_compute_options: GpuComputeOptions,
    rendering_event_queue: &SegQueue<RenderingThreadEvent>,
    rendering_thread_ready: &Arc<Barrier>,
    rendering_result_receiver: &mpsc::Receiver<()>,
) -> PhysicsEngine {
    const EDF_CELL_SIZE: f64 = 4.0;
    const EDF_SAMPLING_AREA_SIZE: usize = 3;

    let energy_field_job_queue = Box::leak(Box::new(ArrayQueue::new(1)));
    let edf_job_queue = &*energy_field_job_queue;
    let edf_result_queue = Box::leak(Box::new(ArrayQueue::new(1)));
    let edf_result_queue = &*edf_result_queue;
    let edf_ready = Arc::new(Barrier::new(2));

    {
        let edf_thread_ready = edf_ready.clone();
        thread::spawn(move || energy_density_field_thread(edf_thread_ready, edf_job_queue, edf_result_queue));
    }

    let mut advance_time = CONFIG.simulation.auto_start;
    let mut time_limit_action_executed = false;
    let mut draw_grid = false;
    let mut draw_ids = false;
    let mut physics = PhysicsEngine::new().unwrap();
    let mut color_source = CONFIG.rendering.color_source;
    create_demo(&mut physics);
    println!("{} objects", physics.objects().len());
    let mut redraw_needed = false;
    let mut last_redraw_instant = Instant::now();
    let mut show_edf = CONFIG.rendering.show_edf;
    rendering_thread_ready.wait();
    edf_ready.wait();
    let mut first_redraw = true;
    let mut edf = Array2::default();
    'main_loop: loop {
        fn send_app_event(event_loop_proxy: &EventLoopProxy<AppEvent>, event: AppEvent) {
            let _ = event_loop_proxy
                .send_event(event.clone())
                .map_err(|e| eprintln!("Failed to send event {event:?}: {}", &e));
        }

        while let Result::Ok(event) = simulation_event_receiver.try_recv() {
            match event {
                SimulationThreadEvent::Exit => {
                    ready_to_exit.wait();
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
                SimulationThreadEvent::SetColorSource(source) => {
                    color_source = source;
                    redraw_needed = true;
                }
                SimulationThreadEvent::ToggleDrawEdf => {
                    show_edf = !show_edf;
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
                    rendering_event_queue.push(RenderingThreadEvent::Exit);
                    ready_to_exit.wait();
                    break 'main_loop;
                }
                TimeLimitAction::Pause => advance_time = false,
            }
        }

        if let Some(new_edf) = edf_result_queue.pop() {
            edf = new_edf;
        }
        if show_edf && edf_job_queue.is_empty() {
            edf_job_queue
                .push(EnergyDensityFieldJob {
                    positions: physics.objects().positions.clone(),
                    velocities: physics.objects().velocities.clone(),
                    radii: physics.objects().radii.clone(),
                    masses: physics.objects().radii.clone(),
                    cell_size: EDF_CELL_SIZE,
                    sampling_area_size: EDF_SAMPLING_AREA_SIZE,
                })
                .map_err(|_| anyhow!("failed to send edf job"))
                .unwrap();
        }

        if advance_time {
            let start = Instant::now();
            physics.advance(CONFIG.simulation.speed_factor, gpu_compute_options);
            *sim_total_duration.lock().unwrap() += start.elapsed();
            send_app_event(app_event_loop_proxy, AppEvent::StatsUpdated(physics.stats().clone()));
        }

        let render_result = rendering_result_receiver.try_recv();
        if first_redraw || redraw_needed || render_result.is_ok() {
            first_redraw = false;
            let previous_redraw_instant = last_redraw_instant;
            last_redraw_instant = Instant::now();
            println!("redraw took {:.2?}", last_redraw_instant - previous_redraw_instant);
            if rendering_event_queue.is_empty() {
                redraw_needed = false;
                rendering_event_queue.push(RenderingThreadEvent::Draw(RenderingData {
                    positions: physics.objects().positions.clone(),
                    velocities: physics.objects().velocities.clone(),
                    radii: physics.objects().radii.clone(),
                    colors: physics.objects().colors.clone(),
                    particle_range: physics.objects().particle_range(),
                    planet_range: physics.objects().planet_range(),
                    color_source,
                    draw_ids,
                    draw_grid,
                    constraints: physics.constraints(),
                    draw_edf: show_edf,
                    edf: edf.clone(),
                    edf_cell_size: EDF_CELL_SIZE,
                    bvh: physics.bvh().clone(),
                }));
            }
        }
    }

    physics
}

struct EnergyDensityFieldJob {
    positions: Vec<Vector2<f64>>,
    velocities: Vec<Vector2<f64>>,
    radii: Vec<f64>,
    masses: Vec<f64>,
    cell_size: f64,
    sampling_area_size: usize,
}

fn energy_density_field_thread(
    edf_thread_ready: Arc<Barrier>,
    energy_field_jobs: &ArrayQueue<EnergyDensityFieldJob>,
    energy_field_result: &ArrayQueue<Array2<f64>>,
) {
    edf_thread_ready.wait();
    let mut edf = Array2::<f64>::default();
    let mut edf_avg = Array2::<f64>::default();
    let thread_pool = ThreadPoolBuilder::new().num_threads(num_cpus::get()).build().unwrap();
    loop {
        if let Some(EnergyDensityFieldJob {
            positions,
            velocities,
            radii,
            masses,
            cell_size,
            sampling_area_size,
        }) = energy_field_jobs.pop()
        {
            assert!(cell_size > 1.0);
            let start = Instant::now();
            let width = (CONFIG.window.width as f64 / cell_size) as usize + 1;
            let height = (CONFIG.window.height as f64 / cell_size) as usize + 1;
            edf.reset((width, height));
            for object_index in 0..positions.len() {
                let position = positions[object_index];
                let edf_position = position / cell_size;
                let velocity = velocities[object_index];
                let radius = radii[object_index];
                let mass = masses[object_index];
                let energy_contribution = radius * 0.5 * mass * velocity.magnitude_squared();
                edf[(edf_position.x as usize, edf_position.y as usize)] += energy_contribution;
            }
            let mut sat = edf.clone(); // summed-area table
            for y in 0..height {
                for x in 0..width {
                    let x_minus_1 = x.checked_sub(1);
                    let y_minus_1 = y.checked_sub(1);
                    sat[(x, y)] = edf[(x, y)]
                        + y_minus_1.map(|y| sat[(x, y)]).unwrap_or(0.0)
                        + x_minus_1.map(|x| sat[(x, y)]).unwrap_or(0.0)
                        - x_minus_1.and_then(|x| y_minus_1.map(|y| sat[(x, y)])).unwrap_or(0.0);
                }
            }

            edf.reset((width, height));
            for y in 0..height {
                for x in 0..width {
                    let x1 = x.saturating_sub(sampling_area_size);
                    let x2 = x.saturating_add(sampling_area_size).min(width - 1);
                    let y1 = y.saturating_sub(sampling_area_size);
                    let y2 = y.saturating_add(sampling_area_size).min(height - 1);
                    edf[(x, y)] = sat[(x1, y1)] + sat[(x2, y2)] - sat[(x1, y2)] - sat[(x2, y1)];
                }
            }

            edf_avg.reset((width, height));
            thread_pool.install(|| {
                edf_avg.data_mut().par_iter_mut().enumerate().for_each(|(i, avg)| {
                    let x = i % width;
                    let y = i / width;
                    let mut count = 0.0_f64;
                    for j in y.saturating_sub(sampling_area_size)..y.saturating_add(sampling_area_size).min(height - 1)
                    {
                        for i in
                            x.saturating_sub(sampling_area_size)..x.saturating_add(sampling_area_size).min(width - 1)
                        {
                            *avg += edf[(i, j)];
                            count += 1.0;
                        }
                    }
                    *avg /= count;
                });
                let max_energy_density = edf_avg
                    .data()
                    .par_iter()
                    .max_by(|a, b| a.total_cmp(b))
                    .unwrap()
                    .max(1.0);
                edf_avg.data_mut().par_iter_mut().for_each(|energy_density| {
                    *energy_density = (*energy_density / max_energy_density).max(0.0);
                });
            });

            println!("edf calculation took {:.2?}", start.elapsed());
            energy_field_result.force_push(edf_avg.clone());
        }
        yield_now();
    }
}

fn rendering_thread(
    rendering_event_queue: &SegQueue<RenderingThreadEvent>,
    ready_to_exit: &Arc<Barrier>,
    rendering_thread_ready: &Arc<Barrier>,
    rendering_result_queue: &mpsc::Sender<()>,
    redraw_job_queue: &ArrayQueue<Scene>,
    app_event_loop_proxy: EventLoopProxy<AppEvent>,
) {
    let mut rendering_data = RenderingData::default();
    rendering_thread_ready.wait();
    'main_loop: loop {
        while let Some(event) = rendering_event_queue.pop() {
            match event {
                RenderingThreadEvent::Draw(data) => rendering_data = data,
                RenderingThreadEvent::Exit => {
                    ready_to_exit.wait();
                    break 'main_loop;
                }
            }
        }
        if !rendering_data.positions.is_empty() && redraw_job_queue.is_empty() {
            let mut scenes = draw_physics(&rendering_data);
            let mut scene = scenes.remove(0);
            for subscene in scenes {
                scene.append(&subscene, None);
            }
            if rendering_data.draw_grid && !rendering_data.bvh.nodes().is_empty() {
                draw_bvh(&mut scene, rendering_data.bvh.nodes());
            }
            redraw_job_queue.force_push(scene);
            let _ = app_event_loop_proxy.send_event(AppEvent::RequestRedraw);
            rendering_result_queue.send(()).unwrap();
        }
        yield_now();
    }
}

fn draw_physics(
    RenderingData {
        positions,
        velocities,
        radii,
        colors,
        particle_range,
        planet_range,
        color_source,
        draw_ids,
        constraints,
        draw_edf,
        edf,
        edf_cell_size,
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
    let chunk_size = particle_range.len().div_ceil(16);
    let chunks = particle_range
        .clone()
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
                        let color = match color_source {
                            ColorSource::None => None,
                            ColorSource::Default => Some(css::GRAY),
                            ColorSource::Demo => colors[object_index],
                            ColorSource::Velocity => Some(color_from_velocity(velocities, object_index)),
                            ColorSource::Dark => Some(Color::new([0.2, 0.2, 0.2, 1.0])),
                        };
                        if let Some(color) = color {
                            draw_circle(
                                &mut scene,
                                transform,
                                particle_position,
                                radii[object_index].max(1.0),
                                color,
                            );
                        }

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
        zip(&positions[planet_range.clone()], &radii[planet_range.clone()]),
        &colors[planet_range.clone()],
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

    println!("edf size: {}x{}", edf.size().0, edf.size().1);

    if *draw_edf && !edf.is_empty() {
        const BYTES_PER_PIXEL: usize = 4;

        let start = Instant::now();
        let width = edf.size().0;
        let height = edf.size().1;
        let image_data_length = edf.size().1 * edf.size().0 * BYTES_PER_PIXEL;
        let mut image_data = Vec::with_capacity(image_data_length);
        image_data.resize(image_data_length, 255);
        for i in 0..width {
            for j in 0..height {
                let energy_density = edf[(i, j)];
                let energy_density_sqrt = energy_density.sqrt() as f32;
                let color = spectrum(energy_density_sqrt, 0.9 * energy_density_sqrt);
                let offset = j * width * BYTES_PER_PIXEL + i * BYTES_PER_PIXEL;
                for i in 0..4 {
                    image_data[offset + i] = (color.components[i] * 255.0) as u8;
                }
            }
        }
        println!("filling edf image took {:.02?}", start.elapsed());

        let start = Instant::now();
        let blob = Blob::new(Arc::new(image_data));
        let image = Image::new(blob, ImageFormat::Rgba8, width as u32, height as u32);
        scene.draw_image(&image, transform.pre_scale(*edf_cell_size));
        println!("rendering edf took {:.2?}", start.elapsed());
    }

    scenes
}

fn color_from_velocity(velocities: &[Vector2<f64>], object_index: usize) -> Color {
    const SCALE_FACTOR: f64 = 0.0004;
    let velocity = velocities[object_index];
    #[allow(clippy::cast_possible_truncation)]
    let spectrum_position = (velocity.magnitude() * SCALE_FACTOR).powf(0.6).clamp(0.0, 1.0) as f32;
    spectrum(spectrum_position, 1.0)
}

fn spectrum(position: f32, alpha: f32) -> Color {
    Color::new([1.0 - position, (1.0 - (position - 0.5).abs() * 2.0), position, alpha])
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

fn draw_bvh(scene: &mut Scene, bvh: &[Node]) {
    for node in bvh {
        if let NodeKind::Tree { .. } = &node.kind {
            scene.stroke(
                &Stroke::default(),
                Affine::IDENTITY,
                css::LIGHT_GRAY,
                None,
                &Rect {
                    x0: node.aabb.topleft.x,
                    y0: node.aabb.topleft.y,
                    x1: node.aabb.bottomright.x,
                    y1: node.aabb.bottomright.y,
                },
            );
        }
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

#[allow(clippy::large_enum_variant)]
#[derive(Clone)]
enum AppEvent {
    StatsUpdated(Stats),
    RequestRedraw,
    Exit,
}

impl Debug for AppEvent {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "AppEvent::")?;
        match self {
            Self::StatsUpdated(_) => write!(f, "StatsUpdated(...)"),
            Self::RequestRedraw => f.write_str("RedrawRequest"),
            Self::Exit => f.write_str("Exit"),
        }
    }
}

#[derive(Debug)]
enum SimulationThreadEvent {
    Exit,
    ToggleAdvanceTime,
    ToggleDrawIds,
    ToggleDrawGrid,
    SetColorSource(ColorSource),
    SetGpuComputeOptions(GpuComputeOptions),
    UnidirectionalKick {
        mouse_position: Vector2<f64>,
        mouse_influence_radius: f64,
    },
    ToggleDrawEdf,
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
    particle_range: Range<usize>,
    planet_range: Range<usize>,
    color_source: ColorSource,
    draw_ids: bool,
    draw_grid: bool,
    constraints: AABB,
    draw_edf: bool,
    edf: Array2<f64>,
    edf_cell_size: f64,
    bvh: Bvh,
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
    rendering_event_queue: &'s SegQueue<RenderingThreadEvent>,
    stats: Stats,
    ready_to_exit: Arc<Barrier>,
    gpu_compute_options: GpuComputeOptions,
    redraw_job_queue: &'s ArrayQueue<Scene>,
    redraw_result_queue: &'s ArrayQueue<()>,
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
                .create_surface(window.clone(), size.width, size.height, PresentMode::AutoVsync);
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

    // TODO fix hang on exit

    fn window_event(&mut self, event_loop: &ActiveEventLoop, window_id: WindowId, event: WindowEvent) {
        let Some(render_state) = &mut self.state else {
            return;
        };
        if render_state.window.id() != window_id {
            return;
        }
        match event {
            WindowEvent::CloseRequested => {
                self.simulation_event_sender
                    .send(SimulationThreadEvent::Exit)
                    .expect("failed to send simulation thread Exit event");
                self.rendering_event_queue.push(RenderingThreadEvent::Exit);
                self.ready_to_exit.wait();
                event_loop.exit();
            }
            WindowEvent::KeyboardInput { event, .. } => {
                if event.state == ElementState::Pressed {
                    match event.logical_key.as_ref() {
                        Key::Named(NamedKey::Escape) => {
                            self.simulation_event_sender
                                .send(SimulationThreadEvent::Exit)
                                .expect("failed to send simulation thread Exit event");
                            self.rendering_event_queue.push(RenderingThreadEvent::Exit);
                            self.ready_to_exit.wait();
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
                        Key::Character("1") => {
                            self.simulation_event_sender
                                .send(SimulationThreadEvent::SetColorSource(ColorSource::None))
                                .unwrap();
                        }
                        Key::Character("2") => {
                            self.simulation_event_sender
                                .send(SimulationThreadEvent::SetColorSource(ColorSource::Default))
                                .unwrap();
                        }
                        Key::Character("3") => {
                            self.simulation_event_sender
                                .send(SimulationThreadEvent::SetColorSource(ColorSource::Demo))
                                .unwrap();
                        }
                        Key::Character("4") => {
                            self.simulation_event_sender
                                .send(SimulationThreadEvent::SetColorSource(ColorSource::Velocity))
                                .unwrap();
                        }
                        Key::Character("5") => {
                            self.simulation_event_sender
                                .send(SimulationThreadEvent::SetColorSource(ColorSource::Dark))
                                .unwrap();
                        }
                        Key::Character("l") => {
                            self.gpu_compute_options.integration = !self.gpu_compute_options.integration;
                            self.simulation_event_sender
                                .send(SimulationThreadEvent::SetGpuComputeOptions(self.gpu_compute_options))
                                .unwrap();
                        }
                        Key::Character("p") => {
                            self.gpu_compute_options.bvh = !self.gpu_compute_options.bvh;
                            self.simulation_event_sender
                                .send(SimulationThreadEvent::SetGpuComputeOptions(self.gpu_compute_options))
                                .unwrap();
                        }
                        Key::Character("e") => {
                            self.simulation_event_sender
                                .send(SimulationThreadEvent::ToggleDrawEdf)
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
                    if let Some(scene) = self.redraw_job_queue.pop() {
                        self.simulation_scene = scene;
                    }
                    if let Some(fps) = self.fps_calculator.update(self.frame_count) {
                        self.last_fps = fps;
                        self.min_fps = self.min_fps.min(fps);
                    }
                    self.scene.reset();
                    self.scene.fill(
                        Fill::NonZero,
                        Affine::IDENTITY,
                        css::BLACK,
                        None,
                        &Rect::new(0.0, 0.0, CONFIG.window.width as f64, CONFIG.window.height as f64),
                    );
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
                    self.redraw_result_queue.force_push(());
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
            AppEvent::StatsUpdated(stats) => {
                self.stats = stats;
                request_redraw(self.state.as_ref());
            }
            AppEvent::RequestRedraw => request_redraw(self.state.as_ref()),
            AppEvent::Exit => {
                self.ready_to_exit.wait();
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

fn write_stats(
    buffer: &mut String,
    (fps, min_fps): (usize, usize),
    Stats {
        sim_time,
        object_count,
        integration_duration,
        bvh_duration,
        collisions_duration,
        constraints_duration,
        total_duration: _,
    }: &Stats,
    gpu_compute_options: GpuComputeOptions,
) -> anyhow::Result<()> {
    writeln!(buffer, "FPS: {fps} (min {min_fps})")?;
    write!(buffer, "sim time: {}", sim_time)?;
    if let Some(time_limit) = CONFIG.simulation.time_limit {
        let action = CONFIG.simulation.time_limit_action.to_string();
        write!(buffer, " ({action} at {time_limit})")?;
    }
    writeln!(buffer)?;

    const FLAG_NAMES: [&str; 2] = ["off", "on"];
    writeln!(
        buffer,
        "gpu compute: integration {}, bvh {}",
        FLAG_NAMES[gpu_compute_options.integration as usize], FLAG_NAMES[gpu_compute_options.bvh as usize]
    )?;
    writeln!(buffer, "objects: {}", object_count)?;
    write_duration_stat(buffer, "integration", integration_duration)?;
    write_duration_stat(buffer, "collision", collisions_duration)?;
    write_duration_stat(buffer, "bvh", bvh_duration)?;
    write_duration_stat(buffer, "constraints", constraints_duration)?;
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
        "{}: {:.2?} (min {:.2?}, max {:.2?}, avg {:.2?})",
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
