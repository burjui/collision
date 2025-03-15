#![feature(more_float_constants)]

use core::f32;
use std::{num::NonZero, path::Path, process::exit, sync::Arc};

use anyhow::{Context, Ok};
use collision::{app_config::AppConfig, fps::FpsCalculator, physics::PhysicsEngine, vector2::Vector2};
use demo::create_demo;
use itertools::Itertools;
use libc::EXIT_SUCCESS;
use vello::{
    kurbo::{Affine, Circle, Rect, Stroke},
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

    let config = AppConfig::from_file(Path::new("config.toml")).context("load config")?;
    let event_loop = EventLoop::new()?;
    let render_context = RenderContext::new();
    let renderers: Vec<Option<Renderer>> = vec![];
    let render_state = None::<RenderState<'_>>;

    let mut physics = PhysicsEngine::new(&config)?;
    create_demo(&mut physics);

    let advance_time = config.sim_time_limit.is_some();
    let mut app = VelloApp {
        config,
        context: render_context,
        renderers,
        state: render_state,
        cached_window: None,
        scene: Scene::new(),
        frame_count: 0,
        fps_calculator: FpsCalculator::default(),
        last_fps: None,
        min_fps: usize::MAX,
        physics,
        advance_time,
        mouse_position: Vector2::new(0.0, 0.0),
        mouse_influence_radius: 50.0,
    };
    event_loop.run_app(&mut app).expect("run to completion");
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
    config: AppConfig,
    context: RenderContext,
    renderers: Vec<Option<Renderer>>,
    state: Option<RenderState<'s>>,
    // Whilst suspended, we drop `render_state`, but need to keep the same window.
    // If render_state exists, we must store the window in it, to maintain drop order
    cached_window: Option<Arc<Window>>,
    scene: Scene,
    frame_count: usize,
    fps_calculator: FpsCalculator,
    last_fps: Option<usize>,
    min_fps: usize,
    physics: PhysicsEngine,
    advance_time: bool,
    mouse_position: Vector2<f32>,
    mouse_influence_radius: f32,
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
                            .with_inner_size(PhysicalSize::new(self.config.width, self.config.height))
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
                .map_err(|e| {
                    // Pretty-print any renderer creation error using Display formatting before unwrapping.
                    anyhow::format_err!("{e}")
                })
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
                        Key::Named(NamedKey::Space) => self.advance_time = !self.advance_time,
                        Key::Named(NamedKey::Escape) => event_loop.exit(),
                        Key::Character("a") => self.physics.enable_gpu = !self.physics.enable_gpu,
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
                    self.scene.reset();
                    render_physics(&self.physics, &mut self.scene);
                    render_mouse_influence(&mut self.scene, self.mouse_position, self.mouse_influence_radius);
                    let renderer = self.renderers[surface.dev_id].as_mut().expect("failed to get renderer");
                    let device_handle = &self.context.devices[surface.dev_id];
                    render_scene(&self.scene, surface, renderer, device_handle);
                    self.frame_count += 1;

                    self.last_fps = self.fps_calculator.update(self.frame_count).or(self.last_fps);
                    if let Some(fps) = self.last_fps {
                        self.min_fps = self.min_fps.min(fps);
                        eprintln!("FPS: {fps} (min {})", self.min_fps);
                        eprintln!("Sim time: {}", self.physics.time());
                    }
                }
            }
            WindowEvent::CursorMoved { position, .. } => {
                self.mouse_position = Vector2::new(position.x as f32, position.y as f32);
            }
            WindowEvent::MouseInput { state, button, .. } => {
                if state == ElementState::Pressed {
                    match button {
                        MouseButton::Left => {
                            for object in self.physics.objects_mut() {
                                let from_mouse_to_object = object.position - self.mouse_position;
                                if (from_mouse_to_object).magnitude() < self.mouse_influence_radius {
                                    object.velocity += from_mouse_to_object.normalize() * 2000.0;
                                }
                            }
                        }
                        _ => {}
                    }
                }
            }
            WindowEvent::MouseWheel { delta, .. } => {
                if let MouseScrollDelta::LineDelta(_, dy) = delta {
                    self.mouse_influence_radius = (self.mouse_influence_radius + dy * 3.0).max(0.0);
                }
            }
            _ => {}
        }
    }

    fn about_to_wait(&mut self, _event_loop: &ActiveEventLoop) {
        const FRAME_INTERVAL: f32 = 1.0 / 60.0;
        const DEFAULT_DT: f32 = FRAME_INTERVAL / 64.0;

        if self
            .config
            .sim_time_limit
            .is_some_and(|limit| self.physics.time() > limit)
        {
            exit(EXIT_SUCCESS);
        }

        if self.advance_time {
            self.physics.advance(DEFAULT_DT);
        }

        if let Some(render_state) = &mut self.state {
            render_state.window.request_redraw();
        }
    }

    fn suspended(&mut self, _event_loop: &ActiveEventLoop) {
        // When we suspend, we need to remove the `wgpu` Surface
        if let Some(render_state) = self.state.take() {
            self.cached_window = Some(render_state.window);
        }
    }
}

fn render_physics(physics: &PhysicsEngine, scene: &mut Scene) {
    let transform = Affine::IDENTITY;

    std::thread::scope(|scope| {
        let handles = physics
            .objects()
            .chunks(physics.objects().len() / 20)
            .map(|chunk| {
                scope.spawn(move || {
                    let mut scene = Scene::new();
                    for object in chunk {
                        if !object.is_planet {
                            scene.fill(
                                Fill::NonZero,
                                transform,
                                object.color.unwrap_or_else(|| {
                                    const SCALE_FACTOR: f32 = 0.0004;
                                    let parameter = (object.velocity.magnitude() * SCALE_FACTOR).powf(0.6);
                                    let spectrum_position = parameter.clamp(0.0, 1.0);
                                    spectrum(spectrum_position)
                                }),
                                None,
                                &Circle::new((object.position.x, object.position.y), object.radius.into()),
                            );
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

    for planet in physics.planets() {
        scene.fill(
            Fill::NonZero,
            transform,
            planet.color.unwrap_or(css::WHITE),
            None,
            &Circle::new((planet.position.x, planet.position.y), (planet.radius * 3.0).into()),
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

fn render_mouse_influence(scene: &mut Scene, mouse_position: Vector2<f32>, mouse_influence_radius: f32) {
    scene.fill(
        Fill::NonZero,
        Affine::IDENTITY,
        Color::new([1.0, 1.0, 1.0, 0.3]),
        None,
        &Circle::new((mouse_position.x, mouse_position.y), mouse_influence_radius as f64),
    );
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
