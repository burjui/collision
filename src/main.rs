use std::{num::NonZero, path::Path, sync::Arc, time::Instant};

use anyhow::{Context, Ok};
use collision::{app_config::AppConfig, fps::FpsCalculator, physics::PhysicsEngine, vector2::Vector2};
use env_logger::TimestampPrecision;
use rayon::iter::{IndexedParallelIterator, IntoParallelIterator, ParallelIterator};
use scene::create_scene;
use vello::{
    kurbo::{Affine, Circle, Point},
    peniko::{color::palette::css, Color, Fill},
    util::{RenderContext, RenderSurface},
    wgpu::{Maintain, PresentMode},
    AaConfig, AaSupport, RenderParams, Renderer, RendererOptions, Scene,
};
use winit::{
    application::ApplicationHandler,
    dpi::PhysicalSize,
    event::{ElementState, WindowEvent},
    event_loop::{ActiveEventLoop, EventLoop},
    keyboard::{Key, NamedKey},
    window::{Window, WindowId},
};

mod scene;

pub fn main() -> anyhow::Result<()> {
    extern "C" {
        fn feenableexcept(excepts: i32) -> i32;
    }
    unsafe {
        feenableexcept(1);
    }

    env_logger::builder()
        .format_timestamp(Some(TimestampPrecision::Millis))
        .init();

    let config = AppConfig::from_file(Path::new("config.toml")).context("load config")?;
    let event_loop = EventLoop::new()?;
    let render_context = RenderContext::new();
    let renderers: Vec<Option<Renderer>> = vec![];
    let render_state = None::<RenderState<'_>>;

    let mut physics = PhysicsEngine::new(&config)?;
    create_scene(&mut physics);

    let mut app = VelloApp {
        config,
        context: render_context,
        renderers,
        state: render_state,
        cached_window: None,
        scene: Scene::new(),
        frame_count: 0,
        fps_calculator: FpsCalculator::new(),
        fps: None,
        min_fps: usize::MAX,
        physics,
        advance_time: false,
        mouse_position: Vector2::new(0.0, 0.0),
        last_frame_time: 0.0,
        output_frame_count: 0,
    };
    event_loop.run_app(&mut app).expect("run to completion");
    Ok(())
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
    fps: Option<usize>,
    min_fps: usize,
    physics: PhysicsEngine,
    advance_time: bool,
    mouse_position: Vector2<f64>,
    last_frame_time: f64,
    output_frame_count: usize,
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
            let render_state = RenderState { window, surface };
            self.renderers.resize_with(self.context.devices.len(), || None);
            let id = render_state.surface.dev_id;
            self.renderers[id].get_or_insert_with(|| {
                let start = Instant::now();
                let renderer = Renderer::new(
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
                .expect("Failed to create renderer");
                log::info!("Creating renderer {id} took {:?}", start.elapsed());
                renderer
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
                        _ => {}
                    }
                }
            }
            WindowEvent::Resized(size) => {
                if let Some(RenderState { surface, window }) = &mut self.state {
                    self.context.resize_surface(surface, size.width, size.height);
                    window.request_redraw();
                };
            }
            WindowEvent::RedrawRequested => {
                self.scene.reset();
                self.render_physics();
                self.frame_count += 1;

                self.fps = self.fps_calculator.update(self.frame_count).or(self.fps);
                if let Some(fps) = self.fps {
                    self.min_fps = self.min_fps.min(fps);
                    log::info!("FPS: {fps} (min {})", self.min_fps);
                    log::info!("Sim time: {}", self.physics.time());
                }
            }
            _ => {}
        }
    }

    fn about_to_wait(&mut self, _event_loop: &ActiveEventLoop) {
        const FRAME_INTERVAL: f64 = 1.0 / 60.0;
        const DEFAULT_DT: f64 = FRAME_INTERVAL / 128.0;

        if self.advance_time {
            self.physics.advance(DEFAULT_DT, 2);
        }

        if let Some(render_state) = &mut self.state {
            render_state.window.request_redraw();
        }
    }

    fn suspended(&mut self, _event_loop: &ActiveEventLoop) {
        log::info!("Suspending");
        // When we suspend, we need to remove the `wgpu` Surface
        if let Some(render_state) = self.state.take() {
            self.cached_window = Some(render_state.window);
        }
    }
}

impl VelloApp<'_> {
    fn render_physics(&mut self) {
        let Some(RenderState { surface, .. }) = &self.state else {
            return;
        };

        for object in self.physics.objects() {
            self.scene.fill(
                Fill::NonZero,
                Affine::IDENTITY,
                Color::new([0.9529, 0.5451, 0.6588, 1.0]),
                None,
                &Circle::new(Point::new(object.position.x, object.position.y), object.radius),
            );
        }

        // const WIDTH: usize = 200;
        // const HEIGHT: usize = 200;
        // const N_CHUNKS: usize = 20;

        // let chunk_size = WIDTH / N_CHUNKS;
        // let mut subscenes = Vec::with_capacity(N_CHUNKS);
        // (0..N_CHUNKS)
        //     .into_par_iter()
        //     .map(|chunk_index| {
        //         let mut scene = Scene::new();
        //         for i in chunk_index * chunk_size..(chunk_index + 1) * chunk_size {
        //             for j in 0..HEIGHT {
        //                 let radius = 4.0;
        //                 let x = i as f64 * radius * 2.0;
        //                 let y = j as f64 * radius * 2.0;
        //                 scene.fill(
        //                     Fill::NonZero,
        //                     Affine::IDENTITY,
        //                     Color::new([0.9529, 0.5451, 0.6588, 1.]),
        //                     None,
        //                     &Circle::new((x, y), radius),
        //                 );
        //             }
        //         }
        //         scene
        //     })
        //     .collect_into_vec(&mut subscenes);
        // for scene in subscenes {
        //     self.scene.append(&scene, None);
        // }

        let width = surface.config.width;
        let height = surface.config.height;
        let device_handle = &self.context.devices[surface.dev_id];
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
        let renderer = self.renderers[surface.dev_id].as_mut().unwrap();
        renderer
            .render_to_surface(
                &device_handle.device,
                &device_handle.queue,
                &self.scene,
                &surface_texture,
                &render_params,
            )
            .expect("failed to render to surface");
        surface_texture.present();
        device_handle.device.poll(Maintain::Poll);
    }
}
