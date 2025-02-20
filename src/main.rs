use std::{num::NonZero, sync::Arc, time::Instant};

use rayon::iter::{IndexedParallelIterator, IntoParallelIterator, ParallelIterator};
use vello::{
    kurbo::{Affine, Circle},
    peniko::{color::palette, Color},
    util::{RenderContext, RenderSurface},
    wgpu::{self, PresentMode},
    AaConfig, AaSupport, Renderer, RendererOptions, Scene,
};
use winit::{
    application::ApplicationHandler,
    dpi::LogicalSize,
    event::{ElementState, WindowEvent},
    event_loop::EventLoop,
    keyboard::{Key, NamedKey},
    window::Window,
};

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
    transform: Affine,
    frame_count: usize,
    start: Instant,
}

impl ApplicationHandler<()> for VelloApp<'_> {
    fn resumed(&mut self, event_loop: &winit::event_loop::ActiveEventLoop) {
        if self.state.is_some() {
            return;
        }
        let window = self.cached_window.take().unwrap_or_else(|| {
            Arc::new(
                event_loop
                    .create_window(
                        Window::default_attributes()
                            .with_inner_size(LogicalSize::new(1044, 800))
                            .with_resizable(true)
                            .with_title("Vello demo"),
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

    fn window_event(
        &mut self,
        event_loop: &winit::event_loop::ActiveEventLoop,
        window_id: winit::window::WindowId,
        event: WindowEvent,
    ) {
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
                        Key::Named(NamedKey::Space) => self.transform = Affine::IDENTITY,
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
                self.render_scene();

                let fps = self.frame_count as f32 / (Instant::now() - self.start).as_secs_f32();
                eprintln!("Rendered {fps:.2} fps");
                self.frame_count += 1;
            }
            _ => {}
        }
    }

    fn about_to_wait(&mut self, _event_loop: &winit::event_loop::ActiveEventLoop) {
        if let Some(render_state) = &mut self.state {
            render_state.window.request_redraw();
        }
    }

    fn suspended(&mut self, _event_loop: &winit::event_loop::ActiveEventLoop) {
        log::info!("Suspending");
        // When we suspend, we need to remove the `wgpu` Surface
        if let Some(render_state) = self.state.take() {
            self.cached_window = Some(render_state.window);
        }
    }
}

impl VelloApp<'_> {
    fn render_scene(&mut self) {
        const WIDTH: usize = 200;
        const HEIGHT: usize = 200;
        const N_CHUNKS: usize = 20;

        let chunk_size = WIDTH / N_CHUNKS;
        let mut subscenes = Vec::with_capacity(N_CHUNKS);
        (0..N_CHUNKS)
            .into_par_iter()
            .map(|chunk_index| {
                let mut scene = Scene::new();
                for i in chunk_index * chunk_size..(chunk_index + 1) * chunk_size {
                    for j in 0..HEIGHT {
                        let radius = 4.0;
                        let x = i as f64 * radius * 2.0;
                        let y = j as f64 * radius * 2.0;
                        scene.fill(
                            vello::peniko::Fill::NonZero,
                            Affine::IDENTITY,
                            Color::new([0.9529, 0.5451, 0.6588, 1.]),
                            None,
                            &Circle::new((x, y), radius),
                        );
                    }
                }
                scene
            })
            .collect_into_vec(&mut subscenes);
        self.scene.reset();
        for scene in subscenes {
            self.scene.append(&scene, None);
        }

        let Some(RenderState { surface, .. }) = &self.state else {
            return;
        };
        let width = surface.config.width;
        let height = surface.config.height;
        let device_handle = &self.context.devices[surface.dev_id];
        let surface_texture = surface
            .surface
            .get_current_texture()
            .expect("failed to get surface texture");
        let render_params = vello::RenderParams {
            base_color: palette::css::BLACK,
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
        device_handle.device.poll(wgpu::Maintain::Poll);
    }
}

pub fn main() -> anyhow::Result<()> {
    env_logger::builder()
        .format_timestamp(Some(env_logger::TimestampPrecision::Millis))
        .filter_level(log::LevelFilter::Warn)
        .init();
    let event_loop = EventLoop::new()?;
    let render_context = RenderContext::new();
    let renderers: Vec<Option<Renderer>> = vec![];
    let render_state = None::<RenderState<'_>>;
    let mut app = VelloApp {
        context: render_context,
        renderers,
        state: render_state,
        cached_window: None,
        scene: Scene::new(),
        transform: Affine::IDENTITY,
        frame_count: 0,
        start: Instant::now(),
    };
    event_loop.run_app(&mut app).expect("run to completion");
    Ok(())
}
