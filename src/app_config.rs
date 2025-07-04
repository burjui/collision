#![allow(clippy::struct_excessive_bools)]

use std::{fmt::Display, fs::File, io::Read, path::Path, sync::LazyLock};

use anyhow::{Context, anyhow};
use num_traits::Num;
use serde_derive::Deserialize;

use crate::demo::{Ball, Brick};

pub static CONFIG: LazyLock<AppConfig> =
    LazyLock::new(|| AppConfig::from_file(Path::new("config.toml")).context("load config").unwrap());

#[derive(Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct AppConfig {
    pub window: WindowConfig,
    pub simulation: SimulationConfig,
    pub demo: DemoConfig,
    pub rendering: RenderConfig,
}

impl AppConfig {
    fn from_file(config_path: &Path) -> anyhow::Result<AppConfig> {
        let mut config_file =
            File::open(config_path).context(format!("open config \"{}\"", config_path.to_string_lossy()))?;
        let mut config_string = String::new();
        config_file.read_to_string(&mut config_string).context("read config")?;
        let config: AppConfig = toml::from_str(&config_string).context("parse config")?;
        config.validate().context("validate config")?;
        Ok(config)
    }

    fn validate(&self) -> anyhow::Result<()> {
        validate_positive(self.window.width, "window.width")?;
        validate_positive(self.window.height, "window.height")?;

        if let DtSource::Fixed(dt) = self.simulation.dt {
            validate_positive(dt, "simulation.dt")?;
        }
        validate_positive(self.simulation.speed_factor, "simulation.speed_factor")?;
        if let Some(time_limit) = self.simulation.time_limit {
            validate_positive(time_limit, "simulation.time_limit")?;
        }
        validate_positive(self.simulation.gpu_integration_local_wg_size, "simulation.gpu_integration_local_wg_size")?;
        validate_positive(self.simulation.gpu_bvh_local_wg_size, "simulation.gpu_bvh_local_wg_size")?;
        validate_restitution_coefficient(
            self.simulation.restitution_coefficient,
            "simulation.restitution_coefficient",
        )?;

        validate_positive(self.demo.object_radius, "demo.object_radius")?;
        validate_positive(self.demo.randomize_position_factor, "demo.randomize_position_factor")?;

        for brick in &self.demo.bricks {
            validate_positive(brick.size.x, "brick width")?;
            validate_positive(brick.size.y, "brick height")?;
            validate_positive(brick.particle_radius, "brick particle radius")?;
            validate_non_negative(brick.particle_spacing, "brick particle spacing")?;
            validate_positive(brick.particle_mass, "brick particle mass")?;
        }

        for ball in &self.demo.balls {
            validate_positive(ball.radius, "ball radius")?;
            validate_positive(ball.particle_radius, "ball particle radius")?;
            validate_non_negative(ball.particle_spacing, "ball particle spacing")?;
            validate_positive(ball.particle_mass, "ball particle mass")?;
        }

        Ok(())
    }
}

fn validate_positive<T: Num + PartialOrd>(value: T, name: &'static str) -> anyhow::Result<()> {
    if value > T::zero() {
        Ok(())
    } else {
        Err(anyhow!("{} must be positive", name))
    }
}

fn validate_non_negative<T: Num + PartialOrd>(value: T, name: &'static str) -> anyhow::Result<()> {
    if value >= T::zero() {
        Ok(())
    } else {
        Err(anyhow!("{} must be positive", name))
    }
}

fn validate_restitution_coefficient(value: f64, name: &'static str) -> anyhow::Result<()> {
    if (0.0..=1.0).contains(&value) {
        Ok(())
    } else {
        Err(anyhow!("{} must be in range [0.0, 1.0]", name))
    }
}

#[derive(Deserialize, Clone, Copy)]
#[serde(deny_unknown_fields)]
pub struct WindowConfig {
    pub width: u32,
    pub height: u32,
}

#[derive(Deserialize, Clone, Copy)]
#[serde(deny_unknown_fields)]
pub struct SimulationConfig {
    #[serde(default)]
    pub auto_start: bool,
    #[serde(default)]
    pub dt: DtSource,
    #[serde(default = "default_speed_factor")]
    pub speed_factor: f64,
    #[serde(default)]
    pub gpu_integration: bool,
    #[serde(default)]
    pub gpu_bvh: bool,
    #[serde(default = "default_wg_size")]
    pub gpu_integration_local_wg_size: usize,
    #[serde(default = "default_wg_size")]
    pub gpu_bvh_local_wg_size: usize,
    pub restitution_coefficient: f64,
    #[serde(default)]
    pub global_gravity: (f64, f64),
    pub gravitational_constant: f64,
    pub time_limit: Option<f64>,
    #[serde(default)]
    pub time_limit_action: TimeLimitAction,
}

fn default_speed_factor() -> f64 {
    1.0
}

fn default_wg_size() -> usize {
    32
}

#[derive(Deserialize, Clone, Copy, Default)]
#[serde(deny_unknown_fields)]
pub enum DtSource {
    #[default]
    #[serde(rename = "auto")]
    Auto,

    #[serde(rename = "fixed")]
    Fixed(f64),
}

#[derive(Deserialize, Clone, Copy, Default)]
pub enum TimeLimitAction {
    #[default]
    #[serde(rename = "exit")]
    Exit,

    #[serde(rename = "pause")]
    Pause,
}

impl Display for TimeLimitAction {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            TimeLimitAction::Exit => f.write_str("exit"),
            TimeLimitAction::Pause => f.write_str("pause"),
        }
    }
}

#[derive(Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct DemoConfig {
    pub object_radius: f64,

    #[serde(default)]
    pub enable_planets: bool,

    #[serde(default)]
    pub randomize_positions: bool,
    #[serde(default)]
    pub randomize_position_factor: f64,
    #[serde(default)]
    pub randomize_radii: bool,
    #[serde(default)]
    pub randomize_radius_factor: f64,

    #[serde(default)]
    pub bricks: Vec<Brick>,

    #[serde(default)]
    pub balls: Vec<Ball>,
}

#[derive(Deserialize, Clone, Copy)]
#[serde(deny_unknown_fields)]
pub struct RenderConfig {
    #[serde(default = "default_rendering_enabled")]
    pub enabled: bool,

    #[serde(rename = "color", default = "default_color_source")]
    pub color_source: ColorSource,

    #[serde(default)]
    pub show_edf: bool,
}

fn default_rendering_enabled() -> bool {
    true
}

fn default_color_source() -> ColorSource {
    ColorSource::Velocity
}

#[derive(Debug, Deserialize, Clone, Copy, Default)]
pub enum ColorSource {
    #[serde(rename = "none")]
    None,

    #[default]
    #[serde(rename = "default")]
    Default,

    #[serde(rename = "demo")]
    Demo,

    #[serde(rename = "velocity")]
    Velocity,

    #[serde(rename = "dark")]
    Dark,
}
