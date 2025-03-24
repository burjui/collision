#![allow(clippy::struct_excessive_bools)]

use std::{fmt::Display, fs::File, io::Read, path::Path, sync::LazyLock};

use anyhow::{Context, anyhow};
use serde_derive::Deserialize;

pub static CONFIG: LazyLock<AppConfig> = LazyLock::new(|| {
    AppConfig::from_file(Path::new("config.toml"))
        .context("load config")
        .unwrap()
});

#[derive(Deserialize, Clone, Copy)]
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
        validate_positive_u32(self.window.width, "window.width")?;
        validate_positive_u32(self.window.height, "window.height")?;
        validate_positive_f64(self.simulation.speed_factor, "simulation.base_dt")?;
        if let Some(time_limit) = self.simulation.time_limit {
            validate_positive_f64(time_limit, "simulation.time_limit")?;
        }
        validate_restitution_coefficient(
            self.simulation.restitution_coefficient,
            "simulation.restitution_coefficient",
        )?;
        validate_positive_f64(self.demo.object_radius, "demo.object_radius")?;
        Ok(())
    }
}

fn validate_positive_u32(value: u32, name: &'static str) -> anyhow::Result<()> {
    if value > 0 {
        Ok(())
    } else {
        Err(anyhow!("{} must be positive", name))
    }
}

fn validate_positive_f64(value: f64, name: &'static str) -> anyhow::Result<()> {
    if value > 0.0 {
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
    #[serde(default = "default_gpu_integration")]
    pub gpu_integration: bool,
    pub restitution_coefficient: f64,
    pub gravity: (f64, f64),
    pub gravitational_constant: f64,
    pub time_limit: Option<f64>,
    #[serde(default)]
    pub time_limit_action: TimeLimitAction,
}

fn default_speed_factor() -> f64 {
    1.0
}

fn default_gpu_integration() -> bool {
    false
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

#[derive(Deserialize, Clone, Copy)]
#[serde(deny_unknown_fields)]
pub struct DemoConfig {
    pub object_radius: f64,
    pub object_spacing: f64,

    #[serde(default)]
    pub enable_planets: bool,

    #[serde(default)]
    pub enable_brick: bool,

    #[serde(default)]
    pub enable_ball: bool,

    #[serde(default)]
    pub randomize_positions: bool,
    pub randomize_position_factor: f64,
}

#[derive(Deserialize, Clone, Copy)]
#[serde(deny_unknown_fields)]
pub struct RenderConfig {
    #[serde(default = "default_color_source")]
    pub color_source: ColorSource,
}

fn default_color_source() -> ColorSource {
    ColorSource::Velocity
}

#[derive(Debug, Deserialize, Clone, Copy, Default)]
pub enum ColorSource {
    #[default]
    #[serde(rename = "demo")]
    Demo,

    #[serde(rename = "velocity")]
    Velocity,
}
