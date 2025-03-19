use std::{fs::File, io::Read, path::Path};

use anyhow::{anyhow, Context};
use serde_derive::Deserialize;

#[derive(Deserialize)]
pub struct WindowConfig {
    pub width: u32,
    pub height: u32,
}

#[derive(Deserialize)]
pub struct SimulationConfig {
    pub speed_factor: f32,
    pub restitution_coefficient: f32,
    pub gravity: (f32, f32),
    pub time_limit: Option<f32>,
}

#[derive(Deserialize)]
pub struct DemoConfig {
    pub object_radius: f32,

    #[serde(default)]
    pub enable_planets: bool,

    #[serde(default)]
    pub enable_brick: bool,

    #[serde(default)]
    pub enable_ball: bool,
}

#[derive(Deserialize, Clone, Copy)]
pub struct RenderConfig {
    pub color_source: ColorSource,
}

#[derive(Deserialize, Clone, Copy)]
pub enum ColorSource {
    #[serde(rename = "demo")]
    Demo,

    #[serde(rename = "velocity")]
    Velocity,
}

#[derive(Deserialize)]
pub struct AppConfig {
    pub window: WindowConfig,
    pub simulation: SimulationConfig,
    pub demo: DemoConfig,
    pub rendering: RenderConfig,
}

impl AppConfig {
    pub fn from_file(config_path: &Path) -> anyhow::Result<AppConfig> {
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
        validate_positive_f32(self.simulation.speed_factor, "simulation.base_dt")?;
        validate_restitution_coefficient(
            self.simulation.restitution_coefficient,
            "simulation.restitution_coefficient",
        )?;
        validate_positive_f32(self.demo.object_radius, "demo.object_radius")?;
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

fn validate_positive_f32(value: f32, name: &'static str) -> anyhow::Result<()> {
    if value > 0.0 {
        Ok(())
    } else {
        Err(anyhow!("{} must be positive", name))
    }
}

fn validate_restitution_coefficient(value: f32, name: &'static str) -> anyhow::Result<()> {
    if (0.0..=1.0).contains(&value) {
        Ok(())
    } else {
        Err(anyhow!("{} must be in range [0.0, 1.0]", name))
    }
}
